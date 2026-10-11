#include "PushPullCore.h"

#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <Preferences.h>

// ======================== ВНУТРЕННЕЕ СОСТОЯНИЕ ========================
namespace {

PushPullConfig g_cfg = {};
bool           g_initialized = false;

// --- MCPWM ---
mcpwm_timer_handle_t timer       = nullptr;
mcpwm_oper_handle_t  oper_a      = nullptr, oper_b      = nullptr;
mcpwm_cmpr_handle_t  cmpr_a1     = nullptr, cmpr_a2     = nullptr;
mcpwm_cmpr_handle_t  cmpr_b1     = nullptr, cmpr_b2     = nullptr;
mcpwm_gen_handle_t   gen_a       = nullptr, gen_b       = nullptr;
mcpwm_fault_handle_t gpio_fault  = nullptr;

uint32_t g_half_period = 0;

// --- ADC ---
adc_oneshot_unit_handle_t adc_handle        = nullptr;
adc_cali_handle_t         adc_cali_v_handle = nullptr;
adc_cali_handle_t         adc_cali_i_handle = nullptr;
bool                      adc_cali_v_ok     = false;
bool                      adc_cali_i_ok     = false;

// --- Состояние ---
volatile SystemState     g_state           = STATE_INIT;
volatile SetpointSource  g_source          = SRC_FIXED;
volatile uint32_t        g_freq_hz         = 50000;
volatile float           g_dead_time_ns    = 1000.0f;
volatile uint32_t        g_dt_ticks        = 0;
volatile float           g_soft_start_duty = 60.0f;

volatile float g_target_voltage_v = 12.0f;
volatile float g_trip_current_ma  = 5000.0f;
volatile float g_pid_kp           = 2.0f;
volatile float g_pid_ki           = 0.09f;

volatile float g_measured_voltage_v = 0.0f;
volatile float g_measured_current_a = 0.0f;

float         g_last_valid_current_a = 0.0f;
volatile bool g_soft_start_running   = false;

SemaphoreHandle_t g_pwm_mutex = nullptr;
SemaphoreHandle_t g_nvs_mutex = nullptr;

Preferences g_prefs;
uint32_t    g_last_save_ms = 0;

// ======================== ШИМ ========================
void set_pushpull_duty_raw(float duty_pct) {
    if (duty_pct > 100.0f) duty_pct = 100.0f;
    if (duty_pct < 0.0f)   duty_pct = 0.0f;

    const uint32_t dt = g_dt_ticks;
    int32_t max_phase_duration = (int32_t)(g_half_period - dt);
    if (max_phase_duration <= 0) return;

    uint32_t width = (uint32_t)roundf((float)max_phase_duration * (duty_pct / 100.0f));

    uint32_t center_a = g_half_period / 2;
    uint32_t center_b = g_half_period + center_a;

    uint32_t half_w = width / 2;
    uint32_t rem_w  = width % 2;

    mcpwm_comparator_set_compare_value(cmpr_a1, center_a - half_w);
    mcpwm_comparator_set_compare_value(cmpr_a2, center_a + half_w + rem_w);
    mcpwm_comparator_set_compare_value(cmpr_b1, center_b - half_w);
    mcpwm_comparator_set_compare_value(cmpr_b2, center_b + half_w + rem_w);
}

bool safe_set_duty(float duty_pct) {
    if (g_pwm_mutex == nullptr) return false;
    if (xSemaphoreTake(g_pwm_mutex, portMAX_DELAY) != pdTRUE) return false;

    if (g_state == STATE_FAULT_ACTIVE) duty_pct = 0.0f;
    set_pushpull_duty_raw(duty_pct);

    xSemaphoreGive(g_pwm_mutex);
    return true;
}

bool is_fault_active_hardware() {
    return digitalRead(g_cfg.pin_fault) == HIGH;
}

// ======================== ADC ========================
bool adc_calibration_init_channel(adc_channel_t ch, adc_cali_handle_t* out) {
    *out = nullptr;

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg = {};
    cfg.unit_id  = ADC_UNIT_1;
    cfg.atten    = ADC_ATTEN_DB_12;
    cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cfg, out);
    if (ret == ESP_OK) {
        Serial.printf("[ADC] line fitting OK for ch %d\n", (int)ch);
        return true;
    }
    Serial.printf("[ADC] line fitting failed for ch %d: %s\n",
                  (int)ch, esp_err_to_name(ret));
#endif

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {};
    cfg.unit_id  = ADC_UNIT_1;
    cfg.chan     = ch;
    cfg.atten    = ADC_ATTEN_DB_12;
    cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cfg, out);
    if (ret == ESP_OK) {
        Serial.printf("[ADC] curve fitting OK for ch %d\n", (int)ch);
        return true;
    }
#endif

    Serial.printf("[ADC] no calibration for ch %d\n", (int)ch);
    return false;
}

void adc_init_internal() {
    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_config = {};
    chan_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    chan_config.atten    = ADC_ATTEN_DB_12;

    adc_channel_t ch_i = (adc_channel_t)g_cfg.adc_ch_current;
    adc_channel_t ch_v = (adc_channel_t)g_cfg.adc_ch_voltage;
    adc_channel_t ch_pv = (adc_channel_t)g_cfg.adc_ch_pot_v;
    adc_channel_t ch_pi = (adc_channel_t)g_cfg.adc_ch_pot_i;

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ch_i,  &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ch_v,  &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ch_pv, &chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ch_pi, &chan_config));

    adc_cali_v_ok = adc_calibration_init_channel(ch_v, &adc_cali_v_handle);
    adc_cali_i_ok = adc_calibration_init_channel(ch_i, &adc_cali_i_handle);

    Serial.printf("[ADC] cali V: %s, cali I: %s\n",
                  adc_cali_v_ok ? "OK" : "fallback",
                  adc_cali_i_ok ? "OK" : "fallback");
}

float adc_channel_to_mv(adc_channel_t ch, adc_cali_handle_t cali, bool cali_ok) {
    int raw = 0;
    if (adc_oneshot_read(adc_handle, ch, &raw) != ESP_OK) return -1.0f;
    int mv = 0;
    if (cali_ok && cali) {
        if (adc_cali_raw_to_voltage(cali, raw, &mv) != ESP_OK) {
            mv = (raw * 3300) / 4095;
        }
    } else {
        mv = (raw * 3300) / 4095;
    }
    return (float)mv;
}

float read_current_a() {
    adc_channel_t ch = (adc_channel_t)g_cfg.adc_ch_current;
    float mv = adc_channel_to_mv(ch, adc_cali_i_handle, adc_cali_i_ok);
    if (mv < 0) return -1.0f;
    float corrected = mv - g_cfg.current_offset_mv;
    if (corrected < 0) corrected = 0;
    return corrected / g_cfg.current_mv_per_amp;
}

float read_current_a_safe() {
    float v = read_current_a();
    if (v >= 0.0f) {
        g_last_valid_current_a = v;
        return v;
    }
    return g_last_valid_current_a;
}

float read_voltage_v() {
    adc_channel_t ch = (adc_channel_t)g_cfg.adc_ch_voltage;
    float mv = adc_channel_to_mv(ch, adc_cali_v_handle, adc_cali_v_ok);
    if (mv < 0) return -1.0f;
    float real = (mv - g_cfg.voltage_offset_mv) * g_cfg.voltage_divider_ratio;
    if (real < 0) real = 0;
    return real / 1000.0f;
}

float read_pot_norm(adc_channel_t ch) {
    int raw = 0;
    if (adc_oneshot_read(adc_handle, ch, &raw) != ESP_OK) return -1.0f;
    if (raw < 0)    raw = 0;
    if (raw > 4095) raw = 4095;
    return raw / 4095.0f;
}

float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    x = clampf(x, in_min, in_max);
    return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

// ======================== DEAD-TIME ========================
bool set_dead_time_ns_internal(float dt_ns) {
    if (dt_ns < g_cfg.dead_time_min_ns) dt_ns = g_cfg.dead_time_min_ns;
    if (dt_ns > g_cfg.dead_time_max_ns) dt_ns = g_cfg.dead_time_max_ns;

    const uint32_t TIMER_CLK_HZ = 80000000;
    float ticks_per_ns = (float)TIMER_CLK_HZ / 1e9f;
    uint32_t new_dt_ticks = (uint32_t)roundf(dt_ns * ticks_per_ns);

    if ((int32_t)new_dt_ticks + 20 >= (int32_t)g_half_period) {
        Serial.printf("[DT] %.0f ns rejected: dt=%lu ticks, half=%lu ticks\n",
                      dt_ns,
                      (unsigned long)new_dt_ticks,
                      (unsigned long)g_half_period);
        return false;
    }

    if (g_pwm_mutex == nullptr) return false;
    if (xSemaphoreTake(g_pwm_mutex, portMAX_DELAY) != pdTRUE) return false;

    set_pushpull_duty_raw(0.0f);

    g_dead_time_ns = dt_ns;
    g_dt_ticks     = new_dt_ticks;

    xSemaphoreGive(g_pwm_mutex);

    Serial.printf("[DT] -> %.0f ns (%lu ticks)\n",
                  dt_ns, (unsigned long)new_dt_ticks);
    return true;
}

// ======================== ЧАСТОТА ========================
bool set_pwm_frequency_internal(uint32_t freq_hz) {
    if (freq_hz < g_cfg.freq_min_hz) freq_hz = g_cfg.freq_min_hz;
    if (freq_hz > g_cfg.freq_max_hz) freq_hz = g_cfg.freq_max_hz;

    const uint32_t TIMER_CLK_HZ = 80000000;
    uint32_t period_ticks = TIMER_CLK_HZ / freq_hz;
    uint32_t half_period  = period_ticks / 2;

    if ((int32_t)half_period <= (int32_t)g_dt_ticks + 20) {
        Serial.printf("[FREQ] %lu Hz rejected: half=%lu, dt=%lu\n",
                      (unsigned long)freq_hz,
                      (unsigned long)half_period,
                      (unsigned long)g_dt_ticks);
        return false;
    }

    if (xSemaphoreTake(g_pwm_mutex, portMAX_DELAY) != pdTRUE) return false;

    set_pushpull_duty_raw(0.0f);

    mcpwm_timer_set_period(timer, period_ticks);
    g_half_period = half_period;
    g_freq_hz     = freq_hz;

    xSemaphoreGive(g_pwm_mutex);

    Serial.printf("[FREQ] -> %lu Hz (half=%lu ticks)\n",
                  (unsigned long)freq_hz, (unsigned long)half_period);
    return true;
}

// ======================== NVS ========================
struct Preset {
    uint32_t magic;
    float    u_set;
    float    i_trip;
    float    pid_kp;
    float    pid_ki;
    uint32_t freq_hz;
    float    dead_time_ns;
    uint32_t source;
};

constexpr uint32_t PRESET_MAGIC = 0x50554C4C;   // "PULL"
constexpr uint32_t SAVE_MIN_INTERVAL_MS = 1000;

void save_preset_internal() {
    if (g_nvs_mutex == nullptr) return;
    if (xSemaphoreTake(g_nvs_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        Serial.println("[NVS] busy, skip save");
        return;
    }

    uint32_t now = millis();
    if (now - g_last_save_ms < SAVE_MIN_INTERVAL_MS) {
        Serial.println("[NVS] save too soon, skip");
        xSemaphoreGive(g_nvs_mutex);
        return;
    }
    g_last_save_ms = now;

    Preset p;
    p.magic        = PRESET_MAGIC;
    p.u_set        = g_target_voltage_v;
    p.i_trip       = g_trip_current_ma;
    p.pid_kp       = g_pid_kp;
    p.pid_ki       = g_pid_ki;
    p.freq_hz      = g_freq_hz;
    p.dead_time_ns = g_dead_time_ns;
    p.source       = (uint32_t)g_source;

    g_prefs.begin(g_cfg.nvs_namespace, false);
    size_t written = g_prefs.putBytes(g_cfg.nvs_key, &p, sizeof(p));
    g_prefs.end();

    xSemaphoreGive(g_nvs_mutex);

    if (written == sizeof(p)) {
        Serial.printf("[NVS] saved: U=%.2f I=%.0f KP=%.2f KI=%.3f F=%lu DT=%.0f src=%lu\n",
                      p.u_set, p.i_trip, p.pid_kp, p.pid_ki,
                      (unsigned long)p.freq_hz, p.dead_time_ns,
                      (unsigned long)p.source);
    } else {
        Serial.println("[NVS] ERROR: save failed");
    }
}

bool load_preset_internal() {
    if (g_nvs_mutex == nullptr) return false;
    if (xSemaphoreTake(g_nvs_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) return false;

    Preset p;
    g_prefs.begin(g_cfg.nvs_namespace, true);
    size_t rd = g_prefs.getBytes(g_cfg.nvs_key, &p, sizeof(p));
    g_prefs.end();

    xSemaphoreGive(g_nvs_mutex);

    if (rd != sizeof(p) || p.magic != PRESET_MAGIC) {
        Serial.println("[NVS] no valid preset");
        return false;
    }

    g_target_voltage_v = p.u_set;
    g_trip_current_ma  = p.i_trip;
    g_pid_kp           = p.pid_kp;
    g_pid_ki           = p.pid_ki;
    g_source           = (SetpointSource)p.source;

    if (p.freq_hz >= g_cfg.freq_min_hz && p.freq_hz <= g_cfg.freq_max_hz) {
        g_freq_hz = p.freq_hz;
    }
    if (p.dead_time_ns >= g_cfg.dead_time_min_ns && p.dead_time_ns <= g_cfg.dead_time_max_ns) {
        g_dead_time_ns = p.dead_time_ns;
    }

    Serial.printf("[NVS] loaded: U=%.2f I=%.0f KP=%.2f KI=%.3f F=%lu DT=%.0f src=%lu\n",
                  p.u_set, p.i_trip, p.pid_kp, p.pid_ki,
                  (unsigned long)p.freq_hz, p.dead_time_ns,
                  (unsigned long)p.source);
    return true;
}

void reset_preset_internal() {
    g_target_voltage_v = g_cfg.target_voltage_v;
    g_trip_current_ma  = g_cfg.trip_current_ma;
    g_pid_kp           = g_cfg.pid_kp;
    g_pid_ki           = g_cfg.pid_ki;
    g_source           = SRC_FIXED;
    g_freq_hz          = g_cfg.freq_hz;
    g_dead_time_ns     = g_cfg.dead_time_ns;

    save_preset_internal();
    Serial.println("[NVS] reset to defaults");
}

// ======================== ЗАДАЧИ ========================
void softStartTask(void*) {
    Serial.println("[SOFT_START] Task started.");
    g_soft_start_running = true;

    const uint32_t steps = 100;
    const uint32_t step_delay_ms = g_cfg.soft_start_ms / steps;
    const float target_duty = g_soft_start_duty;

    for (uint32_t i = 0; i <= steps; i++) {
        if (g_state == STATE_FAULT_ACTIVE) {
            Serial.println("[SOFT_START] Aborted due to FAULT.");
            g_soft_start_running = false;
            vTaskDelete(nullptr);
            return;
        }
        safe_set_duty((target_duty * i) / (float)steps);
        vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    }

    if (g_state == STATE_SOFT_STARTING || g_state == STATE_RECOVERING) {
        g_state = STATE_RUNNING;
        Serial.printf("[SOFT_START] Completed. RUNNING at %.1f%% duty.\n", target_duty);
    }
    g_soft_start_running = false;
    vTaskDelete(nullptr);
}

void controlLoopTask(void*) {
    Serial.println("[PID] Task started.");

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(2);   // 2 мс = 500 Гц

    float integral  = 0.0f;
    bool  first_run = true;

    while (true) {
        float i_a = read_current_a_safe();
        float v_v = read_voltage_v();
        g_measured_current_a = i_a;
        if (v_v >= 0) g_measured_voltage_v = v_v;

        if (g_state == STATE_RUNNING) {
            if (v_v < 0) {
                vTaskDelayUntil(&last_wake, period);
                continue;
            }

            const float target_v = g_target_voltage_v;
            const float kp = g_pid_kp;
            const float ki = g_pid_ki;

            if (first_run) {
                const float error = target_v - v_v;
                const float start_duty = g_soft_start_duty;
                if (ki > 1e-6f) {
                    integral = (start_duty - kp * error) / ki;
                    if (integral >  g_cfg.pid_integral_clamp) integral =  g_cfg.pid_integral_clamp;
                    if (integral < -g_cfg.pid_integral_clamp) integral = -g_cfg.pid_integral_clamp;
                }
                first_run = false;
            } else {
                const float error = target_v - v_v;

                integral += error * 0.002f;
                if (integral >  g_cfg.pid_integral_clamp) integral =  g_cfg.pid_integral_clamp;
                if (integral < -g_cfg.pid_integral_clamp) integral = -g_cfg.pid_integral_clamp;

                float duty = kp * error + ki * integral;

                if (duty > 100.0f) {
                    duty = 100.0f;
                    integral -= error * 0.002f;
                } else if (duty < 0.0f) {
                    duty = 0.0f;
                    integral -= error * 0.002f;
                }

                safe_set_duty(duty);
            }
        } else {
            integral  = 0.0f;
            first_run = true;
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

void faultMonitorTask(void*) {
    Serial.println("[FAULT_MON] Task started.");

    bool     is_in_fault         = false;
    uint32_t fault_clear_time    = 0;
    uint32_t overcurrent_since   = 0;
    bool     overcurrent_latched = false;
    const uint32_t DEBOUNCE_MS   = 100;
    const uint32_t TRIP_DEBOUNCE_MS = 20;

    while (true) {
        bool hw_fault = is_fault_active_hardware();

        float current_a = read_current_a_safe();

        const float trip_ma = g_trip_current_ma;
        bool oc_now = (current_a * 1000.0f > trip_ma);

        if (oc_now) {
            if (overcurrent_since == 0) {
                overcurrent_since = millis();
            } else if (!overcurrent_latched &&
                       (millis() - overcurrent_since >= TRIP_DEBOUNCE_MS)) {
                overcurrent_latched = true;
                Serial.printf("!!! [FAULT] OVERCURRENT: %.2f A > %.2f A !!!\n",
                              current_a, trip_ma / 1000.0f);
            }
        } else {
            overcurrent_since = 0;
        }

        bool any_fault = hw_fault || overcurrent_latched;

        if (any_fault) {
            if (!is_in_fault) {
                is_in_fault = true;
                g_state = STATE_FAULT_ACTIVE;

                if (hw_fault)            Serial.println("!!! [FAULT] HARDWARE FAULT !!!");
                if (overcurrent_latched) Serial.println("!!! [FAULT] SOFTWARE OVERCURRENT !!!");

                safe_set_duty(0.0f);
            }
        } else {
            if (is_in_fault) {
                if (fault_clear_time == 0) {
                    fault_clear_time = millis();
                    Serial.println(">>> [FAULT] cleared. Debouncing...");
                }

                if (millis() - fault_clear_time >= DEBOUNCE_MS) {
                    Serial.println(">>> [FAULT] Recovering...");
                    g_state = STATE_RECOVERING;

                    mcpwm_operator_recover_from_fault(oper_a, gpio_fault);
                    mcpwm_operator_recover_from_fault(oper_b, gpio_fault);

                    overcurrent_latched = false;
                    overcurrent_since   = 0;

                    if (!g_soft_start_running) {
                        xTaskCreate(softStartTask, "SoftStart_Recover",
                                    4096, nullptr, tskIDLE_PRIORITY + 2, nullptr);
                    }

                    is_in_fault      = false;
                    fault_clear_time = 0;
                }
            } else {
                fault_clear_time = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setpointTask(void*) {
    Serial.println("[SETPOINT] Task started.");

    const float DT_SEC = 0.020f;

    float pot_v_filt = -1.0f;
    float pot_i_filt = -1.0f;

    float internal_v_target = g_cfg.target_voltage_v;
    float last_applied_v    = g_cfg.target_voltage_v;
    float last_applied_i    = g_cfg.trip_current_ma;

    while (true) {
        if (g_source != SRC_POTS) {
            bool wire_high = (digitalRead(g_cfg.pin_mode_select) == HIGH);
            SetpointSource from_pin = wire_high ? SRC_REMOTE : SRC_FIXED;
            if (from_pin != g_source) {
                g_source = from_pin;
                Serial.printf("[SRC] -> %s (pin)\n",
                              from_pin == SRC_REMOTE ? "REMOTE" : "FIXED");
            }
        }

        if (g_source == SRC_FIXED) {
            g_target_voltage_v = g_cfg.target_voltage_v;
            g_trip_current_ma  = g_cfg.trip_current_ma;
            internal_v_target  = g_cfg.target_voltage_v;
            last_applied_v     = g_cfg.target_voltage_v;
            last_applied_i     = g_cfg.trip_current_ma;
            pot_v_filt         = -1.0f;
            pot_i_filt         = -1.0f;

        } else if (g_source == SRC_POTS) {
            adc_channel_t ch_pv = (adc_channel_t)g_cfg.adc_ch_pot_v;
            adc_channel_t ch_pi = (adc_channel_t)g_cfg.adc_ch_pot_i;

            float nv = read_pot_norm(ch_pv);
            float ni = read_pot_norm(ch_pi);

            if (nv >= 0.0f) {
                if (pot_v_filt < 0.0f) pot_v_filt = nv;
                else pot_v_filt += 0.05f * (nv - pot_v_filt);

                float v_calc = mapf(pot_v_filt, 0.0f, 1.0f, g_cfg.v_min, g_cfg.v_max);

                if (fabsf(v_calc - last_applied_v) >= 0.08f) {
                    last_applied_v = v_calc;
                }

                float max_v_step = 10.0f * DT_SEC;
                if (last_applied_v > internal_v_target + max_v_step) {
                    internal_v_target += max_v_step;
                } else if (last_applied_v < internal_v_target - max_v_step) {
                    internal_v_target -= max_v_step;
                } else {
                    internal_v_target = last_applied_v;
                }

                g_target_voltage_v = internal_v_target;
            }

            if (ni >= 0.0f) {
                if (pot_i_filt < 0.0f) pot_i_filt = ni;
                else pot_i_filt += 0.05f * (ni - pot_i_filt);

                float i_calc = mapf(pot_i_filt, 0.0f, 1.0f, g_cfg.i_min, g_cfg.i_max);
                if (fabsf(i_calc - last_applied_i) >= 50.0f) {
                    last_applied_i = i_calc;
                }
                g_trip_current_ma = last_applied_i;
            }

        } else {
            g_target_voltage_v = clampf(g_target_voltage_v, g_cfg.v_min, g_cfg.v_max);
            g_trip_current_ma  = clampf(g_trip_current_ma,  g_cfg.i_min, g_cfg.i_max);
            internal_v_target  = g_target_voltage_v;
            last_applied_v     = g_target_voltage_v;
            last_applied_i     = g_trip_current_ma;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

} // namespace

// ======================== ПУБЛИЧНЫЙ API ========================

bool pushpull_init(const PushPullConfig& cfg) {
    if (g_initialized) return true;

    g_cfg = cfg;

    // Пины
    pinMode(g_cfg.pin_pwm_a, OUTPUT); digitalWrite(g_cfg.pin_pwm_a, LOW);
    pinMode(g_cfg.pin_pwm_b, OUTPUT); digitalWrite(g_cfg.pin_pwm_b, LOW);
    pinMode(g_cfg.pin_fault, INPUT_PULLDOWN);
    pinMode(g_cfg.pin_mode_select, INPUT_PULLUP);
    if (g_cfg.use_led && g_cfg.pin_led != 255) {
        pinMode(g_cfg.pin_led, OUTPUT);
        digitalWrite(g_cfg.pin_led, LOW);
    }

    // Начальные значения
    g_freq_hz          = g_cfg.freq_hz;
    g_dead_time_ns     = g_cfg.dead_time_ns;
    g_target_voltage_v = g_cfg.target_voltage_v;
    g_trip_current_ma  = g_cfg.trip_current_ma;
    g_pid_kp           = g_cfg.pid_kp;
    g_pid_ki           = g_cfg.pid_ki;
    g_soft_start_duty  = g_cfg.soft_start_duty_pct;

    // Мьютексы
    g_pwm_mutex = xSemaphoreCreateMutex();
    g_nvs_mutex = xSemaphoreCreateMutex();
    if (g_pwm_mutex == nullptr || g_nvs_mutex == nullptr) {
        Serial.println("[CORE] mutex creation failed");
        return false;
    }

    load_preset_internal();
    adc_init_internal();

    // MCPWM
    const uint32_t TIMER_CLK_HZ = 80000000;
    uint32_t period_ticks = TIMER_CLK_HZ / g_freq_hz;
    g_half_period = period_ticks / 2;

    float ticks_per_ns = (float)TIMER_CLK_HZ / 1e9f;
    g_dt_ticks = (uint32_t)roundf(g_dead_time_ns * ticks_per_ns);

    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id      = 0;
    timer_config.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = TIMER_CLK_HZ;
    timer_config.count_mode    = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks  = period_ticks;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_gpio_fault_config_t fault_config = {};
    fault_config.group_id = 0;
    fault_config.gpio_num = (gpio_num_t)g_cfg.pin_fault;
    fault_config.flags.active_level = true;
    ESP_ERROR_CHECK(mcpwm_new_gpio_fault(&fault_config, &gpio_fault));

    mcpwm_operator_config_t oper_config = {};
    oper_config.group_id = 0;
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper_a));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_a, timer));
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper_b));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_b, timer));

    mcpwm_comparator_config_t cmpr_config = {};
    cmpr_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_a, &cmpr_config, &cmpr_a1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_a, &cmpr_config, &cmpr_a2));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_b, &cmpr_config, &cmpr_b1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_b, &cmpr_config, &cmpr_b2));

    set_pushpull_duty_raw(0.0f);

    mcpwm_generator_config_t gen_config = {};
    gen_config.gen_gpio_num = (gpio_num_t)g_cfg.pin_pwm_a;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_a, &gen_config, &gen_a));
    gen_config.gen_gpio_num = (gpio_num_t)g_cfg.pin_pwm_b;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_b, &gen_config, &gen_b));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_a,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_a,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_a1, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_a,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_a2, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_b,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_b,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_b1, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_b,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpr_b2, MCPWM_GEN_ACTION_LOW)));

    mcpwm_brake_config_t brake_config = {};
    brake_config.fault = gpio_fault;
    brake_config.brake_mode = MCPWM_OPER_BRAKE_MODE_OST;

    ESP_ERROR_CHECK(mcpwm_operator_set_brake_on_fault(oper_a, &brake_config));
    ESP_ERROR_CHECK(mcpwm_operator_set_brake_on_fault(oper_b, &brake_config));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(gen_a,
        MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(gen_b,
        MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    xTaskCreate(faultMonitorTask, "FaultMon", 4096, nullptr, tskIDLE_PRIORITY + 3, nullptr);
    xTaskCreate(controlLoopTask,  "PID",      4096, nullptr, tskIDLE_PRIORITY + 2, nullptr);
    xTaskCreate(setpointTask,     "Setpoint", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);

    g_initialized = true;
    return true;
}

void pushpull_start() {
    if (!g_initialized) return;
    g_state = STATE_SOFT_STARTING;
    xTaskCreate(softStartTask, "SoftStart", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);
}

const PushPullConfig& pushpull_get_config() {
    return g_cfg;
}

// --- Уставки ---
bool pushpull_set_voltage(float v) {
    if (v < g_cfg.v_min || v > g_cfg.v_max) return false;
    g_target_voltage_v = v;
    return true;
}

bool pushpull_set_current_limit(float ma) {
    if (ma < g_cfg.i_min || ma > g_cfg.i_max) return false;
    g_trip_current_ma = ma;
    return true;
}

bool pushpull_set_pid(float kp, float ki) {
    if (kp < 0.0f || kp > 20.0f) return false;
    if (ki < 0.0f || ki > 5.0f)  return false;
    g_pid_kp = kp;
    g_pid_ki = ki;
    return true;
}

bool pushpull_set_frequency(uint32_t hz) {
    if (!g_initialized) return false;
    return set_pwm_frequency_internal(hz);
}

bool pushpull_set_dead_time_ns(float ns) {
    if (!g_initialized) return false;
    return set_dead_time_ns_internal(ns);
}

bool pushpull_set_source(SetpointSource src) {
    if (src > SRC_REMOTE) return false;
    g_source = src;
    return true;
}

bool pushpull_set_soft_start_duty(float pct) {
    if (pct < 0.0f || pct > 100.0f) return false;
    g_soft_start_duty = pct;
    return true;
}

// --- Чтение ---
float pushpull_get_voltage_setpoint() { return g_target_voltage_v; }
float pushpull_get_current_limit()    { return g_trip_current_ma; }
float pushpull_get_voltage_measured() { return g_measured_voltage_v; }
float pushpull_get_current_measured() { return g_measured_current_a; }
float pushpull_get_pid_kp()           { return g_pid_kp; }
float pushpull_get_pid_ki()           { return g_pid_ki; }
uint32_t pushpull_get_frequency()     { return g_freq_hz; }
float pushpull_get_dead_time_ns()     { return g_dead_time_ns; }
float pushpull_get_soft_start_duty()  { return g_soft_start_duty; }
SetpointSource pushpull_get_source()  { return g_source; }
SystemState pushpull_get_state()      { return g_state; }

bool pushpull_is_running() { return g_state == STATE_RUNNING; }
bool pushpull_is_faulted() { return g_state == STATE_FAULT_ACTIVE; }

// --- Пресет ---
bool pushpull_save_preset()       { if (!g_initialized) return false; save_preset_internal(); return true; }
bool pushpull_load_preset()       { if (!g_initialized) return false; return load_preset_internal(); }
void pushpull_reset_to_defaults() { if (!g_initialized) return; reset_preset_internal(); }

// --- Утилиты ---
const char* pushpull_state_str(SystemState s) {
    switch (s) {
        case STATE_INIT:          return "INIT";
        case STATE_SOFT_STARTING: return "SOFT";
        case STATE_RUNNING:       return "RUN";
        case STATE_FAULT_ACTIVE:  return "FAULT";
        case STATE_RECOVERING:    return "RECOVER";
    }
    return "?";
}

const char* pushpull_source_str(SetpointSource s) {
    switch (s) {
        case SRC_FIXED:  return "FIXED";
        case SRC_POTS:   return "POTS";
        case SRC_REMOTE: return "REMOTE";
    }
    return "?";
}

void pushpull_blink_led(int times, int ms) {
    if (!g_cfg.use_led || g_cfg.pin_led == 255) return;
    for (int i = 0; i < times; i++) {
        digitalWrite(g_cfg.pin_led, HIGH);
        vTaskDelay(pdMS_TO_TICKS(ms));
        digitalWrite(g_cfg.pin_led, LOW);
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}