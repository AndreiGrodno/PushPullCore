#pragma once
#include <Arduino.h>

// ======================== ТИПЫ ========================
enum SetpointSource : uint8_t {
    SRC_FIXED  = 0,
    SRC_POTS   = 1,
    SRC_REMOTE = 2
};

enum SystemState : uint8_t {
    STATE_INIT,
    STATE_SOFT_STARTING,
    STATE_RUNNING,
    STATE_FAULT_ACTIVE,
    STATE_RECOVERING
};

// ======================== КОНФИГУРАЦИЯ ========================
// Все поля имеют дефолтные значения. Пользователь заполняет только то,
// что ему нужно:
//
//     PushPullConfig cfg = {};
//     cfg.pin_pwm_a = 18;
//     cfg.freq_hz   = 75000;
//     pushpull_init(cfg);
//
struct PushPullConfig {
    // --- Пины ШИМ и защиты ---
    uint8_t  pin_pwm_a           = 18;
    uint8_t  pin_pwm_b           = 19;
    uint8_t  pin_fault           = 4;

    // --- Перемычка выбора источника уставок ---
    // LOW  (замкнута на GND) → SRC_FIXED
    // HIGH (разомкнута)      → SRC_REMOTE
    uint8_t  pin_mode_select     = 23;

    // --- Кнопка SAVE/RESET и LED (опционально) ---
    uint8_t  pin_btn_save        = 27;
    bool     use_button          = true;   // если false — кнопка не подключается
    uint8_t  pin_led             = 2;
    bool     use_led             = true;   // если false — LED не подключается

    // --- ADC каналы (ADC1, channel 0..7) ---
    uint8_t  adc_ch_current      = 6;   // 6 = GPIO34 на ESP32
    uint8_t  adc_ch_voltage      = 7;   // 7 = GPIO35 на ESP32
    uint8_t  adc_ch_pot_v        = 4;   // 4 = GPIO32 на ESP32
    uint8_t  adc_ch_pot_i        = 5;   // 5 = GPIO33 на ESP32

    // --- ШИМ ---
    uint32_t freq_hz             = 50000;
    uint32_t freq_min_hz         = 20000;
    uint32_t freq_max_hz         = 100000;
    float    dead_time_ns        = 1000.0f;
    float    dead_time_min_ns    = 100.0f;
    float    dead_time_max_ns    = 5000.0f;

    // --- Уставки (SRC_FIXED) ---
    float    target_voltage_v    = 12.0f;
    float    trip_current_ma     = 5000.0f;
    float    v_min               = 1.0f;
    float    v_max               = 24.0f;
    float    i_min               = 500.0f;
    float    i_max               = 5000.0f;

    // --- PID ---
    float    pid_kp              = 2.0f;
    float    pid_ki              = 0.09f;
    float    pid_integral_clamp  = 2000.0f;

    // --- ADC калибровка ---
    float    current_mv_per_amp    = 1000.0f;
    float    current_offset_mv     = 142.0f;
    float    voltage_offset_mv     = 142.0f;
    float    voltage_divider_ratio = 11.0f;

    // --- Soft-start ---
    uint32_t soft_start_ms         = 2000;
    float    soft_start_duty_pct   = 60.0f;

    // --- NVS ---
    const char* nvs_namespace    = "pushpull";
    const char* nvs_key          = "preset";
};

// ======================== ИНИЦИАЛИЗАЦИЯ ========================
// Инициализирует MCPWM, ADC, задачи (PID, faultMon, setpoint, button).
// НЕ запускает soft-start — это делает pushpull_start().
// Возвращает false при критической ошибке (мьютексы, MCPWM).
bool pushpull_init(const PushPullConfig& cfg);

// Запускает soft-start → RUNNING.
void pushpull_start();

// Возвращает сохранённый конфиг (для надстроек).
const PushPullConfig& pushpull_get_config();

// ======================== УСТАВКИ ========================
bool pushpull_set_voltage(float v);
bool pushpull_set_current_limit(float ma);
bool pushpull_set_pid(float kp, float ki);
bool pushpull_set_frequency(uint32_t hz);
bool pushpull_set_dead_time_ns(float ns);
bool pushpull_set_source(SetpointSource src);
bool pushpull_set_soft_start_duty(float pct);

// ======================== ЧТЕНИЕ ========================
float          pushpull_get_voltage_setpoint();
float          pushpull_get_current_limit();
float          pushpull_get_voltage_measured();
float          pushpull_get_current_measured();
float          pushpull_get_pid_kp();
float          pushpull_get_pid_ki();
uint32_t       pushpull_get_frequency();
float          pushpull_get_dead_time_ns();
float          pushpull_get_soft_start_duty();
SetpointSource pushpull_get_source();
SystemState    pushpull_get_state();

bool pushpull_is_running();
bool pushpull_is_faulted();

// ======================== ПРЕСЕТ (NVS) ========================
bool pushpull_save_preset();
bool pushpull_load_preset();
void pushpull_reset_to_defaults();

// ======================== УТИЛИТЫ ========================
const char* pushpull_state_str(SystemState s);
const char* pushpull_source_str(SetpointSource s);

// Вспышка LED (если use_led == true). Если LED отключён — no-op.
void pushpull_blink_led(int times, int ms);

// Запуск Serial-надстройки (обработка команд в отдельной задаче).
void pushpull_serial_iface_init();

// Запуск Button-надстройки (обработка кнопки в отдельной задаче).
void pushpull_button_iface_init();