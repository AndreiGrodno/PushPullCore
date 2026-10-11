// ============================================================
//  Config.h — СПРАВОЧНИК ДЕФОЛТОВ
// ============================================================
//  ВНИМАНИЕ: этот файл НЕ подключается в коде библиотеки.
//  Все дефолты определены в PushPullConfig (PushPullCore.h).
//
//  Оставлен как справочник:
//    - какие параметры доступны для настройки
//    - какие значения используются по умолчанию
//
//  Для настройки используйте структуру PushPullConfig:
//
//      PushPullConfig cfg = {};
//      cfg.pin_pwm_a = 18;
//      cfg.freq_hz   = 75000;
//      pushpull_init(cfg);
//
// ============================================================

#pragma once

// ---------- ПИНЫ ----------
// pin_pwm_a          = 18   (GPIO18)
// pin_pwm_b          = 19   (GPIO19)
// pin_fault          = 4    (GPIO4)
// pin_mode_select    = 23   (GPIO23, INPUT_PULLUP)
// pin_btn_save       = 27   (GPIO27, INPUT_PULLUP)
// use_button         = true
// pin_led            = 2    (GPIO2)
// use_led            = true

// ---------- ADC КАНАЛЫ (ADC1) ----------
// adc_ch_current     = 6    // 6 = GPIO34 на ESP32
// adc_ch_voltage     = 7    // 7 = GPIO35 на ESP32
// adc_ch_pot_v       = 4    // 4 = GPIO32 на ESP32
// adc_ch_pot_i       = 5    // 5 = GPIO33 на ESP32

// ---------- ШИМ ----------
// freq_hz            = 50000
// freq_min_hz        = 20000
// freq_max_hz        = 100000
// dead_time_ns       = 1000.0
// dead_time_min_ns   = 100.0
// dead_time_max_ns   = 5000.0

// ---------- УСТАВКИ ----------
// target_voltage_v   = 12.0
// trip_current_ma    = 5000.0
// v_min              = 1.0
// v_max              = 24.0
// i_min              = 500.0
// i_max              = 5000.0

// ---------- PID ----------
// pid_kp             = 2.0
// pid_ki             = 0.09
// pid_integral_clamp = 2000.0

// ---------- ADC КАЛИБРОВКА ----------
// current_mv_per_amp    = 1000.0
// current_offset_mv     = 142.0
// voltage_offset_mv     = 142.0
// voltage_divider_ratio = 11.0

// ---------- SOFT-START ----------
// soft_start_ms       = 2000
// soft_start_duty_pct = 60.0

// ---------- NVS ----------
// nvs_namespace      = "pushpull"
// nvs_key            = "preset"