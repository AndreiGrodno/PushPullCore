# PushPullCore

Push-pull MCPWM controller for ESP32 with PID, ADC, NVS preset.

## Возможности

- MCPWM push-pull с программным dead-time
- Частота ШИМ 20…100 кГц (на лету)
- PID-регулятор напряжения
- ADC: измерение U/I, калибровка
- Аппаратная защита (nFAULT) + программная по току
- Soft-start
- Три источника уставок: FIXED / POTS / REMOTE
- NVS-пресет

## Быстрый старт

```cpp
#include <PushPullCore.h>

void setup() {
    Serial.begin(115200);
    pushpull_init();
    pushpull_start();
}

void loop() {
    pushpull_set_voltage(12.0f);
    pushpull_set_current_limit(3000.0f);
    delay(1000);
}