#include "ButtonInterface.h"
#include "PushPullCore.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void buttonTask(void*) {
    Serial.println("[BTN] Task started.");

    const PushPullConfig& cfg = pushpull_get_config();

    bool     last_raw       = true;
    bool     stable         = true;
    uint32_t last_change_ms = 0;
    const uint32_t DEBOUNCE_MS = 30;

    uint32_t press_start_ms = 0;
    bool     long_fired     = false;

    while (true) {
        bool raw = (digitalRead(cfg.pin_btn_save) == HIGH);

        if (raw != last_raw) {
            last_change_ms = millis();
            last_raw = raw;
        }

        if ((millis() - last_change_ms) >= DEBOUNCE_MS && raw != stable) {
            stable = raw;

            if (!stable) {
                press_start_ms = millis();
                long_fired = false;
                Serial.println("[BTN] pressed");
            } else {
                uint32_t press_ms = millis() - press_start_ms;
                if (!long_fired && press_ms >= 30) {
                    Serial.println("[BTN] short -> SAVE");
                    pushpull_save_preset();
                    pushpull_blink_led(2, 80);
                }
                Serial.printf("[BTN] released after %lu ms\n",
                              (unsigned long)press_ms);
            }
        }

        if (!stable && !long_fired && (millis() - press_start_ms) >= 2000) {
            long_fired = true;
            Serial.println("[BTN] long -> RESET");
            pushpull_reset_to_defaults();
            pushpull_blink_led(5, 50);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void pushpull_button_iface_init() {
    const PushPullConfig& cfg = pushpull_get_config();
    if (!cfg.use_button || cfg.pin_btn_save == 255) {
        Serial.println("[BTN] disabled by config");
        return;
    }
    pinMode(cfg.pin_btn_save, INPUT_PULLUP);
    xTaskCreate(buttonTask, "Button", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);
}