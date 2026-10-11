#include "SerialInterface.h"
#include "PushPullCore.h"

static void handle_serial_command(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    const PushPullConfig& cfg = pushpull_get_config();

    int eq = cmd.indexOf('=');

    if (eq < 0) {
        String key = cmd;
        key.toUpperCase();

        if (key == "SAVE") {
            pushpull_save_preset();
            pushpull_blink_led(2, 80);
        } else if (key == "LOAD") {
            pushpull_load_preset();
            pushpull_set_frequency(pushpull_get_frequency());
            pushpull_set_dead_time_ns(pushpull_get_dead_time_ns());
        } else if (key == "RESET") {
            pushpull_reset_to_defaults();
            pushpull_set_frequency(pushpull_get_frequency());
            pushpull_set_dead_time_ns(pushpull_get_dead_time_ns());
            pushpull_blink_led(5, 50);
        } else if (key == "STATUS") {
            Serial.printf("[STATUS] Src=%s U_set=%.2f I_trip=%.0f "
                          "KP=%.2f KI=%.3f F=%lu DT=%.0f State=%d\n",
                          pushpull_source_str(pushpull_get_source()),
                          pushpull_get_voltage_setpoint(),
                          pushpull_get_current_limit(),
                          pushpull_get_pid_kp(),
                          pushpull_get_pid_ki(),
                          (unsigned long)pushpull_get_frequency(),
                          pushpull_get_dead_time_ns(),
                          (int)pushpull_get_state());
        } else {
            Serial.printf("[CMD] unknown: %s\n", cmd.c_str());
        }
        return;
    }

    String key = cmd.substring(0, eq);
    String val = cmd.substring(eq + 1);
    key.trim();
    val.trim();
    key.toUpperCase();

    if (key == "U") {
        float u = val.toFloat();
        if (pushpull_set_voltage(u)) {
            Serial.printf("[CMD] U=%.2f V\n", u);
        } else {
            Serial.printf("[CMD] U out of range (%.2f..%.2f)\n", cfg.v_min, cfg.v_max);
        }
    } else if (key == "I") {
        float i = val.toFloat();
        if (pushpull_set_current_limit(i)) {
            Serial.printf("[CMD] I=%.0f mA\n", i);
        } else {
            Serial.printf("[CMD] I out of range (%.0f..%.0f)\n", cfg.i_min, cfg.i_max);
        }
    } else if (key == "KP") {
        float v = val.toFloat();
        if (v >= 0.0f && v <= 20.0f) {
            pushpull_set_pid(v, pushpull_get_pid_ki());
            Serial.printf("[CMD] KP=%.3f\n", v);
        } else {
            Serial.println("[CMD] KP out of range");
        }
    } else if (key == "KI") {
        float v = val.toFloat();
        if (v >= 0.0f && v <= 5.0f) {
            pushpull_set_pid(pushpull_get_pid_kp(), v);
            Serial.printf("[CMD] KI=%.4f\n", v);
        } else {
            Serial.println("[CMD] KI out of range");
        }
    } else if (key == "F") {
        uint32_t f = (uint32_t)val.toInt();
        pushpull_set_frequency(f);
    } else if (key == "DT") {
        float dt = val.toFloat();
        pushpull_set_dead_time_ns(dt);
    } else if (key == "SRC") {
        String s = val;
        s.toUpperCase();
        if (s == "FIXED") {
            pushpull_set_source(SRC_FIXED);
            Serial.println("[CMD] SRC=FIXED");
        } else if (s == "POTS") {
            pushpull_set_source(SRC_POTS);
            Serial.println("[CMD] SRC=POTS");
        } else if (s == "REMOTE" || s == "WEB") {
            pushpull_set_source(SRC_REMOTE);
            Serial.println("[CMD] SRC=REMOTE");
        } else {
            Serial.println("[CMD] SRC must be FIXED|POTS|REMOTE");
        }
    } else {
        Serial.printf("[CMD] unknown key: %s\n", key.c_str());
    }
}

static void serialTask(void*) {
    Serial.println("[SERIAL] Commands: U= I= KP= KI= F= DT= SRC= SAVE LOAD RESET STATUS");

    String buf;
    buf.reserve(64);

    while (true) {
        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\n' || c == '\r') {
                if (buf.length() > 0) {
                    handle_serial_command(buf);
                    buf = "";
                }
            } else {
                if (buf.length() < 63) buf += c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void pushpull_serial_iface_init() {
    xTaskCreate(serialTask, "Serial", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);
}