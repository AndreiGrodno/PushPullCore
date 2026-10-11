// BasicSerial.ino — минимальный пример использования PushPullCore.
// Открыть в Arduino IDE или PlatformIO как отдельный скетч.

#include <PushPullCore.h>

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- PushPullCore BasicSerial ---");

    // Все дефолты. Меняем только пины и частоту — для примера.
    PushPullConfig cfg = {};
    cfg.pin_pwm_a = 18;
    cfg.pin_pwm_b = 19;
    cfg.freq_hz   = 50000;

    if (!pushpull_init(cfg)) {
        Serial.println("!!! pushpull_init() failed !!!");
        while (1) delay(1000);
    }

    // Включаем Serial-надстройку (команды U= I= KP= KI= F= DT= SRC= SAVE LOAD RESET STATUS)
    pushpull_serial_iface_init();

    // Включаем Button-надстройку (кнопка SAVE/RESET + LED)
    pushpull_button_iface_init();

    pushpull_start();
}

void loop() {
    delay(1000);
}