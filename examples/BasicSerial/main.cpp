
// main.cpp — минимальный пример с веб-интерфейсом
//
// Что делает:
//   - поднимает PushPullCore с дефолтными пинами
//   - запускает Serial-команды (U=, I=, F=, DT=, SRC=, SAVE, STATUS)
//   - запускает кнопку SAVE/RESET на GPIO27
//   - поднимает Wi-Fi AP "PushPull"
//   - отдаёт веб-панель на http://192.168.4.1
//
// Зависимости (platformio.ini):
//   lib_deps =
//       andreigrodno/PushPullCore@^1.0.0
//       esp32async/AsyncTCP@^3.5.0
//       esp32async/ESPAsyncWebServer@^3.12.1
//       bblanchon/ArduinoJson@^7.2.2
//  если ты это читаешь, пусть у тебя в жизни все будет хорошо.
// :)
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <PushPullCore.h>

// ======================== НАСТРОЙКИ ========================
#define WIFI_SSID  "PushPull"
#define WIFI_PASS  ""

// ======================== HTML ========================
// Минимальная страница: уставки U/I, выбор источника, кнопка SAVE, телеметрия.
static const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PushPull</title>
<style>
    body { font-family: sans-serif; max-width: 480px; margin: 12px auto; padding: 0 12px; }
    h2   { margin: 16px 0 8px; }
    .row { display: flex; align-items: center; gap: 8px; margin: 6px 0; }
    .row label { width: 70px; }
    .row input[type=range] { flex: 1; }
    .row .v { width: 70px; text-align: right; font-family: monospace; }
    .btns { display: flex; gap: 6px; flex-wrap: wrap; margin: 6px 0; }
    button { padding: 6px 12px; }
    button.active { font-weight: bold; }
    pre { background: #111; color: #0f0; padding: 8px; font-size: 12px; }
</style>
</head>
<body>

<h2>PushPull</h2>

<div class="btns">
    <button id="b_fixed"  onclick="src('FIXED')">FIXED</button>
    <button id="b_pots"   onclick="src('POTS')">POTS</button>
    <button id="b_remote" onclick="src('REMOTE')">REMOTE</button>
</div>

<div class="row">
    <label>U, V</label>
    <input type="range" id="u" min="1" max="24" step="0.1" value="12"
           oninput="uVal.textContent=(+this.value).toFixed(1); sendU(this.value)">
    <span class="v" id="uVal">12.0</span>
</div>

<div class="row">
    <label>I, mA</label>
    <input type="range" id="i" min="500" max="5000" step="50" value="5000"
           oninput="iVal.textContent=this.value; sendI(this.value)">
    <span class="v" id="iVal">5000</span>
</div>

<div class="btns">
    <button onclick="post('/api/save')">SAVE</button>
    <button onclick="post('/api/reset')">RESET</button>
</div>

<pre id="tel">...</pre>

<script>
const uVal = document.getElementById('uVal');
const iVal = document.getElementById('iVal');
const tel  = document.getElementById('tel');

let tU, tI;
function sendU(v) { clearTimeout(tU); tU = setTimeout(() => post('/api/u', {u:+v}), 150); }
function sendI(v) { clearTimeout(tI); tI = setTimeout(() => post('/api/i', {i:+v}), 150); }

async function post(path, body) {
    try {
        await fetch(path, {
            method: 'POST',
            headers: {'Content-Type':'application/json'},
            body: JSON.stringify(body || {})
        });
    } catch(e) {}
}

function src(s) { post('/api/src', {src:s}); }

async function poll() {
    try {
        const r = await fetch('/api/status');
        const s = await r.json();
        tel.textContent =
            'Src   : ' + s.source + '\n' +
            'U_set : ' + s.u_set.toFixed(2) + ' V\n' +
            'I_trip: ' + s.i_trip + ' mA\n' +
            'U_meas: ' + s.u_meas.toFixed(2) + ' V\n' +
            'I_meas: ' + s.i_meas.toFixed(3) + ' A\n' +
            'State : ' + s.state;

        document.querySelectorAll('button').forEach(b => b.classList.remove('active'));
        const el = document.getElementById('b_' + s.source.toLowerCase());
        if (el) el.classList.add('active');

        const ue = document.getElementById('u');
        const ie = document.getElementById('i');
        if (document.activeElement !== ue) {
            ue.value = s.u_set;
            uVal.textContent = s.u_set.toFixed(1);
        }
        if (document.activeElement !== ie) {
            ie.value = s.i_trip;
            iVal.textContent = s.i_trip;
        }
    } catch(e) {}
    setTimeout(poll, 1000);
}

poll();
</script>
</body>
</html>
)HTML";

// ======================== ВЕБ-СЕРВЕР ========================
static AsyncWebServer server(80);

static void sendOk(AsyncWebServerRequest* r) {
    r->send(200, "application/json", "{\"ok\":true}");
}
static void sendErr(AsyncWebServerRequest* r, const char* msg) {
    r->send(400, "application/json", String("{\"err\":\"") + msg + "\"}");
}

static void setupWeb() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(200, "text/html", PAGE);
    });

    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* r) {
        JsonDocument d;
        d["source"] = pushpull_source_str(pushpull_get_source());
        d["u_set"]  = pushpull_get_voltage_setpoint();
        d["i_trip"] = pushpull_get_current_limit();
        d["u_meas"] = pushpull_get_voltage_measured();
        d["i_meas"] = pushpull_get_current_measured();
        d["state"]  = pushpull_state_str(pushpull_get_state());
        String out;
        serializeJson(d, out);
        r->send(200, "application/json", out);
    });

    server.on("/api/u", HTTP_POST, [](AsyncWebServerRequest* r) {}, nullptr,
        [](AsyncWebServerRequest* r, uint8_t* data, size_t len, size_t, size_t) {
            JsonDocument d;
            if (deserializeJson(d, data, len)) return sendErr(r, "json");
            if (!pushpull_set_voltage(d["u"] | 0.0f)) return sendErr(r, "range");
            sendOk(r);
        });

    server.on("/api/i", HTTP_POST, [](AsyncWebServerRequest* r) {}, nullptr,
        [](AsyncWebServerRequest* r, uint8_t* data, size_t len, size_t, size_t) {
            JsonDocument d;
            if (deserializeJson(d, data, len)) return sendErr(r, "json");
            if (!pushpull_set_current_limit(d["i"] | 0.0f)) return sendErr(r, "range");
            sendOk(r);
        });

    server.on("/api/src", HTTP_POST, [](AsyncWebServerRequest* r) {}, nullptr,
        [](AsyncWebServerRequest* r, uint8_t* data, size_t len, size_t, size_t) {
            JsonDocument d;
            if (deserializeJson(d, data, len)) return sendErr(r, "json");
            String s = d["src"] | "";
            s.toUpperCase();
            if      (s == "FIXED")  pushpull_set_source(SRC_FIXED);
            else if (s == "POTS")   pushpull_set_source(SRC_POTS);
            else if (s == "REMOTE") pushpull_set_source(SRC_REMOTE);
            else return sendErr(r, "src");
            sendOk(r);
        });

    server.on("/api/save",  HTTP_POST, [](AsyncWebServerRequest* r) { pushpull_save_preset(); sendOk(r); });
    server.on("/api/reset", HTTP_POST, [](AsyncWebServerRequest* r) {
        pushpull_reset_to_defaults();
        pushpull_set_frequency(pushpull_get_frequency());
        pushpull_set_dead_time_ns(pushpull_get_dead_time_ns());
        sendOk(r);
    });

    server.begin();
}

// ======================== SETUP / LOOP ========================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n--- PushPull minimal web ---");

    // 1. Ядро
    PushPullConfig cfg = {};
    if (!pushpull_init(cfg)) {
        Serial.println("init failed");
        while (1) delay(1000);
    }

    // 2. Serial + кнопка
    pushpull_serial_iface_init();
    pushpull_button_iface_init();

    // 3. Wi-Fi AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Serial.printf("AP: %s / %s\n", WIFI_SSID, WIFI_PASS);
    Serial.printf("Open: http://%s\n", WiFi.softAPIP().toString().c_str());

    // 4. Веб-сервер
    setupWeb();
    Serial.println("web ready");

    // 5. Старт ШИМ
    pushpull_start();
}

void loop() {
    delay(1000);
}
