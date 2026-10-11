// main.cpp — минимальный пример с веб-интерфейсом
//
// Веб-панель со всеми изменяемыми параметрами:
//   - Источник уставок: FIXED / POTS / REMOTE
//   - U_set, I_trip
//   - PID: KP, KI
//   - ШИМ: F, DT
//   - Soft-start duty
//   - Пресет: SAVE / LOAD / RESET / REBOOT
//   - Телеметрия
//
// Зависимости (platformio.ini):
//   lib_deps =
//       andreigrodno/PushPullCore@^1.0.0
//       esp32async/AsyncTCP@^3.5.0
//       esp32async/ESPAsyncWebServer@^3.12.1
//       bblanchon/ArduinoJson@^7.2.2

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <PushPullCore.h>

// ======================== НАСТРОЙКИ ========================
#define WIFI_SSID  "PushPull"
#define WIFI_PASS  ""

// ======================== HTML ========================
static const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PushPull</title>
<style>
    * { box-sizing: border-box; }
    body { font-family: sans-serif; max-width: 520px; margin: 12px auto; padding: 0 12px; color: #222; }
    h2 { margin: 18px 0 6px; font-size: 16px; border-bottom: 1px solid #ccc; padding-bottom: 4px; }
    h2:first-of-type { margin-top: 0; }
    .row { display: flex; align-items: center; gap: 8px; margin: 6px 0; }
    .row label { width: 90px; font-size: 13px; }
    .row input[type=range] { flex: 1; }
    .row .v {
        width: 80px; text-align: right;
        font-family: monospace; font-size: 13px;
        background: #f4f4f4; padding: 2px 6px; border-radius: 3px;
    }
    .btns { display: flex; gap: 6px; flex-wrap: wrap; margin: 6px 0; }
    button {
        padding: 6px 12px;
        font-family: inherit; font-size: 13px;
        background: #eaeaea; border: 1px solid #bbb; border-radius: 3px;
        cursor: pointer;
    }
    button:hover { background: #ddd; }
    button.active { background: #4a90e2; color: #fff; border-color: #2a6fc2; }
    pre {
        background: #111; color: #0f0;
        padding: 8px 10px; font-size: 12px;
        border-radius: 3px; overflow-x: auto;
    }
</style>
</head>
<body>

<h2>PushPull</h2>

<h3 style="font-size:14px;margin:10px 0 4px;">Источник уставок</h3>
<div class="btns">
    <button id="b_fixed"  onclick="src('FIXED')">FIXED</button>
    <button id="b_pots"   onclick="src('POTS')">POTS</button>
    <button id="b_remote" onclick="src('REMOTE')">REMOTE</button>
</div>

<h2>Уставки</h2>
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

<h2>PID</h2>
<div class="row">
    <label>KP</label>
    <input type="range" id="kp" min="0" max="10" step="0.05" value="2"
           oninput="kpVal.textContent=(+this.value).toFixed(2); sendKP(this.value)">
    <span class="v" id="kpVal">2.00</span>
</div>
<div class="row">
    <label>KI</label>
    <input type="range" id="ki" min="0" max="1" step="0.005" value="0.09"
           oninput="kiVal.textContent=(+this.value).toFixed(3); sendKI(this.value)">
    <span class="v" id="kiVal">0.090</span>
</div>

<h2>ШИМ</h2>
<div class="row">
    <label>F, Hz</label>
    <input type="range" id="f" min="20000" max="100000" step="500" value="50000"
           oninput="fVal.textContent=this.value; sendF(this.value)">
    <span class="v" id="fVal">50000</span>
</div>
<div class="row">
    <label>DT, ns</label>
    <input type="range" id="dt" min="100" max="5000" step="50" value="1000"
           oninput="dtVal.textContent=this.value; sendDT(this.value)">
    <span class="v" id="dtVal">1000</span>
</div>

<h2>Soft-start</h2>
<div class="row">
    <label>duty, %</label>
    <input type="range" id="ss" min="0" max="100" step="1" value="60"
           oninput="ssVal.textContent=this.value; sendSS(this.value)">
    <span class="v" id="ssVal">60</span>
</div>

<h2>Пресет</h2>
<div class="btns">
    <button onclick="post('/api/save')">SAVE</button>
    <button onclick="post('/api/load')">LOAD</button>
    <button onclick="post('/api/reset')">RESET</button>
    <button onclick="post('/api/reboot')">REBOOT</button>
</div>

<h2>Телеметрия</h2>
<pre id="tel">...</pre>

<script>
const uVal=document.getElementById('uVal'), iVal=document.getElementById('iVal');
const kpVal=document.getElementById('kpVal'), kiVal=document.getElementById('kiVal');
const fVal=document.getElementById('fVal'), dtVal=document.getElementById('dtVal');
const ssVal=document.getElementById('ssVal');
const tel=document.getElementById('tel');

function deb(fn, ms){ let t; return v => { clearTimeout(t); t = setTimeout(()=>fn(v), ms); }; }
const sendU  = deb(v => post('/api/u',  {u:  +v}), 150);
const sendI  = deb(v => post('/api/i',  {i:  +v}), 150);
const sendKP = deb(v => post('/api/kp', {kp: +v}), 150);
const sendKI = deb(v => post('/api/ki', {ki: +v}), 150);
const sendF  = deb(v => post('/api/f',  {f:  +v}), 300);
const sendDT = deb(v => post('/api/dt', {dt: +v}), 300);
const sendSS = deb(v => post('/api/ss', {ss: +v}), 150);

async function post(path, body) {
    try {
        await fetch(path, {
            method:'POST',
            headers:{'Content-Type':'application/json'},
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
            'Src      : ' + s.source + '\n' +
            'U_set    : ' + s.u_set.toFixed(2) + ' V\n' +
            'I_trip   : ' + s.i_trip + ' mA\n' +
            'U_meas   : ' + s.u_meas.toFixed(2) + ' V\n' +
            'I_meas   : ' + s.i_meas.toFixed(3) + ' A\n' +
            'F        : ' + s.freq + ' Hz\n' +
            'DT       : ' + s.dt.toFixed(0) + ' ns\n' +
            'KP       : ' + s.kp.toFixed(2) + '\n' +
            'KI       : ' + s.ki.toFixed(3) + '\n' +
            'SS duty  : ' + s.ss.toFixed(0) + ' %\n' +
            'State    : ' + s.state;

        document.querySelectorAll('button').forEach(b => b.classList.remove('active'));
        const el = document.getElementById('b_' + s.source.toLowerCase());
        if (el) el.classList.add('active');

        sync('u',  s.u_set, 1);
        sync('i',  s.i_trip, 0);
        sync('kp', s.kp, 2);
        sync('ki', s.ki, 3);
        sync('f',  s.freq, 0);
        sync('dt', s.dt, 0);
        sync('ss', s.ss, 0);

    } catch(e) {}
    setTimeout(poll, 1000);
}

function sync(id, value, dec) {
    const el = document.getElementById(id);
    if (document.activeElement === el) return;
    const v = +value;
    if (Math.abs(+el.value - v) < Math.pow(10,-dec)/2) return;
    el.value = v;
    document.getElementById(id + 'Val').textContent =
        dec ? v.toFixed(dec) : String(v);
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

// Хелпер: парсит JSON-тело и вызывает callback
using JsonHandler = std::function<void(JsonDocument&)>;
static void withJson(AsyncWebServerRequest* r, uint8_t* data, size_t len,
                     JsonHandler fn) {
    JsonDocument d;
    if (deserializeJson(d, data, len)) return sendErr(r, "json");
    fn(d);
}

static void setupWeb() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
        r->send(200, "text/html", PAGE);
    });

    // --- Телеметрия ---
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* r) {
        JsonDocument d;
        d["source"] = pushpull_source_str(pushpull_get_source());
        d["u_set"]  = pushpull_get_voltage_setpoint();
        d["i_trip"] = pushpull_get_current_limit();
        d["u_meas"] = pushpull_get_voltage_measured();
        d["i_meas"] = pushpull_get_current_measured();
        d["freq"]   = pushpull_get_frequency();
        d["dt"]     = pushpull_get_dead_time_ns();
        d["kp"]     = pushpull_get_pid_kp();
        d["ki"]     = pushpull_get_pid_ki();
        d["ss"]     = pushpull_get_soft_start_duty();
        d["state"]  = pushpull_state_str(pushpull_get_state());
        String out;
        serializeJson(d, out);
        r->send(200, "application/json", out);
    });

    // --- Уставки ---
    server.on("/api/u", HTTP_POST, [](AsyncWebServerRequest* r){}, nullptr,
        [](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t, size_t) {
            withJson(r, d, l, [&](JsonDocument& j){
                if (!pushpull_set_voltage(j["u"] | 0.0f)) return sendErr(r, "U out of range");
                sendOk(r);
            });
        });

    server.on("/api/i", HTTP_POST, [](AsyncWebServerRequest* r){}, nullptr,
        [](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t, size_t) {
            withJson(r, d, l, [&](JsonDocument& j){
                if (!pushpull_set_current_limit(j["i"] | 0.0f)) return sendErr(r, "I out of range");
                sendOk(r);
            });
        });

    // --- PID ---
    server.on("/api/kp", HTTP_POST, [](AsyncWebServerRequest* r){}, nullptr,
        [](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t, size_t) {
            withJson(r, d, l, [&](JsonDocument& j){
                float kp = j["kp"] | pushpull_get_pid_kp();
                if (!pushpull_set_pid(kp, pushpull_get_pid_ki()))
                    return sendErr(r, "KP out of range");
                sendOk(r);
            });
        });

    server.on("/api/ki", HTTP_POST, [](AsyncWebServerRequest* r){}, nullptr,
        [](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t, size_t) {
            withJson(r, d, l, [&](JsonDocument& j){
                float ki = j["ki"] | pushpull_get_pid_ki();
                if (!pushpull_set_pid(pushpull_get_pid_kp(), ki))
                    return sendErr(r, "KI out of range");
                sendOk(r);
            });
        });

    // --- ШИМ ---
    server.on("/api/f", HTTP_POST, [](AsyncWebServerRequest* r){}, nullptr,
        [](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t, size_t) {
            withJson(r, d, l, [&](JsonDocument& j){
                uint32_t f = j["f"] | 0;
                if (!pushpull_set_frequency(f)) return sendErr(r, "F rejected");
                sendOk(r);
            });
        });

    server.on("/api/dt", HTTP_POST, [](AsyncWebServerRequest* r){}, nullptr,
        [](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t, size_t) {
            withJson(r, d, l, [&](JsonDocument& j){
                float dt = j["dt"] | 0.0f;
                if (!pushpull_set_dead_time_ns(dt)) return sendErr(r, "DT rejected");
                sendOk(r);
            });
        });

    // --- Soft-start ---
    server.on("/api/ss", HTTP_POST, [](AsyncWebServerRequest* r){}, nullptr,
        [](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t, size_t) {
            withJson(r, d, l, [&](JsonDocument& j){
                float ss = j["ss"] | 0.0f;
                if (!pushpull_set_soft_start_duty(ss)) return sendErr(r, "SS out of range");
                sendOk(r);
            });
        });

    // --- Источник ---
    server.on("/api/src", HTTP_POST, [](AsyncWebServerRequest* r){}, nullptr,
        [](AsyncWebServerRequest* r, uint8_t* d, size_t l, size_t, size_t) {
            withJson(r, d, l, [&](JsonDocument& j){
               const char* src_cstr = j["src"] | "";
String s = src_cstr;
                s.toUpperCase();
                if      (s == "FIXED")  pushpull_set_source(SRC_FIXED);
                else if (s == "POTS")   pushpull_set_source(SRC_POTS);
                else if (s == "REMOTE") pushpull_set_source(SRC_REMOTE);
                else return sendErr(r, "src must be FIXED|POTS|REMOTE");
                sendOk(r);
            });
        });

    // --- Пресет ---
    server.on("/api/save", HTTP_POST, [](AsyncWebServerRequest* r){
        pushpull_save_preset(); sendOk(r);
    });
    server.on("/api/load", HTTP_POST, [](AsyncWebServerRequest* r){
        pushpull_load_preset();
        pushpull_set_frequency(pushpull_get_frequency());
        pushpull_set_dead_time_ns(pushpull_get_dead_time_ns());
        sendOk(r);
    });
    server.on("/api/reset", HTTP_POST, [](AsyncWebServerRequest* r){
        pushpull_reset_to_defaults();
        pushpull_set_frequency(pushpull_get_frequency());
        pushpull_set_dead_time_ns(pushpull_get_dead_time_ns());
        sendOk(r);
    });
    server.on("/api/reboot", HTTP_POST, [](AsyncWebServerRequest* r){
        sendOk(r);
        delay(200);
        ESP.restart();
    });

    server.begin();
}

// ======================== SETUP / LOOP ========================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n--- PushPull web (all params) ---");

    PushPullConfig cfg = {};
    if (!pushpull_init(cfg)) {
        Serial.println("init failed");
        while (1) delay(1000);
    }

    pushpull_serial_iface_init();
    pushpull_button_iface_init();

    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Serial.printf("AP: %s / %s\n", WIFI_SSID, WIFI_PASS);
    Serial.printf("Open: http://%s\n", WiFi.softAPIP().toString().c_str());

    setupWeb();
    Serial.println("web ready");

    pushpull_start();
}

void loop() {
    delay(1000);
}
