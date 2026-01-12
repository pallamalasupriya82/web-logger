/*
  PROJECT: CRANE GUARD (Anti-Collision System)
  DEVICE: UNIT A (MASTER / ANCHOR)
  VERSION: V2.2 (Fixes: Unit Conversion, Stats Counting, Safety Logic)
  
  FIXES:
  1. Unit Match: Dashboard inputs (Meters) now correctly convert to Logic (CM).
  2. Safety Logic: Now reliably triggers WARNING and STOP modes.
  3. Statistics: Counters now increment properly when zones change.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <ModbusMaster.h>

// --- HARDWARE PINOUT ---
#define SERIAL_AT mySerial2
HardwareSerial SERIAL_AT(2);
#define IO_RXD2 18
#define IO_TXD2 17
#define UWB_RESET_PIN 16

#define RS485_RX_PIN 40
#define RS485_TX_PIN 41
#define I2C_SDA 39
#define I2C_SCL 38

// --- CONSTANTS ---
#define MY_DEV_ID 0             
#define WATCHDOG_MS 1000        // Increased tolerance
#define MODBUS_ID 1             
#define HYSTERESIS_CM 20.0      // Reduced hysteresis for faster reaction
#define MAX_LOG_EVENTS 50 

// --- OBJECTS ---
AsyncWebServer server(80);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Preferences pref;
ModbusMaster node;
esp_now_peer_info_t peerInfo;

// --- DATA STRUCTURES ---

struct EventLog {
  char timestamp[16];
  float distance;
  String zone;
  int link_quality;
};
EventLog history[MAX_LOG_EVENTS];
int logHead = 0; 

struct SafetyState {
  float distance_cm;
  String status_msg;
  int safety_level; // 0=Safe, 1=Warn, 2=Stop, 3=CommLoss
  unsigned long last_packet_ms;
  int last_known_zone;
  int link_quality_pct; 
  // Relays
  bool relay_red;    
  bool relay_yellow; 
  bool relay_green;  
  bool relay_health; 
} sys;

struct Statistics {
  unsigned long red_entries;
  unsigned long yellow_entries;
  unsigned long comm_drops;
  int last_level_stats; 
} stats;

struct Config {
  int pan_id;
  float limit_warn_cm; 
  float limit_stop_cm; 
  String ap_ssid;
  String peer_mac; 
} cfg;

uint8_t peerMac[6];

typedef struct packet_t { 
  uint8_t type;         
  float dist;           
  uint8_t safety_lvl;   
} packet_t;
packet_t outgoingData;

// --- HELPER FUNCTIONS ---

String getUptimeStr() {
  unsigned long now = millis() / 1000;
  int sec = now % 60;
  int min = (now / 60) % 60;
  int hr = (now / 3600);
  char buf[20];
  sprintf(buf, "%02d:%02d:%02d", hr, min, sec);
  return String(buf);
}

void logEvent(float dist, String zoneName, int quality) {
  String t = getUptimeStr();
  t.toCharArray(history[logHead].timestamp, 16);
  history[logHead].distance = dist;
  history[logHead].zone = zoneName;
  history[logHead].link_quality = quality;
  
  logHead++;
  if(logHead >= MAX_LOG_EVENTS) logHead = 0; 
}

void OnDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {}

void parseBytes(const char* str, char sep, byte* bytes, int maxBytes, int base) {
  for (int i = 0; i < maxBytes; i++) { bytes[i] = strtoul(str, NULL, base); str = strchr(str, sep); if (str == NULL || *str == '\0') break; str++; }
}

// --- CONFIG & NVS ---
void loadConfig() {
  pref.begin("cg-a", false);
  cfg.pan_id = pref.getInt("pan", 33);
  cfg.limit_stop_cm = pref.getFloat("stop", 500.0); // Default 500cm (5m)
  cfg.limit_warn_cm = pref.getFloat("warn", 1500.0); // Default 1500cm (15m)
  cfg.ap_ssid = pref.getString("ap", "CRANE-GUARD-A");
  cfg.peer_mac = pref.getString("mac", "10:20:BA:4F:FD:94"); 
  pref.end();

  // SANITY CHECK: If limits are illogical (e.g., < 50cm), force defaults
  if(cfg.limit_stop_cm < 50) cfg.limit_stop_cm = 500.0;
  if(cfg.limit_warn_cm < 50) cfg.limit_warn_cm = 1500.0;

  parseBytes(cfg.peer_mac.c_str(), ':', peerMac, 6, 16);
}

void sendAT(String c, int t) { SERIAL_AT.println(c); unsigned long s=millis(); while(millis()-s<t){while(SERIAL_AT.available())SERIAL_AT.read();} }

// =======================================================================
// 1. DASHBOARD HTML
// =======================================================================
const char* html_page = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>CRANE GUARD - LIVE</title>
  <style>
    :root { --bg: #0b0c10; --card: #1f2833; --text: #c5c6c7; --safe: #45a29e; --warn: #ffc107; --stop: #e74c3c; --comm: #66fcf1; }
    body { background: var(--bg); color: var(--text); font-family: 'Segoe UI', sans-serif; margin:0; padding:0; }
    .navbar { background: #000; padding: 15px 20px; border-bottom: 3px solid var(--comm); display: flex; justify-content: space-between; align-items: center; }
    .brand { font-size: 1.5rem; font-weight: bold; letter-spacing: 2px; color: var(--comm); }
    .badge { background: #333; padding: 5px 10px; border-radius: 4px; font-size: 0.8rem; border: 1px solid #444; }
    .container { max-width: 800px; margin: 20px auto; padding: 10px; }
    .card { background: var(--card); border-radius: 10px; padding: 30px; box-shadow: 0 10px 20px rgba(0,0,0,0.5); border: 1px solid #2b3642; text-align: center; }
    .dist-box { padding: 40px 0; border: 4px solid #333; border-radius: 15px; background: #000; transition: all 0.3s; margin-bottom: 20px; }
    .dist-val { font-size: 7rem; font-weight: bold; color: #fff; line-height: 1; font-family: 'Courier New', monospace; }
    .dist-unit { font-size: 1.5rem; color: #666; display: block; }
    .status-msg { font-size: 1.5rem; font-weight: bold; margin-top: 10px; letter-spacing: 2px; }
    .lights { display: flex; gap: 10px; margin: 20px 0; justify-content: center; }
    .lamp { width: 50px; height: 50px; background: #333; border-radius: 50%; opacity: 0.2; border: 2px solid #555; transition: 0.2s; }
    .lamp.active.r { background: var(--stop); opacity: 1; box-shadow: 0 0 25px var(--stop); }
    .lamp.active.y { background: var(--warn); opacity: 1; box-shadow: 0 0 25px var(--warn); }
    .lamp.active.g { background: var(--safe); opacity: 1; box-shadow: 0 0 25px var(--safe); }
    .lamp.active.c { background: var(--comm); opacity: 1; box-shadow: 0 0 25px var(--comm); }
    .btn { width: 100%; padding: 20px; background: linear-gradient(45deg, #1e3c72, #2a5298); color: #fff; border: none; font-weight: bold; font-size: 1.1rem; cursor: pointer; border-radius: 5px; margin-top: 20px; text-transform: uppercase; }
    .link-bar { width: 100%; height: 10px; background: #000; border-radius: 5px; margin-top: 10px; overflow: hidden; }
    .link-fill { height: 100%; background: var(--comm); width: 0%; transition: width 0.5s; }
  </style>
</head>
<body>
  <div class="navbar"><div class="brand">CRANE GUARD <span style="font-size:0.8rem;">LIVE</span></div><div class="badge" id="wifi_rssi">WIFI: -- dBm</div></div>
  <div class="container">
    <div class="card">
      <div style="text-align:left; color:#888; margin-bottom:10px;">PROXIMITY SENSOR</div>
      <div class="dist-box" id="dbox">
        <div class="dist-val" id="d_val">--.--</div>
        <span class="dist-unit">METERS</span>
      </div>
      <div class="status-msg" id="d_msg">SYSTEM READY</div>
      <div class="lights">
        <div class="lamp r" id="l_r"></div>
        <div class="lamp y" id="l_y"></div>
        <div class="lamp g" id="l_g"></div>
        <div class="lamp c" id="l_c"></div>
      </div>
      <div style="text-align:left; font-size:0.8rem; color:#888; margin-top:20px;">
        LINK INTEGRITY: <span id="txt_link">0%</span>
        <div class="link-bar"><div class="link-fill" id="bar_link"></div></div>
      </div>
      <button class="btn" onclick="window.location.href='/stats'">VIEW DATA LOGS & SETTINGS</button>
    </div>
  </div>
<script>
  setInterval(()=>{
    fetch('/live').then(r=>r.json()).then(d=>{
      document.getElementById('d_val').innerText = (d.dist <= 0) ? "--.--" : (d.dist/100).toFixed(2);
      document.getElementById('d_msg').innerText = d.msg;
      let dbox = document.getElementById('dbox');
      let msg = document.getElementById('d_msg');
      dbox.style.borderColor = "#333"; msg.style.color = "#888";
      if(d.r_h) { dbox.style.borderColor = "var(--comm)"; msg.style.color = "var(--comm)"; }
      else if(d.r_r) { dbox.style.borderColor = "var(--stop)"; msg.style.color = "var(--stop)"; }
      else if(d.r_y) { dbox.style.borderColor = "var(--warn)"; msg.style.color = "var(--warn)"; }
      else { dbox.style.borderColor = "var(--safe)"; msg.style.color = "var(--safe)"; }
      document.getElementById('l_r').className = d.r_r ? 'lamp active r' : 'lamp';
      document.getElementById('l_y').className = d.r_y ? 'lamp active y' : 'lamp';
      document.getElementById('l_g').className = d.r_g ? 'lamp active g' : 'lamp';
      document.getElementById('l_c').className = d.r_h ? 'lamp active c' : 'lamp';
      document.getElementById('bar_link').style.width = d.link + "%";
      document.getElementById('txt_link').innerText = d.link + "%";
      document.getElementById('wifi_rssi').innerText = "WIFI: " + d.rssi + " dBm";
    });
  }, 200);
</script>
</body></html>
)rawliteral";

// =======================================================================
// 2. STATISTICS & CONFIG PAGE (With Unit Conversion)
// =======================================================================
const char* html_stats = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>CRANE GUARD - LOGS</title>
  <style>
    :root { --bg: #0b0c10; --card: #1f2833; --text: #c5c6c7; --safe: #45a29e; --warn: #ffc107; --stop: #e74c3c; --comm: #66fcf1; }
    body { background: var(--bg); color: var(--text); font-family: 'Segoe UI', monospace; margin:0; padding:20px; }
    h2 { border-bottom: 2px solid var(--comm); padding-bottom: 10px; color: var(--comm); }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 15px; margin-bottom: 30px; }
    .stat-box { background: #000; padding: 20px; border-radius: 8px; text-align: center; border: 1px solid #333; }
    .stat-val { font-size: 2rem; font-weight: bold; color: #fff; }
    .stat-lbl { font-size: 0.8rem; color: #888; text-transform: uppercase; margin-top: 5px; }
    table { width: 100%; border-collapse: collapse; background: #000; margin-bottom: 20px; }
    th, td { border: 1px solid #333; padding: 12px; text-align: left; }
    th { background: #1a1a1a; color: var(--comm); }
    tr:nth-child(even) { background: #111; }
    .btn { padding: 15px 30px; border: none; font-weight: bold; cursor: pointer; border-radius: 5px; text-decoration: none; display: inline-block; margin-right: 10px; }
    .btn-down { background: var(--safe); color: #000; }
    .btn-back { background: #333; color: #fff; }
    input { background: #000; border: 1px solid #444; color: #fff; padding: 10px; width: 100px; }
  </style>
</head>
<body>
  <h2>SYSTEM STATISTICS & LOGS</h2>
  <div class="grid">
    <div class="stat-box"><div class="stat-val" id="s_up">--:--</div><div class="stat-lbl">UPTIME</div></div>
    <div class="stat-box"><div class="stat-val" id="s_red" style="color:var(--stop)">0</div><div class="stat-lbl">STOP ZONES</div></div>
    <div class="stat-box"><div class="stat-val" id="s_yel" style="color:var(--warn)">0</div><div class="stat-lbl">WARN ZONES</div></div>
    <div class="stat-box"><div class="stat-val" id="s_drop" style="color:var(--comm)">0</div><div class="stat-lbl">COMM LOSS</div></div>
  </div>
  <h2>RECENT EVENT LOG (RAM)</h2>
  <table id="logTable">
    <tr><th>TIME</th><th>DISTANCE</th><th>ZONE</th><th>LINK %</th></tr>
    <tr><td colspan="4" style="text-align:center; color:#666">Loading History...</td></tr>
  </table>
  <div style="margin-top:30px; padding: 20px; background:#1f2833; border-radius:8px;">
    <h3 style="margin-top:0">PARAMETERS (METERS)</h3>
    STOP LIMIT: <input type="number" step="0.1" id="in_stop"> m &nbsp;&nbsp;
    WARN LIMIT: <input type="number" step="0.1" id="in_warn"> m &nbsp;&nbsp;
    <button class="btn btn-back" onclick="saveCfg()">SAVE SETTINGS</button>
  </div>
  <br>
  <a href="/download_csv" class="btn btn-down">DOWNLOAD FULL CSV LOG</a>
  <a href="/" class="btn btn-back">BACK TO DASHBOARD</a>
<script>
  // LOAD CONFIG: Convert CM from Server to Meters for Display
  fetch('/getconf').then(r=>r.json()).then(d=>{
    document.getElementById('in_stop').value = (d.stop / 100).toFixed(2);
    document.getElementById('in_warn').value = (d.warn / 100).toFixed(2);
  });

  function refresh() {
    fetch('/live').then(r=>r.json()).then(d=>{
      document.getElementById('s_up').innerText = d.s_up;
      document.getElementById('s_red').innerText = d.s_red;
      document.getElementById('s_yel').innerText = d.s_yel;
      document.getElementById('s_drop').innerText = d.s_drp;
      let h = d.hist;
      let html = '<tr><th>TIME</th><th>DISTANCE</th><th>ZONE</th><th>LINK %</th></tr>';
      if(h && h.length > 0) {
        for(let i=h.length-1; i>=0; i--) {
          if(h[i].d > 0) {
            let color = "#fff";
            if(h[i].z == "DANGER") color = "var(--stop)";
            if(h[i].z == "WARN") color = "var(--warn)";
            if(h[i].z == "LOSS") color = "var(--comm)";
            html += `<tr><td>${h[i].t}</td><td>${(h[i].d/100).toFixed(2)} m</td><td style="color:${color}; font-weight:bold">${h[i].z}</td><td>${h[i].q}%</td></tr>`;
          }
        }
      } else { html += '<tr><td colspan="4">No events logged yet.</td></tr>'; }
      document.getElementById('logTable').innerHTML = html;
    });
  }
  refresh(); setInterval(refresh, 2000);

  // SAVE CONFIG: Convert Meters from Input to CM for Server
  function saveCfg() {
    let s_m = document.getElementById('in_stop').value;
    let w_m = document.getElementById('in_warn').value;
    // CONVERSION HAPPENS HERE: METERS * 100 = CM
    let s_cm = parseFloat(s_m) * 100;
    let w_cm = parseFloat(w_m) * 100;
    let fd = new FormData();
    fd.append('stop', s_cm);
    fd.append('warn', w_cm);
    fetch('/save', {method:'POST', body:fd}).then(()=>{ alert('Settings Saved! Rebooting...'); window.location.reload(); });
  }
</script>
</body></html>
)rawliteral";

// --- CORE SAFETY LOGIC ---
void checkSafetyAndRelays() {
  unsigned long now = millis();
  unsigned long elapsed = now - sys.last_packet_ms;
  bool signal_lost = (elapsed > WATCHDOG_MS);

  // Calculate Link Quality
  if(elapsed < 100) sys.link_quality_pct = 100;
  else if(elapsed > 2000) sys.link_quality_pct = 0;
  else sys.link_quality_pct = map(elapsed, 100, 2000, 100, 0);

  if (!signal_lost) {
    bool cur_stop = (sys.safety_level == 2);
    bool cur_warn = (sys.safety_level == 1);
    // Hysteresis logic
    float eff_stop = cfg.limit_stop_cm + (cur_stop ? HYSTERESIS_CM : 0);
    float eff_warn = cfg.limit_warn_cm + (cur_warn ? HYSTERESIS_CM : 0);

    // KEY COMPARISON (All in CM)
    if (sys.distance_cm <= eff_stop) {
      sys.safety_level = 2; // STOP
      sys.status_msg = "DANGER: STOP";
    } else if (sys.distance_cm <= eff_warn) {
      sys.safety_level = 1; // WARN
      sys.status_msg = "WARNING: SLOW";
    } else {
      sys.safety_level = 0; // SAFE
      sys.status_msg = "SAFE DISTANCE";
    }
    sys.last_known_zone = sys.safety_level;
  } 
  else {
    // Failsafe Logic
    if (sys.last_known_zone == 0) {
      sys.safety_level = 0; sys.status_msg = "SAFE (NO LINK)";
    } else {
        sys.safety_level = 3; sys.status_msg = "SIGNAL LOST";
    }
  }

  // --- STATS LOGGING ENGINE ---
  // Only log when state CHANGES (e.g., Safe -> Warn)
  if (sys.safety_level != stats.last_level_stats) {
    if (sys.safety_level == 2) stats.red_entries++;
    if (sys.safety_level == 1) stats.yellow_entries++;
    if (sys.safety_level == 3) stats.comm_drops++;
    
    // Create Log Entry
    String zName = "SAFE";
    if(sys.safety_level == 1) zName = "WARN";
    if(sys.safety_level == 2) zName = "DANGER";
    if(sys.safety_level == 3) zName = "LOSS";
    
    // Only log if distance is valid or it's a signal loss
    if(sys.distance_cm > 0 || sys.safety_level == 3) {
      logEvent(sys.distance_cm, zName, sys.link_quality_pct);
    }
    
    stats.last_level_stats = sys.safety_level;
  }

  // Relay Output
  sys.relay_red = (sys.safety_level == 2);
  sys.relay_yellow = (sys.safety_level == 1);
  sys.relay_green = (sys.safety_level == 0);
  sys.relay_health = (sys.safety_level == 3);

  // Modbus Write
  static uint8_t lastCoilStatus = 0xFF;
  static unsigned long lastModbusTime = 0;
  uint8_t coilStatus = 0;
  if (sys.relay_red)     coilStatus |= (1 << 0);
  if (sys.relay_yellow) coilStatus |= (1 << 1);
  if (sys.relay_green)  coilStatus |= (1 << 2);
  if (sys.relay_health) coilStatus |= (1 << 3);

  if (coilStatus != lastCoilStatus || (now - lastModbusTime > 500)) {
    node.setTransmitBuffer(0, coilStatus); 
    node.writeMultipleCoils(2, 4); 
    lastCoilStatus = coilStatus;
    lastModbusTime = now;
  }
}

// --- UWB TASK ---
void uwb_task(void *p) {
  String b=""; 
  for(;;) { 
    while(SERIAL_AT.available()){ 
      char c=SERIAL_AT.read(); 
      if(c=='\n'){
        if(b.startsWith("AT+RANGE=")){
          int rIdx = b.indexOf("range:(");
          if(rIdx > 0) {
            float d = b.substring(rIdx+7).toFloat();
            // FILTER: If UWB returns 0 or garbage, ignore
            if(d > 0.1) {
              sys.distance_cm = d; 
              sys.last_packet_ms = millis();
              
              outgoingData.type = 0xAA;
              outgoingData.dist = d;
              outgoingData.safety_lvl = sys.safety_level; 
              esp_now_send(peerMac, (uint8_t *) &outgoingData, sizeof(outgoingData));
            }
          }
        } 
        b="";
      } else if(c!='\r') b+=c; 
    } 
    vTaskDelay(1 / portTICK_PERIOD_MS); 
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(UWB_RESET_PIN, OUTPUT); digitalWrite(UWB_RESET_PIN, HIGH);
  
  SERIAL_AT.begin(115200, SERIAL_8N1, IO_RXD2, IO_TXD2);
  Serial1.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  node.begin(MODBUS_ID, Serial1);

  Wire.begin(I2C_SDA, I2C_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  
  loadConfig();
  
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(cfg.ap_ssid.c_str(), "12345678", 6); 

  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(OnDataSent); 
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, peerMac, 6); 
    peerInfo.channel = 6; 
    peerInfo.encrypt = false; 
    esp_now_add_peer(&peerInfo);
  }

  // ROUTES
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(200, "text/html", html_page); });
  server.on("/stats", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(200, "text/html", html_stats); });
  
  server.on("/download_csv", HTTP_GET, [](AsyncWebServerRequest *r){
    String csv = "Timestamp,Distance_m,Zone_Status,Link_Quality_Pct\n";
    for(int i=0; i<MAX_LOG_EVENTS; i++) {
      if(history[i].distance != 0) { 
        csv += String(history[i].timestamp) + ",";
        csv += String(history[i].distance/100.0) + ",";
        csv += history[i].zone + ",";
        csv += String(history[i].link_quality) + "\n";
      }
    }
    AsyncWebServerResponse *response = r->beginResponse(200, "text/csv", csv);
    response->addHeader("Content-Disposition", "attachment; filename=crane_logs.csv");
    r->send(response);
  });

  server.on("/live", HTTP_GET, [](AsyncWebServerRequest *r){
    StaticJsonDocument<2048> d;
    d["dist"]=sys.distance_cm; 
    d["msg"]=sys.status_msg;
    d["r_r"]=sys.relay_red; d["r_y"]=sys.relay_yellow; d["r_g"]=sys.relay_green; d["r_h"]=sys.relay_health;
    d["s_up"]=getUptimeStr(); d["s_red"]=stats.red_entries; d["s_yel"]=stats.yellow_entries; d["s_drp"]=stats.comm_drops;
    d["link"]=sys.link_quality_pct;
    d["rssi"]=WiFi.RSSI();
    
    JsonArray hist = d.createNestedArray("hist");
    for(int i=0; i<MAX_LOG_EVENTS; i++) {
        if(history[i].distance > 0) {
            JsonObject obj = hist.createNestedObject();
            obj["t"] = history[i].timestamp;
            obj["d"] = history[i].distance;
            obj["z"] = history[i].zone;
            obj["q"] = history[i].link_quality;
        }
    }
    String j; serializeJson(d, j); r->send(200, "application/json", j);
  });
  
  server.on("/getconf", HTTP_GET, [](AsyncWebServerRequest *r){
    StaticJsonDocument<256> d;
    d["stop"] = cfg.limit_stop_cm;
    d["warn"] = cfg.limit_warn_cm;
    String j; serializeJson(d, j); r->send(200, "application/json", j);
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *r){
     // Receive CM from client (already multiplied by 100 in JS)
     if(r->hasParam("stop", true)) cfg.limit_stop_cm = r->getParam("stop", true)->value().toFloat();
     if(r->hasParam("warn", true)) cfg.limit_warn_cm = r->getParam("warn", true)->value().toFloat();
     
     pref.begin("cg-a", false);
     pref.putFloat("stop", cfg.limit_stop_cm);
     pref.putFloat("warn", cfg.limit_warn_cm);
     pref.end();
     r->send(200, "text/plain", "OK");
     delay(500); ESP.restart();
  });

  server.begin();
  xTaskCreatePinnedToCore(uwb_task, "UWB", 4096, NULL, 1, NULL, 1);

  // UWB Init Screen
  display.clearDisplay(); display.setCursor(0,0);
  display.setTextSize(2); display.setTextColor(WHITE);
  display.println("CRANE GUARD"); 
  display.setTextSize(1); display.println("V2.2 - INIT");
  display.display();
  
  sendAT("AT+RESTORE", 2000); 
  delay(1000);
  sendAT("AT+SETCFG="+String(MY_DEV_ID)+",1,1,1", 1000);
  sendAT("AT+SETPAN="+String(cfg.pan_id), 1000);
  sendAT("AT+SAVE", 1000);
  sendAT("AT+RESTART", 2000);
}

void loop() {
  checkSafetyAndRelays();

  static unsigned long last_oled = 0;
  if(millis() - last_oled > 200) {                  
    display.clearDisplay(); 
    
    // Header
    display.setTextSize(1); display.setTextColor(WHITE); display.setCursor(0,0);
    display.print("RSSI:"); display.print(WiFi.RSSI()); display.print(" | "); display.print(getUptimeStr());
    display.drawLine(0, 10, 128, 10, WHITE);
    
    // Status
    if(sys.safety_level == 2) {
      display.setTextSize(2); display.setCursor(15, 20); display.print("STOP !!!");
    } else if(sys.safety_level == 1) {
      display.setTextSize(2); display.setCursor(10, 20); display.print("WARNING");
    } else if(sys.safety_level == 3) {
      display.setTextSize(2); display.setCursor(5, 20); display.print("NO SIGNAL");
    } else {
      display.setTextSize(2); display.setCursor(30, 20); display.print("SAFE");
    }
    
    // Distance
    display.setTextSize(1); display.setCursor(0, 48); display.print("DIST: ");
    display.setTextSize(2); 
    if(sys.distance_cm > 0 && sys.safety_level != 3) {
      display.print(sys.distance_cm/100.0, 2);
      display.setTextSize(1); display.print("m");
    } else {
      display.print("--.--");
    }
    display.display();
    last_oled = millis();
  }
}