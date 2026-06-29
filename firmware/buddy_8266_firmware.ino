/*
 * buddy_8266_firmware.ino — Buddy v1.0 (ESP8266)
 * ─────────────────────────────────────────────────────────────────────────────
 * Target board : Generic ESP8266 / NodeMCU / Wemos D1 Mini
 *
 * Required libraries (Tools → Manage Libraries):
 *   WebSockets  by Markus Sattler  (>= 2.4.0)
 *   ArduinoJson by Benoit Blanchon  (>= 6.x)
 *
 * Board package: esp8266 by ESP8266 Community (>= 3.1.0)
 * Board select : NodeMCU 1.0 (ESP-12E) or Generic ESP8266
 *
 * Wiring (NodeMCU / Wemos D1 Mini):
 *   Motor driver  → see MOTOR_* defines below
 *   Status LED    → D4 (GPIO2, active-low onboard LED)
 *
 * GPIO map (NodeMCU label → actual GPIO):
 *   D0=GPIO16  D1=GPIO5  D2=GPIO4  D3=GPIO0
 *   D4=GPIO2   D5=GPIO14 D6=GPIO12 D7=GPIO13  D8=GPIO15
 *
 * Config block is patched by the Buddy flash tool before writing.
 * All WiFi / host / ID values below are overwritten at flash time.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

// ─── Config block ─────────────────────────────────────────────────────────────
// DO NOT reorder fields. The flash tool finds the magic bytes and patches
// ssid/pass/host/port/id at fixed offsets from that marker.

struct __attribute__((packed)) BuddyConfig {
  uint8_t  magic[8] = {0xBD,0xBD,0xBD,0xBD,0x42,0x55,0x44,0x59};
  char     ssid[32] = "YOUR_SSID";
  char     pass[64] = "YOUR_PASS";
  char     host[40] = "192.168.1.x";
  uint32_t port     = 3000;
  char     id[16]   = "BDY-8266-001";
} cfg;

// ─── Motor driver pins (L298N or similar) ─────────────────────────────────────
// NodeMCU: D5=GPIO14, D6=GPIO12, D7=GPIO13, D8=GPIO15
#define MTR_A_FWD  14   // D5 — Motor A forward
#define MTR_A_BWD  12   // D6 — Motor A reverse
#define MTR_B_FWD  13   // D7 — Motor B forward
#define MTR_B_BWD  15   // D8 — Motor B reverse

// ─── Onboard LED (active LOW on GPIO2 / D4) ───────────────────────────────────
#define STATUS_LED  2

// ─── Blink pattern codes ──────────────────────────────────────────────────────
#define BLINK_CONNECTING  250   // fast — searching for WiFi
#define BLINK_NO_PEER    1000   // slow — connected to hub, waiting for browser
#define BLINK_ACTIVE        0   // solid on — browser paired

// ─── Globals ──────────────────────────────────────────────────────────────────
WebSocketsClient ws;

bool wsLive     = false;
bool peerOnline = false;

unsigned long lastBlink    = 0;
bool          ledState     = false;
unsigned long blinkInterval = BLINK_CONNECTING;

// ─────────────────────────────────────────────────────────────────────────────
// Motor helpers
// ─────────────────────────────────────────────────────────────────────────────
static void stopMotors() {
  digitalWrite(MTR_A_FWD, LOW);
  digitalWrite(MTR_A_BWD, LOW);
  digitalWrite(MTR_B_FWD, LOW);
  digitalWrite(MTR_B_BWD, LOW);
}

static void driveMotors(const char* dir) {
  stopMotors();
  if      (strcmp(dir, "fwd")   == 0) { digitalWrite(MTR_A_FWD,HIGH); digitalWrite(MTR_B_FWD,HIGH); }
  else if (strcmp(dir, "back")  == 0) { digitalWrite(MTR_A_BWD,HIGH); digitalWrite(MTR_B_BWD,HIGH); }
  else if (strcmp(dir, "left")  == 0) { digitalWrite(MTR_A_BWD,HIGH); digitalWrite(MTR_B_FWD,HIGH); }
  else if (strcmp(dir, "right") == 0) { digitalWrite(MTR_A_FWD,HIGH); digitalWrite(MTR_B_BWD,HIGH); }
  Serial.printf("[mtr] %s\n", dir);
}

// ─────────────────────────────────────────────────────────────────────────────
// Status LED
// ─────────────────────────────────────────────────────────────────────────────
static void updateLed() {
  if (blinkInterval == BLINK_ACTIVE) {
    // Solid on (active LOW → write LOW)
    digitalWrite(STATUS_LED, LOW);
    return;
  }
  unsigned long now = millis();
  if (now - lastBlink >= blinkInterval) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(STATUS_LED, ledState ? LOW : HIGH); // active LOW
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// WebSocket event handler
// ─────────────────────────────────────────────────────────────────────────────
void onWsEvent(WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {

    case WStype_CONNECTED:
      wsLive        = true;
      blinkInterval = BLINK_NO_PEER;
      Serial.printf("[ws]  connected to hub as %s\n", cfg.id);
      break;

    case WStype_DISCONNECTED:
      wsLive        = false;
      peerOnline    = false;
      blinkInterval = BLINK_CONNECTING;
      stopMotors();
      Serial.println("[ws]  disconnected — reconnecting...");
      break;

    case WStype_TEXT: {
      StaticJsonDocument<128> doc;
      if (deserializeJson(doc, payload, len) != DeserializationError::Ok) break;
      const char* t = doc["type"] | "";
      if (strcmp(t, "client_connected") == 0) {
        peerOnline    = true;
        blinkInterval = BLINK_ACTIVE;
        Serial.println("[ws]  browser online");
      } else if (strcmp(t, "client_disconnected") == 0) {
        peerOnline    = false;
        blinkInterval = BLINK_NO_PEER;
        stopMotors();
        Serial.println("[ws]  browser offline");
      } else if (strcmp(t, "cmd") == 0) {
        driveMotors(doc["dir"] | "stop");
      }
      break;
    }

    // ESP8266 has no I2S speaker — ignore incoming audio frames
    case WStype_BIN:
      break;

    default: break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.printf("\n[buddy] booting (ESP8266) — id: %s\n", cfg.id);

  // Motor pins
  pinMode(MTR_A_FWD, OUTPUT); pinMode(MTR_A_BWD, OUTPUT);
  pinMode(MTR_B_FWD, OUTPUT); pinMode(MTR_B_BWD, OUTPUT);
  stopMotors();
  Serial.println("[mtr] ready");

  // Status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH); // off (active LOW)

  // WiFi
  Serial.printf("[wifi] connecting to \"%s\"\n", cfg.ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(cfg.ssid, cfg.pass);
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
    updateLed();
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\n[wifi] connected — %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("\n[wifi] timed out — will retry");

  // WebSocket
  String path = String("/ws?id=") + cfg.id + "&role=device";
  Serial.printf("[ws]  → %s:%u%s\n", cfg.host, cfg.port, path.c_str());
  ws.begin(cfg.host, (uint16_t)cfg.port, path.c_str());
  ws.onEvent(onWsEvent);
  ws.setReconnectInterval(3000);
  ws.enableHeartbeat(15000, 3000, 2);

  Serial.println("[buddy] running");
}

// ─────────────────────────────────────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  ws.loop();
  updateLed();

  // WiFi watchdog
  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 5000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }
}
