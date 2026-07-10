/*
 * buddy_firmware.ino — Buddy v1.0
 * ─────────────────────────────────────────────────────────────────────────────
 * Target board : AI Thinker ESP32-CAM  (+ ESP32-CAM-MB programmer)
 *
 * Required libraries (Tools → Manage Libraries):
 *   WebSockets              by Markus Sattler   (>= 2.4.0)
 *   ArduinoJson             by Benoit Blanchon   (>= 6.x)
 *   Adafruit PWM Servo Driver by Adafruit        (>= 2.4.0)
 *   Adafruit BusIO          by Adafruit          (>= 1.14.0)
 *
 * Board package : esp32 by Espressif (>= 2.0.14)
 * Board select  : AI Thinker ESP32-CAM
 * PSRAM         : Tools → PSRAM → Enabled   ← 4MB QSPI PSRAM
 *
 * Hardware on the ESP32-CAM (no extra wiring needed):
 *   Camera  OV2640  — onboard
 *   Flash LED       — GPIO4 (HIGH = on, avoid using as GPIO while camera runs)
 *
 * External wiring needed:
 *   PCA9685 servo driver  → SDA=GPIO14  SCL=GPIO13  VCC=3.3V  GND=GND
 *     3 continuous-rotation wheel servos (pots removed), tangent to chassis
 *     center, positioned at 0°/120°/240° (clockwise from front, top-down view):
 *       Servo 0 → PCA channel 0  (front, 0°)
 *       Servo 1 → PCA channel 1  (120°)
 *       Servo 2 → PCA channel 2  (240°)
 *   MAX98357A speaker amp → BCLK=GPIO2  LRC=GPIO13  DIN=GPIO12
 *   NOTE: GPIO12 must be LOW at boot — ensure amp DIN is not driving HIGH on power-on
 *
 * Commands (JSON over WebSocket):
 *   { "type": "cmd",   "dir": "fwd"|"back"|"left"|"right"|"stop" }  — holonomic drive
 *   { "type": "servo", "ch": 0, "angle": 90 }                       — raw per-wheel override, ch: 0-2, angle: 0-180
 *
 * Config block is patched by the Buddy flash tool before writing.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "driver/i2s.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ─── Config block ─────────────────────────────────────────────────────────────
// DO NOT reorder fields. Flash tool patches at fixed offsets from magic bytes.

struct __attribute__((packed)) BuddyConfig {
  uint8_t magic[8] = { 0xBD, 0xBD, 0xBD, 0xBD, 0x42, 0x55, 0x44, 0x59 };
  char ssid[32] = "Hack Club";
  char pass[64] = "bettertobeapiratethanjointhenavy";
  char host[40] = "buddy-production-948c.up.railway.app";
  uint32_t port = 443;
  char id[16] = "BDY-00001";
} cfg;

// ─── Camera pins (AI Thinker ESP32-CAM / OV2640) ─────────────────────────────
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ─── I2S speaker (external MAX98357A) ─────────────────────────────────────────
// No onboard mic on ESP32-CAM — mic task is disabled.
#define SPK_I2S     I2S_NUM_1
#define SPK_BCLK     2         // GPIO2
#define SPK_LRC     13         // GPIO13
#define SPK_DIN     12         // GPIO12 — must be LOW at boot (keep amp DIN floating/low until after init)
#define SPK_RATE    16000

// ─── PCA9685 servo driver ─────────────────────────────────────────────────────
// I2C: SDA=GPIO14, SCL=GPIO13 — both broken out on ESP32-CAM header, no conflicts.
// PCA9685 default I2C address is 0x40 (all address pins tied low).
// Pulse range calibrated for standard 50Hz hobby servos:
//   SERVO_MIN (~500 µs) = 0°,  SERVO_MAX (~2500 µs) = 180°
#define SERVO_FREQ    50
#define SERVO_MIN    150    // PCA9685 tick count for 0°  (~500 µs)
#define SERVO_MAX    600    // PCA9685 tick count for 180° (~2500 µs)
#define NUM_SERVOS     3    // channels 0, 1, 2

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ─── Frame type bytes ─────────────────────────────────────────────────────────
#define TYPE_VIDEO  0x01
#define TYPE_AUDIO  0x02

// ─── Globals ──────────────────────────────────────────────────────────────────
WebSocketsClient ws;
volatile bool wsLive     = false;
volatile bool peerOnline = false;
bool camReady = false;
bool spkReady = false;

struct Frame { uint8_t* buf; size_t len; };
QueueHandle_t txQueue;
#define TX_QUEUE_DEPTH 2   // keep small to minimise frame-buffer lag

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static bool queueFrame(uint8_t* buf, size_t len) {
  Frame f = { buf, len };
  if (xQueueSend(txQueue, &f, 0) == pdTRUE) return true;
  free(buf);
  return false;
}

// angle 0–180 → PCA9685 tick, then write to the given channel.
static void setServo(uint8_t ch, int angle) {
  if (ch >= NUM_SERVOS) return;
  angle = constrain(angle, 0, 180);
  uint16_t pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pca.setPWM(ch, 0, pulse);
  Serial.printf("[srv] ch%u → %d°\n", ch, angle);
}

// Per-channel true "stop" offset from 90°, in degrees. These are pot-removed
// continuous-rotation conversions — the pot was decoupled from the shaft, not
// removed from the circuit, so each servo's real neutral point is whatever
// fixed position that pot was left at, not necessarily exactly 90°. Calibrate
// per wheel: send { "type": "servo", "ch": i, "angle": 90 } and nudge the
// angle up/down until that wheel truly stops creeping, then set the offset
// here (measured_stop_angle - 90).
static const int SERVO_TRIM[NUM_SERVOS] = { 0, 0, 0 };

static void centerServos() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    setServo(i, 90 + SERVO_TRIM[i]);
  }
}

// ─── Holonomic drive mixing ────────────────────────────────────────────────────
// 3 continuous-rotation wheel servos, tangent to the chassis center, positioned
// at 0° / 120° / 240° (clockwise from front, top-down view). 90°+trim = stop.
#define DRIVE_DEG 60   // max deflection from center at full commanded speed

// Tangential drive-direction unit vector per wheel, in (forward, right) frame —
// i.e. -sin(pos), cos(pos) for pos = 0°, 120°, 240°.
static const float WHEEL_DX[NUM_SERVOS] = {  0.0f,      -0.8660254f,  0.8660254f };
static const float WHEEL_DY[NUM_SERVOS] = {  1.0f,      -0.5f,       -0.5f };

static void driveServos(float vx, float vy) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    float speed = WHEEL_DX[i] * vx + WHEEL_DY[i] * vy;  // -1..1
    setServo(i, 90 + SERVO_TRIM[i] + (int)(speed * DRIVE_DEG));
  }
}

static void driveCmd(const char* dir) {
  if (strcmp(dir, "fwd") == 0)         driveServos( 1,  0);
  else if (strcmp(dir, "back") == 0)   driveServos(-1,  0);
  else if (strcmp(dir, "left") == 0)   driveServos( 0, -1);
  else if (strcmp(dir, "right") == 0)  driveServos( 0,  1);
  else                                 driveServos( 0,  0);  // stop / unknown
}

// ─────────────────────────────────────────────────────────────────────────────
// WebSocket event handler
// ─────────────────────────────────────────────────────────────────────────────
void onWsEvent(WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED:
      wsLive = true;
      Serial.printf("[ws]  connected — id: %s\n", cfg.id);
      break;

    case WStype_DISCONNECTED:
      wsLive = false;
      peerOnline = false;
      centerServos();
      Serial.println("[ws]  disconnected");
      break;

    case WStype_TEXT:
      {
        StaticJsonDocument<128> doc;
        if (deserializeJson(doc, payload, len) != DeserializationError::Ok) break;
        const char* t = doc["type"] | "";
        if (strcmp(t, "client_connected") == 0) {
          peerOnline = true;
          Serial.println("[ws]  browser online");
        } else if (strcmp(t, "client_disconnected") == 0) {
          peerOnline = false;
          Serial.println("[ws]  browser offline");
          centerServos();
        } else if (strcmp(t, "servo") == 0) {
          // { "type": "servo", "ch": 0, "angle": 90 }
          setServo((uint8_t)(doc["ch"] | 0), (int)(doc["angle"] | 90));
        } else if (strcmp(t, "cmd") == 0) {
          // { "type": "cmd", "dir": "fwd"|"back"|"left"|"right"|"stop" }
          driveCmd(doc["dir"] | "stop");
        }
        break;
      }

    case WStype_BIN:
      if (spkReady && len > 1 && payload[0] == TYPE_AUDIO) {
        size_t written = 0;
        i2s_write(SPK_I2S, payload + 1, len - 1, &written, pdMS_TO_TICKS(20));
      }
      break;

    default: break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Camera task (Core 0)
// ─────────────────────────────────────────────────────────────────────────────
void cameraTask(void*) {
  const TickType_t interval = pdMS_TO_TICKS(50);  // ~20 fps — affordable at QVGA
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    if (peerOnline) {
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb && fb->format == PIXFORMAT_JPEG) {
        uint8_t* buf = (uint8_t*)malloc(1 + fb->len);
        if (buf) {
          buf[0] = TYPE_VIDEO;
          memcpy(buf + 1, fb->buf, fb->len);
          queueFrame(buf, 1 + fb->len);
        }
      }
      if (fb) esp_camera_fb_return(fb);
    }
    vTaskDelayUntil(&wake, interval);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Hardware init
// ─────────────────────────────────────────────────────────────────────────────
void initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk     = XCLK_GPIO_NUM;
  c.pin_pclk     = PCLK_GPIO_NUM;
  c.pin_vsync    = VSYNC_GPIO_NUM;
  c.pin_href     = HREF_GPIO_NUM;
  c.pin_sscb_sda = SIOD_GPIO_NUM;
  c.pin_sscb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn     = PWDN_GPIO_NUM;
  c.pin_reset    = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_QVGA;  // 320×240 — smaller frames cut WS latency significantly
  c.jpeg_quality = 20;              // 0-63, higher = more compression/smaller file
  c.fb_count     = 3;               // 3 buffers + GRAB_LATEST keeps pipeline non-blocking
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&c) != ESP_OK) {
    Serial.println("[cam] init FAILED — check expansion board is attached"); return;
  }
  sensor_t* s = esp_camera_sensor_get();
  s->set_whitebal(s, 1); s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1); s->set_gain_ctrl(s, 1);
  s->set_raw_gma(s, 1);  s->set_lenc(s, 1);
  s->set_vflip(s, 0);    s->set_hmirror(s, 0);
  camReady = true;
  Serial.println("[cam] ready");
}

void initSpeaker() {
  Serial.println("[spk] skipped — not available on ESP32-CAM with camera active");
  return;
  // Remove the early return above if using an ESP32 board without a camera.
  i2s_config_t sc = {};
  sc.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  sc.sample_rate = SPK_RATE;
  sc.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  sc.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  sc.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  sc.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  sc.dma_buf_count        = 4;
  sc.dma_buf_len          = 256;
  sc.use_apll             = false;
  sc.tx_desc_auto_clear   = true;

  i2s_pin_config_t sp = {
    .bck_io_num   = SPK_BCLK,
    .ws_io_num    = SPK_LRC,
    .data_out_num = SPK_DIN,
    .data_in_num  = I2S_PIN_NO_CHANGE
  };

  if (i2s_driver_install(SPK_I2S, &sc, 0, NULL) != ESP_OK || i2s_set_pin(SPK_I2S, &sp) != ESP_OK) {
    Serial.println("[spk] init FAILED — audio TX disabled");
    return;
  }
  spkReady = true;
  Serial.println("[spk] ready");
}

void initServos() {
  Wire.begin(14, 13);                 // SDA=GPIO14, SCL=GPIO13 (ESP32-CAM safe pins)
  pca.begin();
  pca.setOscillatorFrequency(27000000);  // tune to actual PCA9685 oscillator (±10%)
  pca.setPWMFreq(SERVO_FREQ);
  delay(10);
  centerServos();
  Serial.println("[srv] PCA9685 ready — 3 servos centred");
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // disable brownout detector

  Serial.begin(115200);
  delay(500);  // let serial settle before first print
  Serial.printf("\n[buddy] booting — id: %s\n", cfg.id);

  if (!psramFound()) {
    Serial.println("[buddy] WARNING: no PSRAM detected — camera may fail");
  } else {
    Serial.printf("[buddy] PSRAM: %u bytes free\n", ESP.getFreePsram());
  }

  txQueue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(Frame));

  initCamera();
  initSpeaker();
  initServos();

  // WiFi
  Serial.printf("[wifi] connecting to \"%s\"\n", cfg.ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.ssid, cfg.pass);
  WiFi.setAutoReconnect(true);
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\n[wifi] %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("\n[wifi] timed out — will retry");

  // WebSocket — use WSS (port 443) for Railway, plain WS (cfg.port) for local
  String path = String("/ws?id=") + cfg.id + "&role=device";
  bool useSSL = (cfg.port == 443);
  Serial.printf("[ws]  → %s:%u%s (SSL: %s)\n", cfg.host, cfg.port, path.c_str(), useSSL ? "yes" : "no");
  if (useSSL) {
    ws.beginSSL(cfg.host, 443, path.c_str());
  } else {
    ws.begin(cfg.host, (uint16_t)cfg.port, path.c_str());
  }
  ws.onEvent(onWsEvent);
  ws.setReconnectInterval(3000);
  ws.enableHeartbeat(15000, 3000, 2);

  xTaskCreatePinnedToCore(cameraTask, "cam", 8192, NULL, 2, NULL, 0);
  // micTask omitted — ESP32-CAM has no onboard microphone

  Serial.println("[buddy] running");
}

// ─────────────────────────────────────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  Frame f;
  while (xQueueReceive(txQueue, &f, 0) == pdTRUE) {
    if (wsLive && peerOnline) ws.sendBIN(f.buf, f.len);
    free(f.buf);
  }

  ws.loop();

  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }
}
