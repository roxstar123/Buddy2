/*
 * buddy_firmware.ino — Buddy v1.0
 * ─────────────────────────────────────────────────────────────────────────────
 * Target board : AI-Thinker ESP32-CAM
 *
 * Required libraries (Tools → Manage Libraries):
 *   WebSockets  by Markus Sattler  (>= 2.4.0)
 *   ArduinoJson by Benoit Blanchon  (>= 6.x)
 *
 * Board package: esp32 by Espressif (>= 2.0.0)
 * Board select : AI Thinker ESP32-CAM
 *
 * Wiring (AI-Thinker ESP32-CAM, no SD card):
 *   INMP441 mic  → WS=GPIO15  SCK=GPIO14  SD=GPIO13
 *   MAX98357 spk → BCLK=GPIO12  LRC=GPIO2  DIN=GPIO4
 *     note: GPIO2 and GPIO4 are the onboard LEDs — they will flicker with audio
 *   Motors       → see MOTOR_* defines below (change to match your driver)
 *
 * Config block is patched by the Buddy flash tool before writing.
 * All WiFi / host / ID values below are overwritten at flash time.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "driver/i2s.h"

// ─── Config block ─────────────────────────────────────────────────────────────
// DO NOT reorder fields. The flash tool finds the magic bytes and patches
// ssid/pass/host/port/id at fixed offsets from that marker.

struct __attribute__((packed)) BuddyConfig {
  uint8_t  magic[8] = {0xBD,0xBD,0xBD,0xBD,0x42,0x55,0x44,0x59};
  char     ssid[32] = "YOUR_SSID";
  char     pass[64] = "YOUR_PASS";
  char     host[40] = "192.168.1.x";
  uint32_t port     = 3000;
  char     id[16]   = "BDY-00001";
} cfg;

// ─── Camera pins (AI-Thinker ESP32-CAM) ──────────────────────────────────────
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

// ─── I2S mic (INMP441) ───────────────────────────────────────────────────────
#define MIC_I2S     I2S_NUM_0
#define MIC_WS      15
#define MIC_SCK     14
#define MIC_SD      13
#define MIC_RATE    16000
#define MIC_MS      30    // chunk size in ms  →  30ms × 16000 = 480 samples

// ─── I2S speaker (MAX98357A) ─────────────────────────────────────────────────
#define SPK_I2S     I2S_NUM_1
#define SPK_BCK     12
#define SPK_LRC      2
#define SPK_DIN      4
#define SPK_RATE    16000

// ─── Motor driver (L298N or similar) ─────────────────────────────────────────
// Change these to match your wiring.
// ESP32-CAM has limited free GPIO; wire to an I2C motor driver if needed.
#define MTR_A_FWD   16   // Motor A forward
#define MTR_A_BWD    3   // Motor A reverse
#define MTR_B_FWD    1   // Motor B forward
#define MTR_B_BWD   16   // Motor B reverse
// For a standard ESP32 DevKit use e.g. 25/26/27/14

// ─── Frame type bytes (must match client.js) ─────────────────────────────────
#define TYPE_VIDEO  0x01
#define TYPE_AUDIO  0x02

// ─── Globals ──────────────────────────────────────────────────────────────────
WebSocketsClient ws;

volatile bool wsLive     = false;   // server connection up
volatile bool peerOnline = false;   // browser client paired

// Thread-safe send queue — tasks push here, main loop drains
struct Frame { uint8_t* buf; size_t len; };
QueueHandle_t txQueue;
#define TX_QUEUE_DEPTH 3

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

// Push a heap-allocated buffer to the send queue.
// Caller must NOT free buf — queue owns it.  Returns false and frees on overflow.
static bool queueFrame(uint8_t* buf, size_t len) {
  Frame f = {buf, len};
  if (xQueueSend(txQueue, &f, 0) == pdTRUE) return true;
  free(buf);
  return false;
}

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
// WebSocket event handler
// ─────────────────────────────────────────────────────────────────────────────
void onWsEvent(WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {

    case WStype_CONNECTED:
      wsLive = true;
      Serial.printf("[ws]  connected to hub as %s\n", cfg.id);
      break;

    case WStype_DISCONNECTED:
      wsLive     = false;
      peerOnline = false;
      stopMotors();
      Serial.println("[ws]  disconnected — reconnecting...");
      break;

    case WStype_TEXT: {
      StaticJsonDocument<128> doc;
      if (deserializeJson(doc, payload, len) != DeserializationError::Ok) break;
      const char* t = doc["type"] | "";
      if      (strcmp(t, "client_connected")    == 0) { peerOnline = true;  Serial.println("[ws]  browser online");  }
      else if (strcmp(t, "client_disconnected") == 0) { peerOnline = false; Serial.println("[ws]  browser offline"); stopMotors(); }
      else if (strcmp(t, "cmd")                 == 0) { driveMotors(doc["dir"] | "stop"); }
      break;
    }

    case WStype_BIN:
      // Incoming audio from browser mic → play on speaker
      if (len > 1 && payload[0] == TYPE_AUDIO) {
        size_t written = 0;
        i2s_write(SPK_I2S, payload + 1, len - 1, &written, pdMS_TO_TICKS(20));
      }
      break;

    default: break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Camera task  (Core 0)  — captures JPEG, pushes to tx queue
// ─────────────────────────────────────────────────────────────────────────────
void cameraTask(void*) {
  const TickType_t interval = pdMS_TO_TICKS(66); // ~15 fps
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
// Mic task  (Core 1)  — reads I2S PCM, pushes to tx queue
// ─────────────────────────────────────────────────────────────────────────────
void micTask(void*) {
  const int   nSamples  = MIC_RATE * MIC_MS / 1000;   // 480
  const size_t rawBytes  = nSamples * sizeof(int32_t);  // 32-bit read
  const size_t pcmBytes  = nSamples * sizeof(int16_t);  // 16-bit output

  int32_t* raw = (int32_t*)malloc(rawBytes);
  if (!raw) { Serial.println("[mic] malloc fail"); vTaskDelete(NULL); }

  for (;;) {
    size_t got = 0;
    i2s_read(MIC_I2S, raw, rawBytes, &got, pdMS_TO_TICKS(200));
    if (got == 0 || !peerOnline) continue;

    int n = got / sizeof(int32_t);
    uint8_t* buf = (uint8_t*)malloc(1 + n * sizeof(int16_t));
    if (!buf) continue;

    buf[0] = TYPE_AUDIO;
    int16_t* out = (int16_t*)(buf + 1);
    for (int i = 0; i < n; i++) out[i] = (int16_t)(raw[i] >> 11); // 32→16 bit

    queueFrame(buf, 1 + n * sizeof(int16_t));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Hardware init
// ─────────────────────────────────────────────────────────────────────────────
void initCamera() {
  camera_config_t c = {};
  c.ledc_channel    = LEDC_CHANNEL_0;
  c.ledc_timer      = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk        = XCLK_GPIO_NUM;
  c.pin_pclk        = PCLK_GPIO_NUM;
  c.pin_vsync       = VSYNC_GPIO_NUM;
  c.pin_href        = HREF_GPIO_NUM;
  c.pin_sscb_sda    = SIOD_GPIO_NUM;
  c.pin_sscb_scl    = SIOC_GPIO_NUM;
  c.pin_pwdn        = PWDN_GPIO_NUM;
  c.pin_reset       = RESET_GPIO_NUM;
  c.xclk_freq_hz    = 20000000;
  c.pixel_format    = PIXFORMAT_JPEG;
  c.frame_size      = FRAMESIZE_VGA;    // 640×480 — use FRAMESIZE_QVGA for faster stream
  c.jpeg_quality    = 12;               // 0=highest, 63=lowest. 10–15 is good.
  c.fb_count        = 2;
  c.grab_mode       = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&c) != ESP_OK) {
    Serial.println("[cam] init FAILED"); return;
  }
  sensor_t* s = esp_camera_sensor_get();
  s->set_whitebal(s, 1); s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1); s->set_gain_ctrl(s, 1);
  s->set_raw_gma(s, 1); s->set_lenc(s, 1);
  Serial.println("[cam] ready");
}

void initMic() {
  i2s_config_t mc = {};
  mc.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  mc.sample_rate          = MIC_RATE;
  mc.bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT;
  mc.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
  mc.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  mc.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  mc.dma_buf_count        = 4;
  mc.dma_buf_len          = 256;
  mc.use_apll             = true;

  i2s_pin_config_t mp = {MIC_SCK, MIC_WS, I2S_PIN_NO_CHANGE, MIC_SD};

  if (i2s_driver_install(MIC_I2S, &mc, 0, NULL) != ESP_OK ||
      i2s_set_pin(MIC_I2S, &mp)                 != ESP_OK) {
    Serial.println("[mic] init FAILED"); return;
  }
  i2s_zero_dma_buffer(MIC_I2S);
  Serial.println("[mic] ready");
}

void initSpeaker() {
  i2s_config_t sc = {};
  sc.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  sc.sample_rate          = SPK_RATE;
  sc.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
  sc.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
  sc.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  sc.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  sc.dma_buf_count        = 4;
  sc.dma_buf_len          = 256;
  sc.use_apll             = true;
  sc.tx_desc_auto_clear   = true;

  i2s_pin_config_t sp = {SPK_BCK, SPK_LRC, SPK_DIN, I2S_PIN_NO_CHANGE};

  if (i2s_driver_install(SPK_I2S, &sc, 0, NULL) != ESP_OK ||
      i2s_set_pin(SPK_I2S, &sp)                 != ESP_OK) {
    Serial.println("[spk] init FAILED"); return;
  }
  Serial.println("[spk] ready");
}

void initMotors() {
  pinMode(MTR_A_FWD, OUTPUT); pinMode(MTR_A_BWD, OUTPUT);
  pinMode(MTR_B_FWD, OUTPUT); pinMode(MTR_B_BWD, OUTPUT);
  stopMotors();
  Serial.println("[mtr] ready");
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.printf("\n[buddy] booting — id: %s\n", cfg.id);

  txQueue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(Frame));

  initCamera();
  initMic();
  initSpeaker();
  initMotors();

  // WiFi
  Serial.printf("[wifi] connecting to \"%s\"\n", cfg.ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.ssid, cfg.pass);
  WiFi.setAutoReconnect(true);
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
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

  // Spawn tasks on separate cores so camera/mic never block the WS loop
  xTaskCreatePinnedToCore(cameraTask, "cam", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(micTask,    "mic", 4096, NULL, 1, NULL, 1);

  Serial.println("[buddy] running");
}

// ─────────────────────────────────────────────────────────────────────────────
// Loop — drains the tx queue then maintains the WebSocket
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  // Drain outbound frames (produced by camera + mic tasks)
  Frame f;
  while (xQueueReceive(txQueue, &f, 0) == pdTRUE) {
    if (wsLive && peerOnline) ws.sendBIN(f.buf, f.len);
    free(f.buf);
  }

  ws.loop();

  // WiFi watchdog
  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 5000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }
}
