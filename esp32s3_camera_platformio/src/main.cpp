#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_wifi.h"

const char *PI_AP_SSID = "CanSat-Camera";
const char *PI_AP_PASSWORD = "cansat2026";
const char *PI_HOST = "192.168.42.1";
const uint16_t PI_PORT = 5000;

const int WIFI_RETRY_COUNT = 30;
const uint32_t WIFI_RETRY_DELAY_MS = 1000;
const uint32_t TCP_TIMEOUT_MS = 10000;
const uint32_t RECONNECT_DELAY_MS = 1000;
const uint64_t SEARCH_SLEEP_SEC = 30;

// AP探索sleepから起きた確認用LED。不要なら ENABLE_WAKE_LED を false にする。
const bool ENABLE_WAKE_LED = true;
const int WAKE_LED_PIN = LED_BUILTIN;
const uint32_t WAKE_LED_ON_MS = 150;
const uint8_t WAKE_LED_BLINK_COUNT = 3;
const bool WAKE_LED_ACTIVE_LOW = true;

// 撮影設定を変えたいときは、まずここだけ変更する。
const framesize_t CAMERA_FRAME_SIZE = FRAMESIZE_VGA;
const int JPEG_QUALITY = 10;

// Seeed Studio XIAO ESP32S3 Sense のカメラピン設定。
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39
#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

WiFiClient client;

void setupLowPowerWifi();
void printWakeupReason();
bool wokeByTimer();
void blinkWakeLed();
void setWakeLed(bool on);
bool connectToPiAp();
void sleepBeforeNextSearch();
bool connectToPiServer();
void commandLoop();
bool handleCapture();
bool initCamera();
camera_fb_t *captureJpeg();
bool sendImageSize(size_t size);
bool waitOk();
bool sendImageData(const uint8_t *data, size_t size);
bool waitComplete();
void sendReady();
void sendError(const char *code);
String readLine(uint32_t timeoutMs);

void setup() {
  Serial.begin(115200);
  pinMode(WAKE_LED_PIN, OUTPUT);
  setWakeLed(false);
  delay(1000);
  printWakeupReason();
  if (wokeByTimer()) {
    blinkWakeLed();
  }
  setupLowPowerWifi();
}

void loop() {
  // ラズパイAPがまだ無い間は、10秒探して60秒休む。
  if (WiFi.status() != WL_CONNECTED && !connectToPiAp()) {
    Serial.println("ERROR WIFI_CONNECT_FAILED");
    sleepBeforeNextSearch();
    return;
  }

  // Wi-Fi接続後はTCPだけ張り直す。Wi-Fiは維持する。
  if (!client.connected() && !connectToPiServer()) {
    Serial.println("ERROR TCP_CONNECT_FAILED");
    delay(RECONNECT_DELAY_MS);
    return;
  }

  sendReady();
  commandLoop();
}

void setupLowPowerWifi() {
  // 接続後の待機中はWi-Fiを切らず、Auto Light-sleepとModem-sleepに任せる。
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

#if CONFIG_PM_ENABLE
  esp_pm_config_esp32s3_t pmConfig = {};
  pmConfig.max_freq_mhz = 160;
  pmConfig.min_freq_mhz = 40;
  pmConfig.light_sleep_enable = true;
  esp_err_t result = esp_pm_configure(&pmConfig);
  if (result == ESP_OK) {
    Serial.println("Auto Light-sleep enabled");
  } else {
    Serial.printf("ERROR PM_CONFIG_FAILED %d\n", result);
  }
#else
  Serial.println("ERROR CONFIG_PM_ENABLE_DISABLED");
#endif
}

void printWakeupReason() {
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("Wake up by timer");
  } else {
    Serial.printf("Wake up cause: %d\n", cause);
  }
}

bool wokeByTimer() {
  return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
}

void blinkWakeLed() {
  if (!ENABLE_WAKE_LED) {
    return;
  }

  for (uint8_t i = 0; i < WAKE_LED_BLINK_COUNT; i++) {
    setWakeLed(true);
    delay(WAKE_LED_ON_MS);
    setWakeLed(false);
    delay(WAKE_LED_ON_MS);
  }
}

void setWakeLed(bool on) {
  digitalWrite(WAKE_LED_PIN, WAKE_LED_ACTIVE_LOW ? !on : on);
}

bool connectToPiAp() {
  Serial.println("Connecting to Raspberry Pi AP");
  WiFi.begin(PI_AP_SSID, PI_AP_PASSWORD);

  int count = 0;
  while (WiFi.status() != WL_CONNECTED && count < WIFI_RETRY_COUNT) {
    delay(WIFI_RETRY_DELAY_MS);
    Serial.print(".");
    count++;
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Wi-Fi status=");
    Serial.println(WiFi.status());
    return false;
  }

  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  Serial.print("Wi-Fi connected: ");
  Serial.println(WiFi.localIP());
  return true;
}

void sleepBeforeNextSearch() {
  // まだラズパイAPに接続できていない段階だけ、Wi-Fiを切って休む。
  client.stop();
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  esp_sleep_enable_timer_wakeup(SEARCH_SLEEP_SEC * 1000000ULL);
  Serial.println("Deep sleep before next AP search");
  Serial.flush();
  delay(100);
  esp_deep_sleep_start();
}

bool connectToPiServer() {
  Serial.println("Connecting to Raspberry Pi TCP server");
  client.stop();
  client.setTimeout(TCP_TIMEOUT_MS / 1000);
  return client.connect(PI_HOST, PI_PORT);
}

void commandLoop() {
  // 接続後はここでCAPTUREを待ち続ける。delay中にAuto Light-sleepへ入る想定。
  while (WiFi.status() == WL_CONNECTED && client.connected()) {
    String command = readLine(1000);
    if (command == "CAPTURE") {
      handleCapture();
      sendReady();
    }

    delay(50);
  }
}

bool handleCapture() {
  // カメラは撮影時だけ初期化し、送信後すぐ解放する。
  if (!initCamera()) {
    sendError("CAMERA_INIT_FAILED");
    esp_camera_deinit();
    return false;
  }

  camera_fb_t *fb = captureJpeg();
  if (fb == nullptr) {
    sendError("CAPTURE_FAILED");
    esp_camera_deinit();
    return false;
  }

  bool ok = sendImageSize(fb->len) && waitOk() && sendImageData(fb->buf, fb->len) && waitComplete();
  esp_camera_fb_return(fb);
  esp_camera_deinit();

  if (!ok) {
    Serial.println("ERROR IMAGE_SEND_FAILED");
  }
  return ok;
}

bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = CAMERA_FRAME_SIZE;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = JPEG_QUALITY;
  config.fb_count = 1;

  esp_camera_deinit();
  return esp_camera_init(&config) == ESP_OK;
}

camera_fb_t *captureJpeg() {
  return esp_camera_fb_get();
}

bool sendImageSize(size_t size) {
  client.printf("SIZE %u\n", static_cast<unsigned int>(size));
  return client.connected();
}

bool waitOk() {
  return readLine(TCP_TIMEOUT_MS) == "OK";
}

bool sendImageData(const uint8_t *data, size_t size) {
  // JPEG本体はSIZEで伝えたバイト数だけ、そのままTCPへ流す。
  size_t sent = 0;
  while (sent < size) {
    size_t written = client.write(data + sent, size - sent);
    if (written == 0) {
      return false;
    }
    sent += written;
  }
  return true;
}

bool waitComplete() {
  return readLine(TCP_TIMEOUT_MS) == "COMPLETE";
}

void sendReady() {
  client.print("READY\n");
}

void sendError(const char *code) {
  if (client.connected()) {
    client.printf("ERROR %s\n", code);
  }
  Serial.printf("ERROR %s\n", code);
}

String readLine(uint32_t timeoutMs) {
  // TCPは途中までしか届かないことがあるので、改行までの文字を保持する。
  static String pending;
  uint32_t startedAt = millis();

  while (millis() - startedAt < timeoutMs) {
    while (client.available()) {
      char c = client.read();
      if (c == '\n') {
        String line = pending;
        pending = "";
        line.trim();
        return line;
      }
      pending += c;
    }
    delay(10);
  }

  return "";
}
