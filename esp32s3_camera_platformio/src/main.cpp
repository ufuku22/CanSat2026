#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_wifi.h"

// 必要に応じてここだけ書き換える。
const char *PI_AP_SSID = "CanSat-Camera";
const char *PI_AP_PASSWORD = "cansat2026";
const char *PI_HOST = "192.168.42.1";
const uint16_t PI_PORT = 5000;

const int WIFI_RETRY_COUNT = 30;
const uint32_t WIFI_RETRY_DELAY_MS = 1000;
const uint32_t TCP_TIMEOUT_MS = 10000;
const uint32_t RECONNECT_DELAY_MS = 1000;
const uint64_t SEARCH_SLEEP_SEC = 10;

// LED点滅: 1回=sleep復帰、2回=Wi-Fi接続成功、3回=撮影送信成功、速い8回=エラー。
const bool ENABLE_LED_STATUS = true;
const int LED_PIN = LED_BUILTIN;
const bool LED_ACTIVE_LOW = true;
const uint32_t LED_ON_MS = 150;

// 撮影設定。
const framesize_t CAMERA_FRAME_SIZE = FRAMESIZE_VGA;
const int JPEG_QUALITY = 10;

// Seeed Studio XIAO ESP32S3 Sense のカメラピン。
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
bool connectToPiAp();
bool connectToPiServer();
void sleepBeforeNextSearch();
void commandLoop();
bool handleCapture();
bool initCamera();
String readLine(uint32_t timeoutMs);
void sendError(const char *code);
void blinkStatus(uint8_t count);
void blinkError();
void setLed(bool on);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  setLed(false);
  delay(1000);

  printWakeupReason();
  setupLowPowerWifi();
}

void loop() {
  // ラズパイAPが見つからない間は、一定時間sleepしてから再探索する。
  if (WiFi.status() != WL_CONNECTED && !connectToPiAp()) {
    Serial.println("ERROR WIFI_CONNECT_FAILED");
    sleepBeforeNextSearch();
    return;
  }

  // AP接続後、ラズパイ側のTCPサーバへ接続する。
  if (!client.connected() && !connectToPiServer()) {
    Serial.println("ERROR TCP_CONNECT_FAILED");
    delay(RECONNECT_DELAY_MS);
    return;
  }

  client.print("READY\n");
  commandLoop();
}

void setupLowPowerWifi() {
  // 接続後の待機中はAuto Light-sleepとModem-sleepに任せる。
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

#if CONFIG_PM_ENABLE
  esp_pm_config_esp32s3_t pmConfig = {};
  pmConfig.max_freq_mhz = 160;
  pmConfig.min_freq_mhz = 40;
  pmConfig.light_sleep_enable = true;
  esp_pm_configure(&pmConfig);
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

bool connectToPiAp() {
  Serial.println("Connecting to Raspberry Pi AP");
  WiFi.begin(PI_AP_SSID, PI_AP_PASSWORD);

  for (int i = 0; WiFi.status() != WL_CONNECTED && i < WIFI_RETRY_COUNT; i++) {
    delay(WIFI_RETRY_DELAY_MS);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("Wi-Fi status=%d\n", WiFi.status());
    return false;
  }

  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  Serial.print("Wi-Fi connected: ");
  Serial.println(WiFi.localIP());
  blinkStatus(2);
  return true;
}

bool connectToPiServer() {
  Serial.println("Connecting to Raspberry Pi TCP server");
  client.stop();
  client.setTimeout(TCP_TIMEOUT_MS / 1000);
  return client.connect(PI_HOST, PI_PORT);
}

void sleepBeforeNextSearch() {
  // APが見つからない時だけWi-Fiを切ってlight sleepする。復帰できたらLEDを1回点滅する。
  client.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_sleep_enable_timer_wakeup(SEARCH_SLEEP_SEC * 1000000ULL);
  Serial.println("Sleep before next AP search");
  Serial.flush();
  delay(100);

  esp_light_sleep_start();

  delay(500);
  blinkStatus(1);
  Serial.println("Wake from AP search sleep");
  setupLowPowerWifi();
}

void commandLoop() {
  // PiからCAPTUREが来たら1枚撮影して送信する。
  while (WiFi.status() == WL_CONNECTED && client.connected()) {
    String command = readLine(1000);
    if (command == "CAPTURE") {
      handleCapture();
      client.print("READY\n");
    }
    delay(50);
  }
}

bool handleCapture() {
  if (!initCamera()) {
    blinkError();
    sendError("CAMERA_INIT_FAILED");
    esp_camera_deinit();
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    blinkError();
    sendError("CAPTURE_FAILED");
    esp_camera_deinit();
    return false;
  }

  client.printf("SIZE %u\n", static_cast<unsigned int>(fb->len));
  bool ok = readLine(TCP_TIMEOUT_MS) == "OK";

  if (ok) {
    size_t sent = 0;
    while (sent < fb->len) {
      size_t written = client.write(fb->buf + sent, fb->len - sent);
      if (written == 0) {
        ok = false;
        break;
      }
      sent += written;
    }
  }

  ok = ok && readLine(TCP_TIMEOUT_MS) == "COMPLETE";
  esp_camera_fb_return(fb);
  esp_camera_deinit();

  if (ok) {
    blinkStatus(3);
  } else {
    blinkError();
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

String readLine(uint32_t timeoutMs) {
  // TCPは分割されて届くので、改行まで文字をためて読む。
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

void sendError(const char *code) {
  if (client.connected()) {
    client.printf("ERROR %s\n", code);
  }
  Serial.printf("ERROR %s\n", code);
}

void blinkStatus(uint8_t count) {
  if (!ENABLE_LED_STATUS) {
    return;
  }

  for (uint8_t i = 0; i < count; i++) {
    setLed(true);
    delay(LED_ON_MS);
    setLed(false);
    delay(LED_ON_MS);
  }
  delay(300);
}

void blinkError() {
  if (!ENABLE_LED_STATUS) {
    return;
  }

  for (uint8_t i = 0; i < 8; i++) {
    setLed(true);
    delay(70);
    setLed(false);
    delay(70);
  }
  delay(300);
}

void setLed(bool on) {
  digitalWrite(LED_PIN, LED_ACTIVE_LOW ? !on : on);
}
