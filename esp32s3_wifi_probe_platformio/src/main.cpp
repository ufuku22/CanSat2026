#include <Arduino.h>
#include <WiFi.h>

const char *TARGET_SSID = "CanSat-Camera";
const char *TARGET_PASSWORD = "cansat2026";

const uint32_t SCAN_INTERVAL_MS = 5000;
const uint32_t CONNECT_TIMEOUT_MS = 15000;

void scanWifi();
bool connectTargetWifi();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32S3 Wi-Fi probe start");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(500);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("CONNECTED ");
    Serial.print(WiFi.SSID());
    Serial.print(" IP=");
    Serial.print(WiFi.localIP());
    Serial.print(" RSSI=");
    Serial.println(WiFi.RSSI());
    delay(SCAN_INTERVAL_MS);
    return;
  }

  scanWifi();

  if (connectTargetWifi()) {
    Serial.println("TARGET_CONNECT_OK");
  } else {
    Serial.println("TARGET_CONNECT_FAILED");
  }

  delay(SCAN_INTERVAL_MS);
}

void scanWifi() {
  Serial.println("SCAN_START");
  int count = WiFi.scanNetworks();
  Serial.printf("SCAN_DONE count=%d\n", count);

  bool found = false;
  for (int i = 0; i < count; i++) {
    String ssid = WiFi.SSID(i);
    int32_t rssi = WiFi.RSSI(i);
    wifi_auth_mode_t auth = WiFi.encryptionType(i);

    Serial.printf("[%d] ssid=%s rssi=%d auth=%d", i, ssid.c_str(), rssi, auth);
    if (ssid == TARGET_SSID) {
      Serial.print("  <-- TARGET");
      found = true;
    }
    Serial.println();
  }

  if (!found) {
    Serial.println("TARGET_NOT_FOUND");
  }

  WiFi.scanDelete();
}

bool connectTargetWifi() {
  Serial.printf("CONNECT_START ssid=%s\n", TARGET_SSID);
  WiFi.begin(TARGET_SSID, TARGET_PASSWORD);

  uint32_t startedAt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startedAt < CONNECT_TIMEOUT_MS) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("CONNECT_STATUS=%d\n", WiFi.status());
    WiFi.disconnect(true);
    delay(500);
    return false;
  }

  Serial.print("IP=");
  Serial.println(WiFi.localIP());
  Serial.print("GATEWAY=");
  Serial.println(WiFi.gatewayIP());
  Serial.print("RSSI=");
  Serial.println(WiFi.RSSI());
  return true;
}
