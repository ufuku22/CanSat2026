#include <Arduino.h>
#include <WiFi.h>

const char *TARGET_SSID = "CanSat-Camera";
const char *TARGET_PASSWORD = "cansat2026";

const int CONNECT_RETRY_COUNT = 30;
const uint32_t CONNECT_RETRY_DELAY_MS = 1000;
const uint32_t LOOP_DELAY_MS = 5000;

void connectTargetWifi();
void printConnectionStatus();
void scanWifiAfterFailure();

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("ESP32S3 Wi-Fi simple connection test");
  Serial.print("Connecting to SSID: ");
  Serial.println(TARGET_SSID);

  WiFi.mode(WIFI_STA);
  connectTargetWifi();
}

void loop() {
  delay(LOOP_DELAY_MS);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Still connected. IP: ");
    Serial.print(WiFi.localIP());
    Serial.print(" RSSI: ");
    Serial.println(WiFi.RSSI());
  } else {
    Serial.print("Disconnected. status=");
    Serial.println(WiFi.status());
    connectTargetWifi();
  }
}

void connectTargetWifi() {
  WiFi.begin(TARGET_SSID, TARGET_PASSWORD);

  int count = 0;
  while (WiFi.status() != WL_CONNECTED && count < CONNECT_RETRY_COUNT) {
    delay(CONNECT_RETRY_DELAY_MS);
    Serial.print(".");
    count++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Wi-Fi connected!");
    printConnectionStatus();
  } else {
    Serial.println("Wi-Fi connection failed.");
    Serial.print("status=");
    Serial.println(WiFi.status());
    Serial.println("Scanning after failure...");
    WiFi.disconnect(false);
    delay(1000);
    scanWifiAfterFailure();
  }
}

void printConnectionStatus() {
  Serial.print("ESP32S3 IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("Gateway IP: ");
  Serial.println(WiFi.gatewayIP());

  Serial.print("Signal strength RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

void scanWifiAfterFailure() {
  int count = WiFi.scanNetworks();
  Serial.printf("SCAN_DONE count=%d\n", count);

  for (int i = 0; i < count; i++) {
    String ssid = WiFi.SSID(i);
    Serial.printf(
      "[%d] ssid=%s rssi=%d auth=%d",
      i,
      ssid.c_str(),
      WiFi.RSSI(i),
      WiFi.encryptionType(i)
    );
    if (ssid == TARGET_SSID) {
      Serial.print("  <-- TARGET");
    }
    Serial.println();
  }
}
