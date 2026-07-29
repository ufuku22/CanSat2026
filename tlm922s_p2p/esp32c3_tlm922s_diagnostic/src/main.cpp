/*
  ESP32-C3 用 TLM922S 診断・P2P設定プログラム。

  TLM922S との UART 応答を確認し、P2P設定が期待値と違う場合は修正して保存する。
  受信処理は esp32c3_ground_station_receiver 側に分離している。
*/

#include <Arduino.h>

#ifndef TLM_RX_PIN
#define TLM_RX_PIN 20
#endif

#ifndef TLM_TX_PIN
#define TLM_TX_PIN 21
#endif

#ifndef TLM_BAUD
#define TLM_BAUD 115200
#endif

static const uint32_t PC_BAUD = 115200;
static const uint32_t STATUS_INTERVAL_MS = 5000;
static const uint32_t UART_CHECK_INTERVAL_MS = 10000;

HardwareSerial TlmSerial(1);
bool uartOk = false;
bool p2pConfigured = false;
bool radioSettingsChanged = false;
uint32_t lastStatusAt = 0;
uint32_t lastUartCheckAt = 0;

struct P2pSetting {
  const char* label;
  const char* getCommand;
  const char* setCommand;
  const char* expected;
};

static const P2pSetting P2P_SETTINGS[] = {
  {"freq", "p2p get_freq", "p2p set_freq 922500000", "922500000"},
  {"pwr", "p2p get_pwr", "p2p set_pwr 20", "20"},
  {"sf", "p2p get_sf", "p2p set_sf 12", "12"},
  {"bw", "p2p get_bw", "p2p set_bw 125", "125"},
  {"cr", "p2p get_cr", "p2p set_cr 4/6", "4/6"},
  {"prlen", "p2p get_prlen", "p2p set_prlen 16", "16"},
  {"crc", "p2p get_crc", "p2p set_crc on", "on"},
  {"iqi", "p2p get_iqi", "p2p set_iqi off", "off"},
  {"sync", "p2p get_sync", "p2p set_sync 12", "12"},
};

bool checkAndConfigureRadio();
bool checkUartAlive();
bool ensureP2pSetting(const P2pSetting& setting, bool& changed);
bool sendRadioCommand(const String& command, uint32_t timeoutMs, String& response);
String firstRadioValue(const String& response);
bool responseHasOk(const String& response);
void printRadioResponse(const String& response);
void printRadioStatus();

void setup() {
  Serial.begin(PC_BAUD);

  // USB シリアルが開かれるまで少し待つ。単体電源でも止まらないよう最大 5 秒。
  uint32_t startedAt = millis();
  while (!Serial && millis() - startedAt < 5000) {
    delay(10);
  }

  TlmSerial.begin(TLM_BAUD, SERIAL_8N1, TLM_RX_PIN, TLM_TX_PIN);

  Serial.println();
  Serial.println("ESP32-C3 TLM922S diagnostic");
  uartOk = checkAndConfigureRadio();
  p2pConfigured = uartOk;
  lastUartCheckAt = millis();
  printRadioStatus();
}

void loop() {
  uint32_t now = millis();

  if (!p2pConfigured && now - lastUartCheckAt >= UART_CHECK_INTERVAL_MS) {
    lastUartCheckAt = now;
    Serial.println("Retrying TLM922S UART and P2P settings check...");
    uartOk = checkAndConfigureRadio();
    p2pConfigured = uartOk;
    printRadioStatus();
    return;
  }

  if (p2pConfigured && now - lastUartCheckAt >= UART_CHECK_INTERVAL_MS) {
    lastUartCheckAt = now;
    uartOk = checkUartAlive();
    printRadioStatus();
    return;
  }

  if (now - lastStatusAt >= STATUS_INTERVAL_MS) {
    printRadioStatus();
  }
}

bool checkAndConfigureRadio() {
  Serial.println("Checking TLM922S UART and P2P settings...");

  bool changed = false;
  for (const P2pSetting& setting : P2P_SETTINGS) {
    if (!ensureP2pSetting(setting, changed)) {
      Serial.println("TLM922S startup check failed.");
      radioSettingsChanged = changed;
      return false;
    }
  }

  radioSettingsChanged = changed;
  if (changed) {
    String response;
    if (!sendRadioCommand("p2p save", 1000, response) || !responseHasOk(response)) {
      Serial.println("WARNING: p2p save was not accepted.");
      return false;
    }
    Serial.println("P2P settings saved to flash.");
  } else {
    Serial.println("P2P settings already match expected values.");
  }

  return true;
}

bool checkUartAlive() {
  String response;
  if (!sendRadioCommand("mod get_ver", 1000, response)) {
    Serial.println("ERROR: no response for mod get_ver");
    return false;
  }
  return true;
}

bool ensureP2pSetting(const P2pSetting& setting, bool& changed) {
  String response;
  if (!sendRadioCommand(setting.getCommand, 1000, response)) {
    Serial.print("ERROR: no response for ");
    Serial.println(setting.getCommand);
    return false;
  }

  String value = firstRadioValue(response);
  if (value.length() == 0) {
    Serial.print("ERROR: could not read ");
    Serial.print(setting.label);
    Serial.println(" setting.");
    return false;
  }

  Serial.print("P2P ");
  Serial.print(setting.label);
  Serial.print("=");
  Serial.println(value);

  if (value.equalsIgnoreCase(setting.expected)) {
    return true;
  }

  Serial.print("P2P ");
  Serial.print(setting.label);
  Serial.print(" is not expected value ");
  Serial.print(setting.expected);
  Serial.println(". Updating...");

  if (!sendRadioCommand(setting.setCommand, 1000, response) || !responseHasOk(response)) {
    Serial.print("ERROR: setting command failed: ");
    Serial.println(setting.setCommand);
    return false;
  }

  changed = true;
  return true;
}

bool sendRadioCommand(const String& command, uint32_t timeoutMs, String& response) {
  while (TlmSerial.available() > 0) {
    TlmSerial.read();
  }

  response = "";
  Serial.print("> ");
  Serial.println(command);
  TlmSerial.print(command);
  TlmSerial.print('\r');

  uint32_t startedAt = millis();
  uint32_t lastReceivedAt = startedAt;
  while (millis() - startedAt < timeoutMs) {
    while (TlmSerial.available() > 0) {
      char c = static_cast<char>(TlmSerial.read());
      response += c;
      lastReceivedAt = millis();
    }
    if (response.length() > 0 && millis() - lastReceivedAt > 100) {
      break;
    }
    delay(5);
  }

  printRadioResponse(response);
  return response.length() > 0;
}

String firstRadioValue(const String& response) {
  String normalized = response;
  normalized.replace('\r', '\n');

  int start = 0;
  while (start < normalized.length()) {
    int end = normalized.indexOf('\n', start);
    if (end < 0) {
      end = normalized.length();
    }

    String line = normalized.substring(start, end);
    line.trim();
    if (line.startsWith(">> ")) {
      String value = line.substring(3);
      value.trim();
      if (!value.equalsIgnoreCase("Ok")) {
        return value;
      }
    }

    start = end + 1;
  }
  return "";
}

bool responseHasOk(const String& response) {
  return response.indexOf(">> Ok") >= 0;
}

void printRadioResponse(const String& response) {
  String normalized = response;
  normalized.replace('\r', '\n');

  int start = 0;
  while (start < normalized.length()) {
    int end = normalized.indexOf('\n', start);
    if (end < 0) {
      end = normalized.length();
    }

    String line = normalized.substring(start, end);
    line.trim();
    if (line.length() > 0) {
      Serial.print("< ");
      Serial.println(line);
    }

    start = end + 1;
  }
}

void printRadioStatus() {
  lastStatusAt = millis();
  Serial.print("Radio status: uart=");
  Serial.print(uartOk ? "ok" : "failed");
  Serial.print(" p2p=");
  Serial.print(p2pConfigured ? "configured" : "unknown");
  Serial.print(" saved=");
  Serial.print(radioSettingsChanged ? "yes" : "not_needed");
  Serial.println(" freq=922500000 pwr=20 sf=12 bw=125 cr=4/6 prlen=16 crc=on iqi=off sync=12");
}
