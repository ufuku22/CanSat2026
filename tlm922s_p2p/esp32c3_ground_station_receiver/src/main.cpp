/*
  ESP32-C3 用 TLM922S 地上局受信プログラム。

  ESP32-C3 は TLM922S を受信待ちにして、Raspberry Pi から届いた
  JSON パケットを USB シリアルモニタへ表示する。
*/

#include <Arduino.h>

#ifndef TLM_RX_PIN
#define TLM_RX_PIN 4
#endif

#ifndef TLM_TX_PIN
#define TLM_TX_PIN 5
#endif

#ifndef TLM_BAUD
#define TLM_BAUD 115200
#endif

static const uint32_t PC_BAUD = 115200;
static const uint32_t P2P_CONFIG_RETRY_INTERVAL_MS = 10000;

HardwareSerial TlmSerial(1);
String tlmLine;
bool p2pConfigured = false;
uint32_t lastP2pConfigAttemptAt = 0;

struct P2pSetting {
  const char* label;
  const char* getCommand;
  const char* setCommand;
  const char* expected;
};

// Raspberry Pi側の raspberry_pi_zero_wh/p2p_config.py と同じ設定。
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
bool ensureP2pSetting(const P2pSetting& setting, bool& changed);
bool sendRadioCommand(const String& command, uint32_t timeoutMs, String& response);
String firstRadioValue(const String& response);
bool responseHasOk(const String& response);
void printRadioResponse(const String& response);
void startReceive();
void handleTlmLine(const String& line);
bool parseRadioRx(const String& line, String& payloadHex, String& rssi, String& snr);
bool isHexText(const String& value);
bool isImagePacketHex(const String& payloadHex);
String hexToText(const String& hex);
String jsonStringField(const String& json, const String& key);
String jsonNumberField(const String& json, const String& key);
bool isIntegerText(const String& value);
void printPacket(const String& payloadHex, const String& rssi, const String& snr);
void printGps(const String& payload);

void setup() {
  Serial.begin(PC_BAUD);

  // USB シリアルが開かれるまで少し待つ。単体電源でも止まらないよう最大 5 秒。
  uint32_t startedAt = millis();
  while (!Serial && millis() - startedAt < 5000) {
    delay(10);
  }

  // TLM922S とは別 UART で通信する。ピン番号は platformio.ini から変更できる。
  TlmSerial.begin(TLM_BAUD, SERIAL_8N1, TLM_RX_PIN, TLM_TX_PIN);

  Serial.println();
  Serial.println("ESP32-C3 TLM922S ground station receiver");
  p2pConfigured = checkAndConfigureRadio();
  lastP2pConfigAttemptAt = millis();
  if (p2pConfigured) {
    Serial.println("Waiting for packets from Raspberry Pi...");
    startReceive();
  } else {
    Serial.println("P2P configuration failed. Waiting before retry...");
  }
}

void loop() {
  if (!p2pConfigured) {
    uint32_t now = millis();
    if (now - lastP2pConfigAttemptAt >= P2P_CONFIG_RETRY_INTERVAL_MS) {
      lastP2pConfigAttemptAt = now;
      Serial.println("Retrying TLM922S P2P configuration...");
      p2pConfigured = checkAndConfigureRadio();
      if (p2pConfigured) {
        Serial.println("Waiting for packets from Raspberry Pi...");
        startReceive();
      }
    }
    return;
  }

  // TLM922S の応答は行単位で来るため、改行までためてから処理する。
  while (TlmSerial.available() > 0) {
    char c = static_cast<char>(TlmSerial.read());
    if (c == '\r' || c == '\n') {
      if (tlmLine.length() > 0) {
        handleTlmLine(tlmLine);
        tlmLine = "";
      }
    } else {
      tlmLine += c;
    }
  }
}

bool checkAndConfigureRadio() {
  Serial.println("Checking TLM922S P2P settings...");

  bool changed = false;
  for (const P2pSetting& setting : P2P_SETTINGS) {
    if (!ensureP2pSetting(setting, changed)) {
      Serial.println("TLM922S P2P configuration failed.");
      return false;
    }
  }

  if (changed) {
    String response;
    if (
      !sendRadioCommand("p2p save", 1000, response)
      || !responseHasOk(response)
    ) {
      Serial.println("ERROR: p2p save was not accepted.");
      return false;
    }
    Serial.println("P2P settings updated and saved to flash.");
  } else {
    Serial.println("P2P settings already match the Raspberry Pi sender.");
  }

  Serial.println(
    "P2P ready: freq=922500000 pwr=20 sf=12 bw=125 "
    "cr=4/6 prlen=16 crc=on iqi=off sync=12"
  );
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
    Serial.print("ERROR: could not read P2P ");
    Serial.print(setting.label);
    Serial.println(".");
    return false;
  }

  Serial.print("P2P ");
  Serial.print(setting.label);
  Serial.print("=");
  Serial.println(value);
  if (value.equalsIgnoreCase(setting.expected)) {
    return true;
  }

  Serial.print("Updating P2P ");
  Serial.print(setting.label);
  Serial.print(" to ");
  Serial.println(setting.expected);
  if (
    !sendRadioCommand(setting.setCommand, 1000, response)
    || !responseHasOk(response)
  ) {
    Serial.print("ERROR: setting command failed: ");
    Serial.println(setting.setCommand);
    return false;
  }

  changed = true;
  return true;
}

bool sendRadioCommand(
  const String& command,
  uint32_t timeoutMs,
  String& response
) {
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

void startReceive() {
  // 0 は無期限受信。1 パケット受けると待ち状態が終わるため、受信後にもう一度呼ぶ。
  TlmSerial.print("p2p rx 0\r");
  Serial.println("> p2p rx 0");
}

void handleTlmLine(const String& line) {
  // 生の応答も残しておくと、配線や設定ミスの切り分けがしやすい。
  Serial.print("< ");
  Serial.println(line);

  String payloadHex;
  String rssi;
  String snr;
  if (parseRadioRx(line, payloadHex, rssi, snr)) {
    printPacket(payloadHex, rssi, snr);
    delay(50);
    startReceive();
    return;
  }

  // エラーが返ったときは少し待ってから受信待ちに戻す。
  if (line.indexOf("radio_err") >= 0) {
    delay(300);
    startReceive();
  }
}

bool parseRadioRx(const String& line, String& payloadHex, String& rssi, String& snr) {
  // 代表的な受信行:
  //   >> radio_rx <payload_hex> <rssi> <snr>
  // 一部のファームウェアでは payload の前に追加の数値が入ることがある。
  if (!line.startsWith(">> radio_rx ")) {
    return false;
  }

  String parts[6];
  int count = 0;
  int start = 0;
  for (int i = 0; i <= line.length() && count < 6; i++) {
    if (i == line.length() || line.charAt(i) == ' ') {
      if (i > start) {
        parts[count++] = line.substring(start, i);
      }
      start = i + 1;
    }
  }

  int payloadIndex = -1;
  if (count >= 5 && isHexText(parts[2]) && isIntegerText(parts[3]) && isIntegerText(parts[4])) {
    payloadIndex = 2;
  } else if (count >= 6 && isHexText(parts[3]) && isIntegerText(parts[4]) && isIntegerText(parts[5])) {
    payloadIndex = 3;
  }

  if (payloadIndex < 0) {
    return false;
  }

  payloadHex = parts[payloadIndex];
  rssi = parts[payloadIndex + 1];
  snr = parts[payloadIndex + 2];
  return true;
}

bool isHexText(const String& value) {
  if (value.length() == 0 || (value.length() % 2) != 0) {
    return false;
  }
  for (int i = 0; i < value.length(); i++) {
    if (!isxdigit(value.charAt(i))) {
      return false;
    }
  }
  return true;
}

bool isImagePacketHex(const String& payloadHex) {
  // 画像パケットは先頭が "CI" + version(1) + type("I")。
  // 16進文字列では 43 49 01 49 で始まる。
  return payloadHex.length() >= 8 && payloadHex.substring(0, 8).equalsIgnoreCase("43490149");
}

String hexToText(const String& hex) {
  // Python 側は JSON を UTF-8 bytes -> 16進文字列にして送っている。
  String text;
  for (int i = 0; i + 1 < hex.length(); i += 2) {
    char buf[3] = {hex.charAt(i), hex.charAt(i + 1), '\0'};
    char c = static_cast<char>(strtoul(buf, nullptr, 16));
    text += c;
  }
  return text;
}

String jsonStringField(const String& json, const String& key) {
  // ArduinoJson を追加せずに済ませるため、表示に必要な浅い項目だけ読む。
  String pattern = "\"" + key + "\":\"";
  int start = json.indexOf(pattern);
  if (start < 0) {
    return "";
  }
  start += pattern.length();
  int end = json.indexOf('"', start);
  if (end < 0) {
    return "";
  }
  return json.substring(start, end);
}

String jsonNumberField(const String& json, const String& key) {
  // 緯度・経度などの数値だけを、文字列として取り出して表示する。
  String pattern = "\"" + key + "\":";
  int start = json.indexOf(pattern);
  if (start < 0) {
    return "";
  }
  start += pattern.length();
  int end = start;
  while (end < json.length()) {
    char c = json.charAt(end);
    if (!(isDigit(c) || c == '-' || c == '.')) {
      break;
    }
    end++;
  }
  return json.substring(start, end);
}

bool isIntegerText(const String& value) {
  // RSSI/SNR の列を見分けるための簡単な整数チェック。
  if (value.length() == 0) {
    return false;
  }
  int start = value.charAt(0) == '-' ? 1 : 0;
  if (start == value.length()) {
    return false;
  }
  for (int i = start; i < value.length(); i++) {
    if (!isDigit(value.charAt(i))) {
      return false;
    }
  }
  return true;
}

void printPacket(const String& payloadHex, const String& rssi, const String& snr) {
  if (isImagePacketHex(payloadHex)) {
    // PC側の自律受信スクリプトがこの1行を拾って、FEC復元とJPEG保存を行う。
    Serial.print("IMG_PACKET ");
    Serial.print(payloadHex);
    Serial.print(" RSSI=");
    Serial.print(rssi);
    Serial.print(" SNR=");
    Serial.println(snr);
    Serial.println();
    return;
  }

  String payload = hexToText(payloadHex);
  String type = jsonStringField(payload, "type");
  String seq = jsonNumberField(payload, "seq");
  String time = jsonStringField(payload, "time");

  Serial.println();
  Serial.print("RX type=");
  Serial.print(type.length() > 0 ? type : "unknown");
  Serial.print(" seq=");
  Serial.print(seq.length() > 0 ? seq : "?");
  Serial.print(" time=");
  Serial.print(time);
  Serial.print(" RSSI=");
  Serial.print(rssi);
  Serial.print(" SNR=");
  Serial.println(snr);
  Serial.print("JSON ");
  Serial.println(payload);
  printGps(payload);
  Serial.println();
}

void printGps(const String& payload) {
  // GNSS が入っているパケットだけ、人が見やすい 1 行を追加で出す。
  String lat = jsonNumberField(payload, "lat");
  String lon = jsonNumberField(payload, "lon");
  if (lat.length() == 0 || lon.length() == 0) {
    return;
  }

  Serial.print("GPS lat=");
  Serial.print(lat);
  Serial.print(" lon=");
  Serial.print(lon);
  Serial.print(" alt=");
  Serial.print(jsonNumberField(payload, "alt"));
  Serial.print(" sat=");
  Serial.print(jsonNumberField(payload, "sat"));
  Serial.print(" fix=");
  Serial.println(jsonNumberField(payload, "fix"));
}
