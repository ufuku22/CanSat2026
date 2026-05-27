/*
  ESP32-C3 autonomous ground station receiver for TLM922S P2P.

  The ESP32-C3 keeps the TLM922S in receive mode and prints received JSON
  packets to the USB serial monitor. Raspberry Pi sends one-way packets with
  CommunicationManager.
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

HardwareSerial TlmSerial(1);
String tlmLine;

void startReceive();
void handleTlmLine(const String& line);
bool parseRadioRx(const String& line, String& payloadHex, String& rssi, String& snr);
String hexToText(const String& hex);
String jsonStringField(const String& json, const String& key);
String jsonNumberField(const String& json, const String& key);
bool isIntegerText(const String& value);
void printPacket(const String& payloadHex, const String& rssi, const String& snr);

void setup() {
  Serial.begin(PC_BAUD);

  uint32_t startedAt = millis();
  while (!Serial && millis() - startedAt < 5000) {
    delay(10);
  }

  TlmSerial.begin(TLM_BAUD, SERIAL_8N1, TLM_RX_PIN, TLM_TX_PIN);

  Serial.println();
  Serial.println("ESP32-C3 TLM922S ground station receiver");
  Serial.println("Waiting for one-way packets from Raspberry Pi...");
  startReceive();
}

void loop() {
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

void startReceive() {
  TlmSerial.print("p2p rx 0\r");
  Serial.println("> p2p rx 0");
}

void handleTlmLine(const String& line) {
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

  if (line.indexOf("radio_err") >= 0) {
    delay(300);
    startReceive();
  }
}

bool parseRadioRx(const String& line, String& payloadHex, String& rssi, String& snr) {
  if (!line.startsWith(">> radio_rx ")) {
    return false;
  }

  String parts[8];
  int count = 0;
  int start = 0;
  for (int i = 0; i <= line.length() && count < 8; i++) {
    if (i == line.length() || line.charAt(i) == ' ') {
      if (i > start) {
        parts[count++] = line.substring(start, i);
      }
      start = i + 1;
    }
  }

  int bestIndex = -1;
  for (int i = 0; i + 2 < count; i++) {
    if (parts[i].length() == 0 || (parts[i].length() % 2) != 0) {
      continue;
    }

    bool hex = true;
    for (int j = 0; j < parts[i].length(); j++) {
      if (!isxdigit(parts[i].charAt(j))) {
        hex = false;
        break;
      }
    }
    if (!hex) {
      continue;
    }

    if (isIntegerText(parts[i + 1]) && isIntegerText(parts[i + 2])) {
      if (bestIndex < 0 || parts[i].length() > parts[bestIndex].length()) {
        bestIndex = i;
      }
    }
  }

  if (bestIndex < 0) {
    return false;
  }

  payloadHex = parts[bestIndex];
  rssi = parts[bestIndex + 1];
  snr = parts[bestIndex + 2];
  return true;
}

String hexToText(const String& hex) {
  String text;
  for (int i = 0; i + 1 < hex.length(); i += 2) {
    char buf[3] = {hex.charAt(i), hex.charAt(i + 1), '\0'};
    char c = static_cast<char>(strtoul(buf, nullptr, 16));
    text += c;
  }
  return text;
}

String jsonStringField(const String& json, const String& key) {
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
  Serial.println();
}
