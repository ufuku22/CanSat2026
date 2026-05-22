/*
  ESP32-C3 USB <-> TLM922S UARTブリッジ for PlatformIO

  VS Code + PlatformIOでこのフォルダを開き、Upload後にMonitorを開きます。
  PC側でTLM922Sコマンドを入力してEnterを押すと、ESP32-C3がCR終端に
  変換してTLM922Sへ送ります。
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
String pcLine;

void sendLineToTlm(const String& line) {
  if (line.length() == 0) {
    return;
  }

  Serial.print("> ");
  Serial.println(line);

  TlmSerial.print(line);
  TlmSerial.print('\r');
}

void setup() {
  Serial.begin(PC_BAUD);
  delay(500);

  TlmSerial.begin(TLM_BAUD, SERIAL_8N1, TLM_RX_PIN, TLM_TX_PIN);

  Serial.println();
  Serial.println("ESP32-C3 USB <-> TLM922S UART bridge");
  Serial.println("PlatformIO version");
  Serial.println("Type a TLM922S command, then press Enter.");
  Serial.println("Example: mod get_ver");
}

void loop() {
  // PC -> TLM922S。PC側のCR/LFを1コマンドの終わりとして扱う。
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == '\r' || c == '\n') {
      sendLineToTlm(pcLine);
      pcLine = "";
    } else if (isPrintable(c)) {
      pcLine += c;
    }
  }

  // TLM922S -> PC。無線機から返った文字をそのままPCへ流す。
  while (TlmSerial.available() > 0) {
    Serial.write(TlmSerial.read());
  }
}
