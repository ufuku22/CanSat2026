/*
  ESP32-C3 USB <-> TLM922S UARTブリッジ

  Arduino IDEのシリアルモニタ、またはPCのターミナルからESP32-C3の
  USBシリアルを開きます。TLM922Sコマンドを入力してEnterを押します。

  TLM922Sのコマンド終端はCRなので、このブリッジがEnterをCRに変換します。
*/

#include <Arduino.h>

// ESP32-C3ボードの配線に合わせて変更してください。
// ESP32-C3 TX_PIN -> TLM922S RXD
// ESP32-C3 RX_PIN <- TLM922S TXD
static const int TLM_RX_PIN = 4;
static const int TLM_TX_PIN = 5;

static const uint32_t PC_BAUD = 115200;
static const uint32_t TLM_BAUD = 115200;

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
