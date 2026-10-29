/*
  ESP32-C3 USB <-> TLM922S UART bridge for PlatformIO.

  Open this folder with VS Code + PlatformIO. After upload, run
  usb_bridge_monitor.py and type a TLM922S command followed by Enter.
  The bridge forwards PC-side line endings as CR to the TLM922S UART.
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

  uint32_t startedAt = millis();
  while (!Serial && millis() - startedAt < 5000) {
    delay(10);
  }

  TlmSerial.begin(TLM_BAUD, SERIAL_8N1, TLM_RX_PIN, TLM_TX_PIN);

  Serial.println();
  Serial.println("ESP32-C3 USB <-> TLM922S UART bridge");
  Serial.println("Type a TLM922S command, then press Enter.");
  Serial.println("Example: mod get_ver");
}

void loop() {
  // PC -> TLM922S. Treat CR/LF from the monitor as command terminators.
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == '\r' || c == '\n') {
      sendLineToTlm(pcLine);
      pcLine = "";
    } else if (isPrintable(c)) {
      pcLine += c;
    }
  }

  // TLM922S -> PC. Forward replies without modification.
  while (TlmSerial.available() > 0) {
    Serial.write(TlmSerial.read());
  }
}
