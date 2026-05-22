# TLM922S P2P通信テスト

2台のTLM922SでP2P通信を確認するための最小構成です。

```text
Raspberry Pi Zero WH -> TLM922S-A  ~~ LoRa P2P ~~  TLM922S-B -> ESP32-C3 -> PC
```

まずは片方向通信から始めます。

1. ESP32-C3側を受信待ちにする
2. Raspberry Pi側から送信する

これが通れば、送受信を入れ替えたり、双方向通信へ進めます。

## フォルダ構成

```text
tlm922s_p2p/
  raspberry_pi_zero_wh/
    tlm922s_uart.py  shared UART helper
    p2p_config.py    set/check P2P parameters
    p2p_tx.py        send test packets
    p2p_rx.py        receive test packets
  esp32c3_usb_bridge/
    esp32c3_usb_bridge.ino  PC USB serial to TLM922S UART bridge
```

## 配線

### Raspberry Pi Zero WH to TLM922S-A

```text
Pi GPIO14 TXD, pin 8   -> TLM922S RXD
Pi GPIO15 RXD, pin 10  -> TLM922S TXD
Pi GND                 -> TLM922S GND
```

### ESP32-C3 to TLM922S-B

スケッチの初期設定では以下のピンを使います。

```text
ESP32-C3 GPIO5 TX      -> TLM922S RXD
ESP32-C3 GPIO4 RX      -> TLM922S TXD
ESP32-C3 GND           -> TLM922S GND
```

ESP32-C3ボードによって使いやすいピンが違うので、必要なら
`esp32c3_usb_bridge.ino` のここを変更してください。

```cpp
static const int TLM_RX_PIN = 4;
static const int TLM_TX_PIN = 5;
```

UARTは3.3Vロジックです。RS-232レベルを直接つながないでください。

## ESP32-C3側の準備

1. Arduino IDEで `esp32c3_usb_bridge/esp32c3_usb_bridge.ino` を開く
2. 使用するESP32-C3ボードを選択する
3. スケッチを書き込む
4. シリアルモニタを `115200` で開く
5. 改行コードは `Newline` または `Both NL & CR` にする
6. TLM922Sの応答を確認する

```text
mod get_ver
```

`>> ...` のような応答が返れば、ESP32-C3経由のUARTは動いています。

## 2台のTLM922Sを同じP2P設定にする

Raspberry Pi側で実行します。

```bash
cd tlm922s_p2p/raspberry_pi_zero_wh
python3 p2p_config.py
```

ESP32-C3側では、PCのシリアルモニタから同じ設定コマンドを打ちます。

```text
p2p set_freq 922500000
p2p set_pwr 14
p2p set_sf 7
p2p set_bw 125
p2p set_cr 4/6
p2p set_prlen 12
p2p set_crc on
p2p set_iqi off
p2p set_sync 12
```

設定をTLM922S内に保存したい場合は、それぞれのモジュールで実行します。

```text
p2p save
```

## 片方向テスト: Piから送信、PC側で受信

先にESP32-C3側のシリアルモニタで受信待ちにします。

```text
p2p rx 10000
```

10秒以内にRaspberry Pi側で送信します。

```bash
cd tlm922s_p2p/raspberry_pi_zero_wh
python3 p2p_tx.py
```

送信側で期待する応答:

```text
>> Ok
>> radio_tx_ok
```

受信側で期待する応答:

```text
>> Ok
>> radio_rx 48656c6c6f... -90 -50
```

受信データは16進文字列です。例: `48656c6c6f` は `Hello` です。

## 逆向きテスト: Pi側で受信

Raspberry Pi側を受信にする場合:

```bash
cd tlm922s_p2p/raspberry_pi_zero_wh
python3 p2p_rx.py
```

ESP32-C3側のシリアルモニタから送信します。

```text
p2p tx 48656c6c6f
```

## 注意

- TLM922SのコマンドはASCII文字列で、終端はCRです。
- ESP32-C3のブリッジは、PC側のEnterをCRに変換してTLM922Sへ送ります。
- P2P設定は2台で完全に同じにしてください。
- 日本国内で電波を出す場合は、周波数、出力、アンテナ、送信時間が試験条件として適切か確認してください。
