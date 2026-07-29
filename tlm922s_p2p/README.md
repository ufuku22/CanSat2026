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
    p2p_config.py    set/check P2P parameters
  esp32c3_usb_bridge/
    platformio.ini          PlatformIO設定
    src/main.cpp            PlatformIO用USB-UARTブリッジ
    usb_bridge_monitor.py   PC側のコマンド送受信モニタ
    esp32c3_usb_bridge.ino  Arduino IDE用USB-UARTブリッジ
  esp32c3_tlm922s_diagnostic/
    platformio.ini          TLM922S診断・P2P設定確認
    src/main.cpp            起動時にp2p get/set/saveを実行
  esp32c3_ground_station_receiver/
    platformio.ini          地上局受信ファーム
    ground_station_monitor.py PC側の受信ログ保存・JPEG復元
```

## 配線

### Raspberry Pi Zero WH to TLM922S-A

```text
Pi GPIO14 TXD, pin 8   -> TLM922S RXD
Pi GPIO15 RXD, pin 10  -> TLM922S TXD
Pi GND                 -> TLM922S GND
```

### ESP32-C3 to TLM922S-B

PlatformIOの各ESP32-C3プロジェクトでは以下のピンを使います。

```text
ESP32-C3 GPIO21 TX     -> TLM922S RXD
ESP32-C3 GPIO20 RX     -> TLM922S TXD
ESP32-C3 GND           -> TLM922S GND
```

Arduino IDE用の`esp32c3_usb_bridge.ino`だけはGPIO5 TX/GPIO4 RXが
初期値です。使用するプロジェクトと実際の配線を一致させてください。

```cpp
static const int TLM_RX_PIN = 4;
static const int TLM_TX_PIN = 5;
```

UARTは3.3Vロジックです。RS-232レベルを直接つながないでください。

## ESP32-C3側の準備: PlatformIO

1. VS Codeで `tlm922s_p2p/esp32c3_usb_bridge` フォルダを開く
2. PlatformIOの `Upload` を実行する
3. PC側で `usb_bridge_monitor.py` を起動する
4. TLM922Sの応答を確認する

```text
mod get_ver
```

`>> ...` のような応答が返れば、ESP32-C3経由のUARTは動いています。

```bash
cd tlm922s_p2p/esp32c3_usb_bridge
python usb_bridge_monitor.py --port COM4
```

ボードが `esp32-c3-devkitm-1` ではない場合は、`platformio.ini` の
`board` を使用しているESP32-C3ボードに合わせて変更してください。

UARTピンを変える場合は、`platformio.ini` のこの値だけ変更します。

```ini
build_flags =
  -D TLM_RX_PIN=20
  -D TLM_TX_PIN=21
  -D TLM_BAUD=115200
```

Arduino IDEで使いたい場合は、同じフォルダ内の
`esp32c3_usb_bridge.ino` を使えます。

## 実機で通信できたP2P設定

実機ログでは、2026年7月18日17:58に次の設定を確認した受信機で、
同日18:01にRaspberry Pi側からのパケットを連続受信できています。
地上局受信ファームウェアは、起動時にこの設定を自動確認し、異なる項目を
修正して`p2p save`します。

```text
p2p set_freq 922500000
p2p set_pwr 20
p2p set_sf 12
p2p set_bw 125
p2p set_cr 4/6
p2p set_prlen 16
p2p set_crc on
p2p set_iqi off
p2p set_sync 12
```

Raspberry Pi側では`CommunicationManager`が最初にシリアルポートを
開いた時、`p2p_config.py`の設定処理を同じ接続上で自動実行します。
受信側と違う項目だけを変更し、変更があった場合だけ`p2p save`します。
そのため、利用者が`p2p_config.py`を別に起動する必要はありません。

USBブリッジから手動設定した場合だけ、最後に次を実行します。

```text
p2p save
```

## 片方向テスト: Piから送信、PC側で受信

先にESP32-C3地上局受信ファームウェアを起動します。設定確認後に
`p2p rx 0`を自動実行するため、受信コマンドの手入力は不要です。
シリアルモニタに`P2P ready`と`> p2p rx 0`が表示されたら、
Raspberry Pi側で送信スクリプトを実行します。


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


ESP32-C3側のシリアルモニタから送信します。

```text
p2p tx 48656c6c6f
```

## 注意

- TLM922SのコマンドはASCII文字列で、終端はCRです。
- ESP32-C3のブリッジは、PC側のEnterをCRに変換してTLM922Sへ送ります。
- P2P設定は2台で完全に同じにしてください。
- 日本国内で電波を出す場合は、周波数、出力、アンテナ、送信時間が試験条件として適切か確認してください。
