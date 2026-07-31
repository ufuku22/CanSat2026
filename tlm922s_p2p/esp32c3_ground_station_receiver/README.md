# ESP32-C3 Ground Station Receiver

TLM922S の受信側を ESP32-C3 と PlatformIO で動かす最小構成です。

```text
Raspberry Pi -> TLM922S-A ~~ LoRa P2P ~~ TLM922S-B -> ESP32-C3 -> PC
```

ESP32-C3 は起動直後に`p2p rx 0`を実行し、無期限の受信待ちにします。
通常の通信経路ではP2P設定の取得・変更を行いません。

パケットを1個受けるたびにJSONをUSBシリアルモニタへ表示し、もう一度
`p2p rx 0`を送って次のパケットを待ちます。

## P2P設定

送信側・受信側で使用する設定は次のとおりです。2026年7月18日の実機
ログでは、17:58にこの受信設定を確認し、18:01にRaspberry Pi側からの
パケットを連続受信できています。

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

通信試験の最初にRaspberry Pi側で`p2p_config.py`を単独実行し、設定・
保存・確認が完了してから送信コードを起動します。通常の送受信コードは
送信側で直接`p2p tx`、受信側で直接`p2p rx 0`を実行します。試験場所で
この周波数・出力・アンテナ条件を使用できることは、電波を出す前に必ず
確認してください。

## 配線

`platformio.ini` の初期設定は Seeed XIAO ESP32C3 向けです。

```text
ESP32-C3 GPIO20 RX -> TLM922S TXD
ESP32-C3 GPIO21 TX -> TLM922S RXD
ESP32-C3 GND       -> TLM922S GND
```

別の ESP32-C3 ボードを使う場合は、`platformio.ini` の `TLM_RX_PIN` と
`TLM_TX_PIN` を実際の配線に合わせて変更してください。

## 実行

1. VS Code でこのフォルダを開く
2. PlatformIO の `Upload` を実行する
3. PlatformIO の `Monitor` を開く

受信すると次のように表示されます。

```text
Waiting for packets from Raspberry Pi...
> p2p rx 0
RX type=tlm seq=1 time=2026-05-27T08:00:00.000Z RSSI=-90 SNR=-12
JSON {"v":1,"type":"tlm","seq":1,"time":"...","data":{...}}
GPS lat=35.6687 lon=139.7613 alt=44.5 sat=8 fix=1
```

GPS 行は、受信した JSON に緯度と経度が入っている場合だけ表示されます。
