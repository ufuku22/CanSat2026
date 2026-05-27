# ESP32-C3 Ground Station Receiver

TLM922S の受信側を ESP32-C3 と PlatformIO で動かす最小構成です。

```text
Raspberry Pi -> TLM922S-A ~~ LoRa P2P ~~ TLM922S-B -> ESP32-C3 -> PC
```

ESP32-C3 は起動後に `p2p rx 0` を実行し、TLM922S を無期限の受信待ちにします。
パケットを 1 個受けるたびに JSON を USB シリアルモニタへ表示し、もう一度 `p2p rx 0`
を送って次のパケットを待ちます。

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
RX type=tlm seq=1 time=2026-05-27T08:00:00.000Z RSSI=-90 SNR=-12
JSON {"v":1,"type":"tlm","seq":1,"time":"...","data":{...}}
GPS lat=35.6687 lon=139.7613 alt=44.5 sat=8 fix=1
```

GPS 行は、受信した JSON に緯度と経度が入っている場合だけ表示されます。

