# ESP32-C3 Ground Station Receiver

TLM922S 受信側を ESP32-C3 + PlatformIO で動かすための最小構成です。

```text
Raspberry Pi -> TLM922S-A ~~ LoRa P2P ~~ TLM922S-B -> ESP32-C3 -> PC
```

ESP32-C3 は起動後に自動で `p2p rx 0` を実行し、パケットを受け取るたびにもう一度 `p2p rx 0` に戻ります。PC 側は PlatformIO Monitor を開いておくだけで、受信した JSON パケットを確認できます。

## 配線

`platformio.ini` の初期設定は Seeed XIAO ESP32C3 向けです。

```text
ESP32-C3 GPIO20 RX -> TLM922S TXD
ESP32-C3 GPIO21 TX -> TLM922S RXD
ESP32-C3 GND       -> TLM922S GND
```

別の ESP32-C3 ボードを使う場合は `platformio.ini` の `TLM_RX_PIN` / `TLM_TX_PIN` を変更してください。

## 実行

1. VS Code でこのフォルダを開く
2. PlatformIO の `Upload` を実行
3. PlatformIO の `Monitor` を開く

受信すると次のように表示されます。

```text
RX type=tlm seq=1 time=2026-05-27T08:00:00.000Z RSSI=-90 SNR=-12
JSON {"v":1,"type":"tlm","seq":1,"time":"...","data":{...}}
```

画像送信や再送要求はまだ入れていません。
