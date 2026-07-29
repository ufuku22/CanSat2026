# ESP32-C3 Ground Station Receiver

TLM922S の受信側を ESP32-C3 と PlatformIO で動かす最小構成です。

```text
Raspberry Pi -> TLM922S-A ~~ LoRa P2P ~~ TLM922S-B -> ESP32-C3 -> PC
```

ESP32-C3 は起動時にTLM922SのP2P設定を読み取り、実機で通信成功を確認
できた送信側設定と異なる項目を自動更新します。
変更した場合は`p2p save`でTLM922Sのフラッシュへ保存し、全項目の確認が
成功してから`p2p rx 0`を実行して無期限の受信待ちにします。

パケットを1個受けるたびにJSONをUSBシリアルモニタへ表示し、もう一度
`p2p rx 0`を送って次のパケットを待ちます。設定確認に失敗した場合は、
受信を開始せず10秒間隔で設定処理を再試行します。

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

地上局受信ファームウェアが起動時に各`p2p get_*`の結果を照合するため、
受信側の手動設定は不要です。送信側では`CommunicationManager`の開始時に
`p2p_config.py`の設定処理が自動実行され、同じ値へ揃います。利用者は
送信スクリプトだけを起動します。試験場所でこの周波数・出力・アンテナ
条件を使用できることは、電波を出す前に必ず確認してください。

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
P2P ready: freq=922500000 pwr=20 sf=12 bw=125 cr=4/6 prlen=16 crc=on iqi=off sync=12
> p2p rx 0
RX type=tlm seq=1 time=2026-05-27T08:00:00.000Z RSSI=-90 SNR=-12
JSON {"v":1,"type":"tlm","seq":1,"time":"...","data":{...}}
GPS lat=35.6687 lon=139.7613 alt=44.5 sat=8 fix=1
```

GPS 行は、受信した JSON に緯度と経度が入っている場合だけ表示されます。
