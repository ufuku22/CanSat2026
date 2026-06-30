# 地上局PCでの通信監視・JPEG保存

実運用では、PlatformIO Monitorは書き込み後の起動確認までにして、通信監視は
`ground_station_monitor.py` に任せます。同じCOMポートを複数のアプリで同時に開けないことが多いためです。

画像パケットを受信すると、ESP32-C3は次の形式でPCへ中継します。

```text
IMG_PACKET <payload_hex> RSSI=<rssi> SNR=<snr>
```

PC側では、このフォルダの `ground_station_monitor.py` を起動しておくと、通信ログを保存しながら、必要なFECパケットが集まった時点で自動的にJPEGへ復元します。

通常はCOMポートを省略できます。ESP32-C3らしいUSBシリアルポートを自動判定します。

```bash
python ground_station_monitor.py --baudrate 115200 --image-dir received_images --log-dir ground_station_logs
```

自動判定がうまくいかない場合や、USBシリアル機器が複数ある場合は明示します。

```bash
python ground_station_monitor.py --port COM4 --baudrate 115200 --image-dir received_images --log-dir ground_station_logs
```

起動中の `ground_station_monitor.py` にTLM922Sコマンドを入力してEnterを押すと、
ESP32-C3経由でTLM922Sへ送信できます。

```text
p2p get_freq
p2p set_freq 922500000
p2p save
```

コマンド送信後、ESP32-C3側は自動で `p2p rx 0` を送り直して受信待ちに戻ります。
画像受信中に設定変更コマンドを送ると、その受信は途切れる可能性があります。

保存されるログは次の3種類です。

```text
ground_station_logs/raw_serial_*.log      受信した全シリアル行
ground_station_logs/non_image_*.log       画像パケット以外の通信ログ
ground_station_logs/image_transfer_*.log  画像復元の進行状況と保存結果
```

TLM922SのUART診断やP2P設定確認は、別フォルダの
`../esp32c3_tlm922s_diagnostic` を使います。

Raspberry Pi側から送信する例です。

```bash
cd ~/CanSat2026
python send_image_test.py input.jpg
```

現在の最小構成ではACKを使いません。各画像パケットは復元に必要なメタ情報を持っているため、STARTパケットが落ちる問題はありません。初期設定では約33%のFEC冗長パケットを追加します。
