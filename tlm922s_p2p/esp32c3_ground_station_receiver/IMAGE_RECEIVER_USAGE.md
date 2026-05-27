# JPEG画像の自律受信・保存

画像パケットを受信すると、ESP32-C3はJSON表示ではなく次の形式でPCへ中継します。

```text
IMG_PACKET <payload_hex> RSSI=<rssi> SNR=<snr>
```

PC側では、このフォルダの `pc_image_receiver.py` を起動しておくと、必要なFECパケットが集まった時点で自動的にJPEGへ復元し、`received_images` に保存します。

```bash
python pc_image_receiver.py --port COM4 --baudrate 115200 --output-dir received_images
```

Raspberry Pi側から送信する例です。

```bash
cd ~/CanSat2026
python send_image_test.py input.jpg --port /dev/serial0
```

現在の最小構成ではACKを使いません。各画像パケットは復元に必要なメタ情報を持っているため、STARTパケットが落ちる問題はありません。初期設定では約33%のFEC冗長パケットを追加します。
