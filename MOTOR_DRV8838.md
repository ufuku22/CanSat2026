# DRV8838 モータードライバの簡単な動作確認

## 配線例

`motor_drv8838.py` の初期設定は BCM GPIO 番号です。

| DRV8838 | Raspberry Pi |
| --- | --- |
| PH | GPIO23 |
| EN | GPIO24 |
| nSLEEP | GPIO25 |
| VCC | 3.3V |
| GND | GND |
| VM | モータ用電源 + |
| OUT1 / OUT2 | モータ |

モータ用電源の GND と Raspberry Pi の GND は共通にしてください。
DRV8838 のデータシートでは、VCC と VM それぞれに 0.1 uF のバイパスコンデンサを GND へ入れる構成が推奨されています。

## Raspberry Pi 側の準備

```bash
sudo apt update
sudo apt install -y python3-gpiozero
```

## 実行例

```bash
# 正転を 2 秒
python3 motor_drv8838.py forward --speed 0.6 --seconds 2

# 逆転を 2 秒
python3 motor_drv8838.py reverse --speed 0.6 --seconds 2

# 正転、停止、逆転のデモ
python3 motor_drv8838.py demo --speed 0.5 --seconds 1.5

# スリープ
python3 motor_drv8838.py sleep
```

配線を変えた場合は、次のように GPIO 番号を指定できます。

```bash
python3 motor_drv8838.py forward --ph-pin 17 --en-pin 18 --sleep-pin 27
```

