# CanSat2026

## BME280 を Raspberry Pi Zero WH で読む

Raspberry Pi Zero WH と BME280 は I2C で接続します。

| BME280 | Raspberry Pi Zero WH |
| --- | --- |
| VIN / VCC | 3.3V |
| GND | GND |
| SDA | GPIO2 / SDA, 物理ピン 3 |
| SCL | GPIO3 / SCL, 物理ピン 5 |

Raspberry Pi 側で I2C を有効化します。

```bash
sudo raspi-config
```

`Interface Options` から `I2C` を有効にしてください。

必要な Python ライブラリを入れます。

```bash
sudo apt update
sudo apt install -y python3-smbus i2c-tools
python3 -m pip install -r requirements.txt
```

センサの I2C アドレスを確認します。

```bash
i2cdetect -y 1
```

多くの BME280 は `0x76` または `0x77` と表示されます。

1 回だけ測定する場合:

```bash
python3 bme280_reader.py --address 0x76
```

1 秒ごとに測定し続ける場合:

```bash
python3 bme280_reader.py --address 0x76 --count 0 --interval 1
```

プログラムから扱いやすい JSON 形式で出力する場合:

```bash
python3 bme280_reader.py --address 0x76 --json
```

値が取れるかだけ確認する場合:

```bash
python3 test_bme280_once.py --address 0x76
```
