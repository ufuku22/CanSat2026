# SensorManager 説明書

`sensor_manager.py` は、ローバーに搭載するセンサをまとめて扱うためのファイルです。
他の制御プログラムからは、基本的に `SensorManager` を使います。

## 扱うセンサ

- 環境センサ: AE-BME280
- 9軸センサ: AE-BNO055-BO
- GNSS: LC76G GPSモジュール
- 距離センサ: TSD20
- 前方カメラ: Raspberry Pi カメラモジュールV3

前方カメラ以外はI2C通信を使います。

## 基本の使い方

```python
from sensor_manager import SensorManager

with SensorManager() as sensors:
    sensors.setup()

    env = sensors.get_environment()
    imu = sensors.get_imu()
    gnss = sensors.get_gnss()
    distance = sensors.get_distance_m()
```

`with` を使うと、処理が終わったあとにI2Cバスを自動で閉じます。

## 環境センサ

```python
env = sensors.get_environment()
```

出力例:

```python
{
    "temperature_c": 24.8,
    "pressure_hpa": 1008.6,
    "humidity_percent": 52.3,
}
```

値を単体で使う例:

```python
temperature = env["temperature_c"]
pressure = env["pressure_hpa"]
humidity = env["humidity_percent"]

if temperature > 35.0:
    print("温度が高い")
```

## 9軸センサ

```python
imu = sensors.get_imu()
```

出力例:

```python
{
    "heading_deg": 135.25,
    "roll_deg": -1.38,
    "pitch_deg": 4.56,
    "accel_mps2": (0.02, -0.13, 9.79),
    "gyro_dps": (0.0, 0.06, -0.12),
    "calibration": 255,
}
```

値を単体で使う例:

```python
heading = imu["heading_deg"]
roll = imu["roll_deg"]
pitch = imu["pitch_deg"]

ax, ay, az = imu["accel_mps2"]
gx, gy, gz = imu["gyro_dps"]
```

方位だけを速く読みたい場合は、次のメソッドを使います。

```python
heading = sensors.get_heading_deg()
```

BNO055のNDOF fusionモードは100Hz出力なので、方位だけなら100Hz制御ループで使う想定です。

```python
import time

while True:
    heading = sensors.get_heading_deg()
    print(heading)
    time.sleep(0.01)
```

100Hzで使うループでは、カメラ撮影やGNSS読み取りを一緒に行わない方が安定します。

## GNSS

```python
gnss = sensors.get_gnss()
```

出力例:

```python
{
    "latitude_deg": 35.6687,
    "longitude_deg": 139.7613,
    "altitude_m": 44.5,
    "satellites": 8,
    "fix_quality": 1,
    "raw": "$GNGGA,...",
}
```

値を単体で使う例:

```python
lat = gnss["latitude_deg"]
lon = gnss["longitude_deg"]

if lat is not None and lon is not None:
    print("位置情報が使えます")
else:
    print("まだ測位できていません")
```

測位できていない項目は `None` になります。

## 距離センサ

```python
distance = sensors.get_distance_m()
```

出力例:

```python
1.234
```

単位はmです。
測距できない場合は `None` になります。

```python
if distance is not None and distance < 0.5:
    print("前方に障害物あり")
```

## 前方カメラ

```python
image_path = sensors.capture_front_image(
    width=1280,
    height=720,
    hdr=True,
    timeout_ms=2000,
)
```

出力例:

```python
/home/pi/cansat_camera_images/front_20260525_134210.jpg
```

`width` と `height` で解像度、`hdr` でHDRの有無を指定できます。

## 全部まとめて読む

```python
data = sensors.read_all(with_camera=False)
```

出力例:

```python
{
    "environment": {
        "temperature_c": 24.8,
        "pressure_hpa": 1008.6,
        "humidity_percent": 52.3,
    },
    "imu": {
        "heading_deg": 135.25,
        "roll_deg": -1.38,
        "pitch_deg": 4.56,
        "accel_mps2": (0.02, -0.13, 9.79),
        "gyro_dps": (0.0, 0.06, -0.12),
        "calibration": 255,
    },
    "gnss": {
        "latitude_deg": 35.6687,
        "longitude_deg": 139.7613,
        "altitude_m": 44.5,
        "satellites": 8,
        "fix_quality": 1,
        "raw": "$GNGGA,...",
    },
    "distance_m": 1.234,
}
```

カメラ撮影も含めたい場合は、次のようにします。

```python
data = sensors.read_all(with_camera=True)
image_path = data["front_image"]
```

カメラ撮影は時間がかかるため、制御ループ中では必要なときだけ使ってください。

## 他のプログラムから使う例

```python
from sensor_manager import SensorManager

with SensorManager() as sensors:
    sensors.setup()

    heading = sensors.get_heading_deg()
    distance = sensors.get_distance_m()
    env = sensors.get_environment()

    temperature = env["temperature_c"]

    if distance is not None and distance < 0.5:
        print("停止")
    elif heading < 90:
        print("右へ補正")

    print(f"temperature = {temperature:.1f} C")
```
