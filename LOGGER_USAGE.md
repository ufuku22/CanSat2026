# Loggerの使い方

`logger.py` の `Logger` は、ミッション中のイベントログとセンサログを同じファイルへ保存するためのクラスです。

## 基本

```python
from logger import Logger

logger = Logger()
logger.event("Mission start")
```

`Logger()` はデフォルトでプロジェクト直下の `logs/log.txt` に追記します。
別名のログファイルにしたい場合だけ `filename` を指定します。

```python
logger = Logger(filename="mission.txt")
```

## イベントを記録する

```python
logger.event("放出判定開始")
```

`event()` は画面に表示し、同じ内容をログファイルにも保存します。
ミッションの区切り、成功、失敗、例外内容などは基本的に `event()` で記録します。

## センサ値を記録する

センサ値を記録する場合は、`sensor()` を使います。

```python
sensor_data = sensors.read_all()
logger.sensor(sensor_data)
```

`Logger` 作成時に `sensor_manager` を渡しておくと、引数なしでもセンサ値を読んで保存できます。

```python
logger = Logger(sensor_manager=sensors)
logger.sensor()
```

特定の値だけを短く残したい場合も、`sensor()` を使います。

```python
environment = sensors.get_environment()
logger.sensor("pressure_hpa", environment["pressure_hpa"])
```

複数の値をまとめて残す場合は、辞書で渡します。

```python
imu = sensors.get_imu()
ax, ay, az = imu["accel_mps2"]

logger.sensor({
    "ax": ax,
    "ay": ay,
    "az": az,
})
```

`sensor()` は、全体のセンサログにも、必要な値だけの短いログにも使います。

## 特定のセンサだけ読む

`SensorManager` には、センサごとに値を読むメソッドがあります。
必要なセンサだけ読んで、`logger.sensor()` に渡します。

気圧・温度・湿度だけ読む場合:

```python
environment = sensors.get_environment()

logger.sensor(environment)
```

IMUの加速度だけ読む場合:

```python
imu = sensors.get_imu()
logger.sensor(imu)
```

GPSだけ読む場合:

```python
gnss = sensors.get_gnss()
logger.sensor(gnss)
```

距離センサだけ読む場合:

```python
distance_m = sensors.get_distance_m()
logger.sensor("distance_m", distance_m)
```

## 処理の開始・完了・失敗を記録する

センサ初期化など、例外が出る可能性がある処理は `step()` で囲めます。

```python
logger.step("BME280 setup", sensors.environment.setup)
```

`step()` は開始、完了、失敗を自動で記録します。
失敗判定は「渡した関数が例外を出したかどうか」です。

リトライしたい場合は `retries` と `retry_delay` を指定します。

```python
environment = logger.step(
    "BME280 read",
    sensors.get_environment,
    retries=5,
    retry_delay=0.5,
)
```

戻り値がある関数を渡した場合、`step()` はその戻り値を返します。
ただし、戻り値が `False` でも例外が出なければ完了扱いです。

## 判定関数で使う

`judge.py` の放出判定・着地判定には `logger` を渡せます。

```python
from judge import judge_landing, judge_release

released = judge_release(sensors, logger=logger)
landed = judge_landing(sensors, logger=logger)
```

`logger` を渡すと、判定開始・成功・失敗が同じログファイルに保存されます。

## 経過時間をリセットする

ログの `t:` は `Logger` を作ってからの経過秒です。
ミッション開始時などに基準時刻を戻したい場合は `reset_timer()` を使います。

```python
logger.reset_timer()
logger.event("Mission start")
```
