"""
test_sensors.py

SensorManager の既存関数を使って、各センサの値を読み取って表示するテストプログラムです。
"""

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor_manager import SensorManager


def run_sensor_test(title, setup_func, read_func):
    print(f"\n=== {title} ===")
    try:
        print("初期化中...")
        setup_func()
        print("読み取り中...")
        print(read_func())
    except Exception as exc:
        print(f"エラー: {type(exc).__name__}: {exc}")


def main():
    """各センサの値を1回読み取って表示する。"""
    sensors = SensorManager()

    try:
        print("=== センサ読み取りテスト開始 ===")
        run_sensor_test("BME280 環境センサ", sensors.environment.setup, sensors.get_environment)
        run_sensor_test("BNO055 IMU", sensors.imu.setup, sensors.get_imu)
        run_sensor_test("LC76G GNSS", sensors.gnss.setup, sensors.get_gnss)
        run_sensor_test("TSD20 距離センサ", sensors.distance.setup, sensors.get_distance_m)

        print("\n=== センサ読み取りテスト終了 ===")

    except KeyboardInterrupt:
        print("\nセンサ読み取りテストを中断しました")

    finally:
        sensors.close()


if __name__ == "__main__":
    main()
