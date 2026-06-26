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


def print_section(title, data):
    print(f"\n=== {title} ===")
    print(data)


def main():
    """各センサの値を1回読み取って表示する。"""
    sensors = SensorManager()

    try:
        print("=== センサ読み取りテスト開始 ===")
        print("センサを初期化します")
        sensors.setup()

        print_section("BME280 環境センサ", sensors.get_environment())
        print_section("BNO055 IMU", sensors.get_imu())
        print_section("LC76G GNSS", sensors.get_gnss())
        print_section("TSD20 距離センサ", sensors.get_distance_m())

        print("\n=== センサ読み取りテスト終了 ===")

    except KeyboardInterrupt:
        print("\nセンサ読み取りテストを中断しました")

    finally:
        sensors.close()


if __name__ == "__main__":
    main()
