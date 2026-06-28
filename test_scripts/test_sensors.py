"""
test_sensors.py

SensorManager の既存関数を使って、各センサの値を読み取って表示するテストプログラムです。
"""

from pathlib import Path
import select
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor_manager import SensorManager


READ_INTERVAL_S = 1.0


def enter_pressed():
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False
    sys.stdin.readline()
    return True


def read_sensor(title, setup_func, read_func, is_ready):
    try:
        if not is_ready:
            setup_func()
            is_ready = True
        return is_ready, read_func()
    except Exception as exc:
        return False, f"エラー: {type(exc).__name__}: {exc}"


def main():
    """Enterが押されるまで各センサの値を1秒間隔で読み取り続ける。"""
    sensors = SensorManager()
    sensor_tests = [
        ("BME280 環境センサ", sensors.environment.setup, sensors.get_environment, False),
        ("BNO055 IMU", sensors.imu.setup, sensors.get_imu, False),
        ("LC76G GNSS", sensors.gnss.setup, sensors.get_gnss, False),
        ("TSD20 距離センサ", sensors.distance.setup, sensors.get_distance_m, False),
    ]

    try:
        print("=== センサ読み取りテスト開始 ===")
        print("Enterキーを押すと終了します")

        while True:
            print(f"\n--- {time.strftime('%Y-%m-%d %H:%M:%S')} ---")
            updated_tests = []
            for title, setup_func, read_func, is_ready in sensor_tests:
                is_ready, result = read_sensor(title, setup_func, read_func, is_ready)
                status = "OK" if is_ready else "NG"
                print(f"{title} [{status}]: {result}")
                updated_tests.append((title, setup_func, read_func, is_ready))
            sensor_tests = updated_tests

            if enter_pressed():
                break

            time.sleep(READ_INTERVAL_S)

        print("\n=== センサ読み取りテスト終了 ===")

    except KeyboardInterrupt:
        print("\nセンサ読み取りテストを中断しました")

    finally:
        sensors.close()


if __name__ == "__main__":
    main()
