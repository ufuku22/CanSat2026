"""
test_sensors.py

SensorManager の既存関数を使って、各センサの値を読み取って表示するテストプログラムです。
"""

from pathlib import Path
import select
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
# test_scripts から実行しても、リポジトリ直下の sensor_manager.py を読めるようにする。
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor_manager import SensorManager


READ_INTERVAL_S = 3.0


def enter_pressed():
    """Enterキーが押されたかを、待ち時間なしで確認する。"""
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False
    sys.stdin.readline()
    return True


def read_sensor(title, setup_func, read_func, is_ready):
    try:
        # 初回または前回失敗時だけ setup し、成功したセンサは次回以降そのまま読む。
        if not is_ready:
            setup_func()
            is_ready = True
        return is_ready, read_func()
    except Exception as exc:
        # 読み取りや初期化に失敗したセンサはNGに戻し、次の周回で再セットアップする。
        return False, f"エラー: {type(exc).__name__}: {exc}"


def get_gnss_summary(sensors):
    """GNSSの全データから、動作確認で見たい項目だけを抜き出す。"""
    gnss = sensors.get_gnss()
    return {
        "has_fix": gnss.get("has_fix"),
        "latitude_deg": gnss.get("latitude_deg"),
        "longitude_deg": gnss.get("longitude_deg"),
        "satellites": gnss.get("satellites"),
    }


def main():
    """Enterが押されるまで各センサの値を一定間隔で読み取り続ける。"""
    sensors = SensorManager()
    # 各センサごとに、表示名・初期化関数・読み取り関数・初期化済みかをまとめて扱う。
    sensor_tests = [
        ("BME280 環境センサ", sensors.environment.setup, sensors.get_environment, False),
        ("BNO055 IMU", sensors.imu.setup, sensors.get_imu, False),
        ("LC76G GNSS", sensors.gnss.setup, lambda: get_gnss_summary(sensors), False),
        ("TSD20 距離センサ", sensors.distance.setup, sensors.get_distance_m, False),
    ]

    try:
        print("=== センサ読み取りテスト開始 ===")
        print("Enterキーを押すと終了します")

        while True:
            print(f"\n--- {time.strftime('%Y-%m-%d %H:%M:%S')} ---")
            updated_tests = []
            for title, setup_func, read_func, is_ready in sensor_tests:
                # センサごとの準備状態を更新しながら、今回の読み取り結果を表示する。
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
