"""
test_avoid_parachute.py

前方カメラ画像から赤色パラシュートを判定し、回避動作を確認するテストプログラムです。
"""

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_controller import NavigationController
from sensor_manager import SensorManager


def main():
    """パラシュート回避テストを実行する。"""
    driver = DriveController()
    sensors = SensorManager()
    navigator = NavigationController()

    try:
        print("=== パラシュート回避テスト開始 ===")
        result = navigator.avoid_parachute(driver, sensors)
        print("=== パラシュート回避テスト結果 ===")
        print(f"動作: {result['action']}")
        print(f"赤色占有率: {result['red_ratio']:.3f}")
        print(f"しきい値: {result['red_threshold']:.3f}")
        print(f"撮影画像: {result['image_path']}")
        print("=== パラシュート回避テスト終了 ===")

    except KeyboardInterrupt:
        print("\nパラシュート回避テストを中断しました")

    finally:
        driver.cleanup()
        sensors.close()


if __name__ == "__main__":
    main()
