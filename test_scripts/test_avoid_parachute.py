"""
test_avoid_parachute.py

前方カメラ画像から紫色パラシュートを判定し、
必要なら90度右旋回してPD制御で前進するテストプログラムです。
"""

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from image_processor import ImageProcessor
from navigation_controller import NavigationController
from sensor_manager import SensorManager


def main():
    """パラシュート回避テストを実行する。"""
    driver = None
    sensors = None

    try:
        print("=== パラシュート回避テスト開始 ===")

        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()

        navigator = NavigationController()
        image_processor = ImageProcessor()

        result = navigator.avoid_parachute(
            driver,
            sensors,
            image_processor=image_processor,
        )

        print("=== パラシュート回避テスト結果 ===")
        print(f"動作: {result['action']}")
        print(f"完了: {result['completed']}")
        print(f"試行回数: {result['attempts']}")
        print(f"紫色検知: {result['purple_detected']}")

        if result["purple_ratio"] is not None:
            print(f"紫色占有率: {result['purple_ratio']:.3f}")

        print(f"しきい値: {result['purple_threshold']:.3f}")
        print(f"直進速度: {result['move_speed']:.1f}")
        print(f"直進時間: {result['move_duration_s']:.1f} 秒")
        print(f"旋回角度: {result['rotate_angle_deg']:.1f} 度")
        print(f"旋回速度: {result['rotate_speed']:.1f}")

        print("=== パラシュート回避テスト終了 ===")

    except KeyboardInterrupt:
        print("\nパラシュート回避テストを中断しました")

    finally:
        if driver is not None:
            driver.cleanup()
        if sensors is not None:
            sensors.close()


if __name__ == "__main__":
    main()
