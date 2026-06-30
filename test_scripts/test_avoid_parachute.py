"""
test_avoid_parachute.py

前方カメラ画像から赤色パラシュートを判定し、
5分割結果に基づいて回避動作を確認するテストプログラムです。
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
    driver = None
    sensors = None

    try:
        print("=== パラシュート回避テスト開始 ===")

        driver = DriveController()
        sensors = SensorManager()
        sensors.setup()

        navigator = NavigationController()

        result = navigator.avoid_parachute(
            driver,
            sensors,
            red_threshold=0.05,
            move_speed=35,
            turn_speed=35,
            safe_forward_duration_s=2.0,
            cautious_forward_duration_s=0.4,
            turn_duration_s=0.35,
            far_turn_duration_s=0.65,
            max_attempts=10,
            capture_width=640,
            capture_height=480,
            capture_hdr=False,
            capture_timeout_ms=1000,
        )

        print("=== パラシュート回避テスト結果 ===")
        print(f"動作: {result['action']}")
        print(f"完了: {result['completed']}")
        print(f"試行回数: {result['attempts']}")

        if result["red_ratio"] is not None:
            print(f"赤色占有率: {result['red_ratio']:.3f}")

        print(f"しきい値: {result['red_threshold']:.3f}")
        print(f"最も安全な方向: {result['best_direction']}")

        if result["best_block_ratio"] is not None:
            print(f"最も安全な領域の赤色率: {result['best_block_ratio']:.3f}")

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