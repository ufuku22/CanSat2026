"""
test_avoid_parachute.py

前方カメラ画像から赤色パラシュートを判定し、
赤色が検知されなくなるまで90度時計回りに旋回して回避するテストプログラムです。
"""

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_controller import NavigationController
from navigation_defaults import navigation_default
from sensor_manager import SensorManager


AVOID_PARACHUTE_DEFAULT = NavigationController.avoid_parachute


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

        result = navigator.avoid_parachute(
            driver,
            sensors,
            red_threshold=navigation_default(AVOID_PARACHUTE_DEFAULT, "red_threshold"),

            # 赤色が見えなくなった後の直進
            move_speed=navigation_default(AVOID_PARACHUTE_DEFAULT, "move_speed"),
            move_duration_s=navigation_default(AVOID_PARACHUTE_DEFAULT, "move_duration_s"),

            # 赤色検知時の旋回
            rotate_angle_deg=navigation_default(AVOID_PARACHUTE_DEFAULT, "rotate_angle_deg"),
            rotate_speed=navigation_default(AVOID_PARACHUTE_DEFAULT, "rotate_speed"),
            rotate_tolerance_deg=navigation_default(AVOID_PARACHUTE_DEFAULT, "rotate_tolerance_deg"),
            rotate_timeout_s=navigation_default(AVOID_PARACHUTE_DEFAULT, "rotate_timeout_s"),

            # 最大確認回数
            max_attempts=navigation_default(AVOID_PARACHUTE_DEFAULT, "max_attempts"),

            # カメラ設定
            capture_width=navigation_default(AVOID_PARACHUTE_DEFAULT, "capture_width"),
            capture_height=navigation_default(AVOID_PARACHUTE_DEFAULT, "capture_height"),
            capture_hdr=navigation_default(AVOID_PARACHUTE_DEFAULT, "capture_hdr"),
            capture_timeout_ms=navigation_default(AVOID_PARACHUTE_DEFAULT, "capture_timeout_ms"),
        )

        print("=== パラシュート回避テスト結果 ===")
        print(f"動作: {result['action']}")
        print(f"完了: {result['completed']}")
        print(f"試行回数: {result['attempts']}")
        print(f"赤色検知: {result['red_detected']}")

        if result["red_ratio"] is not None:
            print(f"赤色占有率: {result['red_ratio']:.3f}")

        print(f"しきい値: {result['red_threshold']:.3f}")
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
