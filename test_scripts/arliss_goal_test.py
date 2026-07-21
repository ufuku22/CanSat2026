#!/usr/bin/env python3
"""ARLISS向けのボール接近・赤色検知を確認する実機テスト。"""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_goal import GoalNavigator
from sensor_manager import SensorManager


TARGET_DISTANCE_M = 2.0
DISTANCE_SCAN_ANGLE_DEG = 10.0
FORWARD_STOP_DISTANCE_M = 0.5
CENTER_RED_RATIO_THRESHOLD = 0.01
FOLLOW_FORWARD_DURATION_S = 1.0


def main() -> int:
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()

        # このテストで使用する9軸センサと距離センサだけを初期化する。
        sensors.imu.setup()
        sensors.distance.setup()

        navigator = GoalNavigator()

        print(
            "赤色探索を開始します。赤色方向へ向いた後、"
            f"画面中央の赤色割合が{CENTER_RED_RATIO_THRESHOLD * 100:.1f}%を"
            "超えた場合に、"
            f"{DISTANCE_SCAN_ANGLE_DEG:.1f}度ずつ距離を測定し、"
            f"{TARGET_DISTANCE_M:.1f} m以内を検知した方向から"
            f"{FORWARD_STOP_DISTANCE_M:.1f} mまで直進します"
        )
        result = navigator.detect_ball(
            driver,
            sensors,
            center_red_ratio_threshold=CENTER_RED_RATIO_THRESHOLD,
            follow_forward_duration_s=FOLLOW_FORWARD_DURATION_S,
            target_distance_m=TARGET_DISTANCE_M,
            distance_scan_angle_deg=DISTANCE_SCAN_ANGLE_DEG,
            forward_stop_distance_m=FORWARD_STOP_DISTANCE_M,
        )

        print(f"赤色占有率: {result['red_ratio'] * 100:.2f}%")

        if not result["ball_detected"]:
            print(f"ボールを検知できませんでした: {result['reason']}")
            return 1

        print(f"停止距離: {result['stop_distance_m']:.3f} m")

        return 0

    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("テストを中断しました")
        return 130
    finally:
        if sensors is not None:
            sensors.close()
        if driver is not None:
            driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
