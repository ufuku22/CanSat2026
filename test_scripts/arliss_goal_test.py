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


LOWER_DISTANCE_THRESHOLD_M = 0.1
UPPER_DISTANCE_THRESHOLD_M = 0.5
FORWARD_STOP_DISTANCE_M = 0.5


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
            "5分割画像による赤色探索を開始します。"
            f"直進停止距離は{FORWARD_STOP_DISTANCE_M:.1f} m、"
            f"停止距離が{LOWER_DISTANCE_THRESHOLD_M:.1f}～"
            f"{UPPER_DISTANCE_THRESHOLD_M:.1f} mなら検知成功とします"
        )
        result = navigator.detect_ball(
            driver,
            sensors,
            LOWER_DISTANCE_THRESHOLD_M,
            UPPER_DISTANCE_THRESHOLD_M,
            forward_stop_distance_m=FORWARD_STOP_DISTANCE_M,
        )

        print(f"赤色占有率: {result['red_ratio'] * 100:.2f}%")

        if not result["ball_detected"]:
            print("ボールを検知できませんでした")
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
