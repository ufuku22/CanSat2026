#!/usr/bin/env python3
"""距離が閾値以下になるまで方位を保って直進する実機テスト。"""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_goal import GoalNavigator
from sensor_manager import SensorManager


DISTANCE_THRESHOLD_M = 1.5
BASE_SPEED = 60.0


def main() -> int:
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()

        # rider_forward() で使用するセンサだけを初期化する。
        sensors.imu.setup()
        sensors.distance.setup()

        navigator = GoalNavigator()
        print(
            f"rider_forwardテスト開始: "
            f"距離閾値={DISTANCE_THRESHOLD_M:.3f} m, "
            f"速度={BASE_SPEED:.1f}%"
        )

        stop_distance = navigator.rider_forward(
            driver,
            sensors,
            DISTANCE_THRESHOLD_M,
            base_speed=BASE_SPEED,
        )

        if stop_distance is None:
            print("距離を測定できなかったため、テストに失敗しました")
            return 1

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
