#!/usr/bin/env python3
"""赤ボール誘導を実行する実機テスト。"""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_controller import NavigationController
from navigation_goal import guide_to_red_ball, guide_to_square_zone
from sensor_manager import SensorManager


def main() -> int:
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        sensors.distance.setup()

        navigation_controller = NavigationController()
        result = guide_to_red_ball(navigation_controller, driver, sensors)
        print(
            f"赤ボール誘導結果: target_reached={result['target_reached']}, "
            f"reason={result['reason']}"
        )
        print(f"試行回数: {result['steps']}")
        print(f"最終距離: {result['last_distance_m']}")
        if not result["target_reached"]:
            return 1

        square_result = guide_to_square_zone(
            navigation_controller,
            driver,
            sensors,
        )
        print(
            "スクエアゾーン誘導結果: "
            f"square_zone_reached={square_result['square_zone_reached']}, "
            f"reason={square_result['reason']}"
        )
        print(f"接近したボール数: {square_result['approached_balls']}")
        print(f"最終距離: {square_result['last_distance_m']}")
        return 0 if square_result["square_zone_reached"] else 1

    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("赤ボール誘導テストを中断しました")
        return 130
    finally:
        if sensors is not None:
            sensors.close()
        if driver is not None:
            driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
