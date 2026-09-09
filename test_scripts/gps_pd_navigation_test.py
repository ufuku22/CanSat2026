#!/usr/bin/env python3
"""Drive toward a GPS goal with NavigationController PD control."""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_controller import NavigationController
from sensor_manager import SensorManager

TARGET_LATITUDE_DEG = 35.9188814    # 目標緯度
TARGET_LONGITUDE_DEG = 139.9093615  # 目標経度


def setup_navigation_sensors(sensors: SensorManager) -> None:
    sensors.imu.setup()
    sensors.gnss.setup()


def main() -> int:
    navigator = NavigationController(TARGET_LATITUDE_DEG, TARGET_LONGITUDE_DEG)
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()
        setup_navigation_sensors(sensors)
        print(
            f"目標座標: lat={TARGET_LATITUDE_DEG:.7f}, "
            f"lon={TARGET_LONGITUDE_DEG:.7f}"
        )

        reached_goal = navigator.follow_target(
            driver,
            sensors,
            status_callback=print,
        )
        print("ゴール成功" if reached_goal else "ゴール失敗")
        return 0 if reached_goal else 1

    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("ゴール失敗")
        return 130
    finally:
        if sensors is not None:
            sensors.close()
        if driver is not None:
            driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
