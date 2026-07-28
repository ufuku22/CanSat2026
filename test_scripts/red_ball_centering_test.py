#!/usr/bin/env python3
"""認識した赤ボールを画像中央に合わせる実機テスト。"""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_controller import NavigationController
from navigation_goal import align_red_ball_to_center
from sensor_manager import SensorManager


def main() -> int:
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()

        result = align_red_ball_to_center(
            NavigationController(),
            driver,
            sensors,
        )
        print(
            f"中央合わせ結果: centered={result['centered']}, "
            f"red_detected={result['red_detected']}, "
            f"reason={result['reason']}"
        )
        print(f"試行回数: {result['steps']}")
        return 0 if result["centered"] else 1

    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("赤ボール中央合わせテストを中断しました")
        return 130
    finally:
        if sensors is not None:
            sensors.close()
        if driver is not None:
            driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
