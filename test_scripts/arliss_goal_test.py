#!/usr/bin/env python3
"""ARLISS向けの赤コーン誘導を確認する実機テスト。"""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_controller import NavigationController
from navigation_goal import guide_to_red_cone
from sensor_manager import SensorManager


def main() -> int:
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()

        # 赤コーン誘導で使用する9軸センサだけを初期化する。
        sensors.imu.setup()

        navigator = NavigationController()

        print("赤コーン誘導を開始します。config.pyのデフォルト設定を使用します")
        result = guide_to_red_cone(navigator, driver, sensors)

        print(
            f"誘導結果: goal_reached={result['goal_reached']}, "
            f"reason={result['reason']}"
        )
        print(f"試行回数: {result['steps']}")

        if not result["goal_reached"]:
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
