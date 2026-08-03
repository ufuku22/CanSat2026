#!/usr/bin/env python3
"""PD直進中にスタックを監視し、回避後も直進を再開するテスト。"""

from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import NavigationMotionConfig
from drive_controller import DriveController
from navigation_controller import NavigationController
from sensor_manager import SensorManager


def main() -> int:
    speed = float(input("モーター出力[%]: "))
    if not 0.0 < speed <= 100.0:
        raise ValueError("モーター出力は0より大きく100以下にしてください")

    driver = None
    sensors = None
    try:
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        navigator = NavigationController()

        target_heading = float(sensors.get_heading_deg())
        prev_error = 0.0
        interval = NavigationMotionConfig.FOLLOW_FORWARD_LOOP_INTERVAL_S
        print(f"PD直進開始: 出力={speed:g}%, Ctrl+Cで終了")

        while True:
            _, _, prev_error = navigator.drive_toward_heading(
                driver,
                sensors,
                target_heading=target_heading,
                base_speed=speed,
                prev_error=prev_error,
                loop_interval=interval,
            )
            if navigator.avoid_stuck(driver, sensors):
                target_heading = float(sensors.get_heading_deg())
                prev_error = 0.0
                print(
                    f"スタック回避完了: 方位{target_heading:.1f}度でPD直進を再開"
                )
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nテストを終了します")
        return 130
    finally:
        if driver is not None:
            driver.cleanup()
        if sensors is not None:
            sensors.close()


if __name__ == "__main__":
    raise SystemExit(main())
