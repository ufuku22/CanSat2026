#!/usr/bin/env python3
"""9軸センサを使った姿勢復帰を1回実行する。"""

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_controller import NavigationController
from sensor_manager import SensorManager


def main():
    input("機体から離れ、準備できたらEnterを押してください")

    driver = DriveController()
    sensors = SensorManager()
    try:
        sensors.imu.setup()
        NavigationController().restore_posture(driver, sensors)
    finally:
        sensors.close()
        driver.cleanup()


if __name__ == "__main__":
    main()
