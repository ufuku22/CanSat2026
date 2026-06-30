import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from fusing import fuse
from sensor_manager import SensorManager


DROP_WAIT_S = 10.0
FUSE_SECONDS = 3.0
DRIVE_WAIT_S = 5.0
DRIVE_SPEED = 100
DRIVE_SECONDS = 3.0


def print_environment_and_imu_once():
    sensors = SensorManager()
    try:
        sensors.environment.setup()
        sensors.imu.setup()

        environment = sensors.get_environment()
        imu = sensors.get_imu()

        print("=== センサ値 ===")
        print(f"気圧センサ: {environment}")
        print(f"9軸センサ: {imu}")
    finally:
        sensors.close()


def main():
    print("=== キャリア開放デモ ===")
    print(f"Enter後 {DROP_WAIT_S}秒で溶断")
    print(f"溶断後 {DRIVE_WAIT_S}秒で前進")
    input("CanSatをキャリアに入れて、準備できたらEnterを押してください")

    time.sleep(DROP_WAIT_S)

    print("溶断開始")
    fuse(FUSE_SECONDS)
    print("溶断終了")

    time.sleep(DRIVE_WAIT_S)

    driver = DriveController()
    try:
        print("前進開始")
        driver.drive(DRIVE_SPEED)
        time.sleep(DRIVE_SECONDS)
        driver.stop()
        print("前進終了")
        print_environment_and_imu_once()
    except KeyboardInterrupt:
        print("\nデモを中断しました")
    finally:
        driver.cleanup()


if __name__ == "__main__":
    main()
