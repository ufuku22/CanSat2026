#!/usr/bin/env python3
"""前進中にスタックを検知し、既存の離脱動作を1回実行するテスト。"""

from __future__ import annotations

from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_controller import NavigationController
from sensor_manager import SensorManager


FORWARD_SPEED = 60.0
TEST_TIMEOUT_S = 30.0
POLL_INTERVAL_S = 0.01


class AccelerationLoggingSensors:
    """avoid_stuck()が取得した加速度を表示し、その他の処理は委譲する。"""

    def __init__(self, sensors: SensorManager) -> None:
        self._sensors = sensors

    def get_imu(self):
        imu = self._sensors.get_imu()
        accel_x, accel_y, accel_z = imu["accel_mps2"]
        print(
            "加速度: "
            f"X={float(accel_x):+.3f}, "
            f"Y={float(accel_y):+.3f}, "
            f"Z={float(accel_z):+.3f} m/s^2"
        )
        return imu

    def get_heading_deg(self) -> float:
        return self._sensors.get_heading_deg()


def main() -> int:
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        navigator = NavigationController()
        config = navigator.stuck_avoidance_config
        logging_sensors = AccelerationLoggingSensors(sensors)

        print("=== スタック検知・離脱テスト ===")
        print(f"前進出力: {FORWARD_SPEED:g}%")
        print(
            "スタック条件: "
            f"|X| <= {config.ACCEL_X_UPPER_MPS2:g} m/s^2、"
            f"|Y| <= {config.ACCEL_Y_UPPER_MPS2:g} m/s^2が"
            f"{config.DETECTION_DURATION_S:g}秒継続"
        )
        print(
            "離脱動作: "
            f"{config.REVERSE_DURATION_S:g}秒後退 → "
            f"右へ{config.RIGHT_TURN_ANGLE_DEG:g}度旋回 → "
            f"{config.FORWARD_DURATION_S:g}秒前進"
        )
        print(f"{TEST_TIMEOUT_S:g}秒以内に検知しなければ終了します。")
        input("周囲の安全を確認し、機体から離れてEnterを押してください")

        driver.drive(FORWARD_SPEED)
        print("前進開始。スタック判定を繰り返します。")
        deadline = time.monotonic() + TEST_TIMEOUT_S

        while time.monotonic() < deadline:
            if navigator.avoid_stuck(driver, logging_sensors):
                print("スタックを検知し、離脱動作が完了しました。")
                return 0
            time.sleep(POLL_INTERVAL_S)

        print("テスト時間内にスタックを検知しませんでした。")
        return 1

    except KeyboardInterrupt:
        print("\nテストを中断しました。")
        return 130
    finally:
        if driver is not None:
            driver.stop()
        if sensors is not None:
            sensors.close()
        if driver is not None:
            driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
