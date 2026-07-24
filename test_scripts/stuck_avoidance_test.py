#!/usr/bin/env python3
"""前進中に9軸センサーで衝突を検知し、回避動作を1回実行するテスト。"""

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
    """avoid_stuck()が取得した線形加速度を表示し、その他の処理は委譲する。"""

    def __init__(self, sensors: SensorManager) -> None:
        self._sensors = sensors
        self._previous_sample_time: float | None = None
        self._previous_accel_xy: tuple[float, float] | None = None

    def get_altitude_motion(self):
        motion = self._sensors.get_altitude_motion()
        accel_x, accel_y, accel_z = motion["linear_accel_mps2"]
        accel_x = float(accel_x)
        accel_y = float(accel_y)
        accel_z = float(accel_z)
        now = time.monotonic()
        horizontal_accel = (
            accel_x ** 2 + accel_y ** 2
        ) ** 0.5
        horizontal_jerk = 0.0
        if (
            self._previous_sample_time is not None
            and self._previous_accel_xy is not None
        ):
            sample_interval = now - self._previous_sample_time
            if sample_interval > 0.0:
                horizontal_jerk = (
                    (accel_x - self._previous_accel_xy[0]) ** 2
                    + (accel_y - self._previous_accel_xy[1]) ** 2
                ) ** 0.5 / sample_interval
        self._previous_sample_time = now
        self._previous_accel_xy = (accel_x, accel_y)

        print(
            "線形加速度: "
            f"X={accel_x:+.3f}, "
            f"Y={accel_y:+.3f}, "
            f"Z={accel_z:+.3f}, "
            f"水平合成={horizontal_accel:.3f} m/s^2, "
            f"変化率={horizontal_jerk:.3f} m/s^3"
        )
        return motion

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

        print("=== 衝突検知・回避テスト ===")
        print(f"前進出力: {FORWARD_SPEED:g}%")
        print(
            "衝突条件: "
            f"水平線形加速度 >= "
            f"{config.COLLISION_ACCEL_THRESHOLD_MPS2:g} m/s^2、"
            f"変化率 >= {config.COLLISION_JERK_THRESHOLD_MPS3:g} m/s^3"
        )
        print(f"走行開始後{config.STARTUP_IGNORE_S:g}秒間は判定しません。")
        print(
            "離脱動作: "
            f"{config.REVERSE_DURATION_S:g}秒後退 → "
            f"右へ{config.RIGHT_TURN_ANGLE_DEG:g}度旋回 → "
            f"{config.FORWARD_DURATION_S:g}秒前進"
        )
        print(f"{TEST_TIMEOUT_S:g}秒以内に検知しなければ終了します。")
        input("周囲の安全を確認し、機体から離れてEnterを押してください")

        driver.drive(FORWARD_SPEED)
        print("前進開始。衝突判定を繰り返します。")
        deadline = time.monotonic() + TEST_TIMEOUT_S

        while time.monotonic() < deadline:
            if navigator.avoid_stuck(driver, logging_sensors):
                print("衝突を検知し、回避動作が完了しました。")
                return 0
            time.sleep(POLL_INTERVAL_S)

        print("テスト時間内に衝突を検知しませんでした。")
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
