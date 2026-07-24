#!/usr/bin/env python3
"""前進と9軸センサーによる衝突回避を繰り返すテスト。"""

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


POLL_INTERVAL_S = 0.01


class AccelerationLoggingSensors:
    """avoid_stuck()が取得した線形加速度を表示し、その他の処理は委譲する。"""

    def __init__(self, sensors: SensorManager, config) -> None:
        self._sensors = sensors
        self._forward_axis = str(config.SENSOR_FORWARD_AXIS).lower()
        self._forward_sign = float(config.SENSOR_FORWARD_SIGN)
        self._previous_sample_time: float | None = None
        self._previous_forward_accel: float | None = None

    def get_altitude_motion(self):
        motion = self._sensors.get_altitude_motion()
        accel_x, accel_y, accel_z = motion["linear_accel_mps2"]
        accel_x = float(accel_x)
        accel_y = float(accel_y)
        accel_z = float(accel_z)
        forward_accel = {
            "x": accel_x,
            "y": accel_y,
            "z": accel_z,
        }[self._forward_axis] * self._forward_sign
        now = time.monotonic()
        forward_jerk = 0.0
        if (
            self._previous_sample_time is not None
            and self._previous_forward_accel is not None
        ):
            sample_interval = now - self._previous_sample_time
            if sample_interval > 0.0:
                forward_jerk = (
                    forward_accel - self._previous_forward_accel
                ) / sample_interval
        self._previous_sample_time = now
        self._previous_forward_accel = forward_accel

        print(
            "線形加速度: "
            f"X={accel_x:+.3f}, "
            f"Y={accel_y:+.3f}, "
            f"Z={accel_z:+.3f}, "
            f"前方向={forward_accel:+.3f} m/s^2, "
            f"前方向変化率={forward_jerk:+.3f} m/s^3"
        )
        return motion

    def get_heading_deg(self) -> float:
        return self._sensors.get_heading_deg()

    def reset(self) -> None:
        self._previous_sample_time = None
        self._previous_forward_accel = None


def main() -> int:
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        navigator = NavigationController()
        config = navigator.stuck_avoidance_config
        logging_sensors = AccelerationLoggingSensors(sensors, config)

        print("=== 衝突検知・回避テスト ===")
        print(f"前進出力: {driver.FORWARD_SPEED:g}%")
        forward_direction = (
            f"{config.SENSOR_FORWARD_AXIS}"
            f"{'+' if config.SENSOR_FORWARD_SIGN > 0 else '-'}"
        )
        print(f"センサー前方向: {forward_direction}")
        print(
            "衝突条件: "
            f"前方向加速度 <= "
            f"-{config.COLLISION_DECEL_THRESHOLD_MPS2:g} m/s^2、"
            f"前方向変化率 <= "
            f"-{config.COLLISION_DECEL_JERK_THRESHOLD_MPS3:g} m/s^3"
        )
        print(f"走行開始後{config.STARTUP_IGNORE_S:g}秒間は判定しません。")
        print(
            "回避動作: "
            f"{config.REVERSE_DURATION_S:g}秒後退 → "
            f"右へ{config.RIGHT_TURN_ANGLE_DEG:g}度旋回"
        )
        print("Ctrl+Cで終了するまで前進と衝突回避を繰り返します。")
        input("周囲の安全を確認し、機体から離れてEnterを押してください")

        driver.drive(driver.FORWARD_SPEED)
        print("前進開始。衝突判定を繰り返します。")
        avoidance_count = 0

        while True:
            if navigator.avoid_stuck(driver, logging_sensors):
                avoidance_count += 1
                print(
                    f"衝突回避が完了しました。回避回数={avoidance_count}"
                )
                logging_sensors.reset()
                driver.drive(driver.FORWARD_SPEED)
                print("前進を再開し、次の衝突を待ちます。")
            time.sleep(POLL_INTERVAL_S)

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
