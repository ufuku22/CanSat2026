#!/usr/bin/env python3
"""Z軸加速度の時間平均から着地判定を行うモジュール。"""

from __future__ import annotations

from collections import deque
import time
from typing import Optional

from sensor_manager import SensorManager


GRAVITY_MPS2 = 9.8
DEFAULT_TOLERANCE_MPS2 = 1.0
DEFAULT_AVERAGE_WINDOW_S = 10
DEFAULT_MEASUREMENT_INTERVAL_S = 0.5


class LandingJudge:
    """SensorManagerのZ軸加速度を使って着地状態を判定する。"""

    def __init__(
        self,
        sensor_manager: SensorManager,
        *,
        target_z_accel_mps2: float = GRAVITY_MPS2,
        tolerance_mps2: float = DEFAULT_TOLERANCE_MPS2,
        average_window_s: float = DEFAULT_AVERAGE_WINDOW_S,
        measurement_interval_s: float = DEFAULT_MEASUREMENT_INTERVAL_S,
    ) -> None:
        self.sensor_manager = sensor_manager
        self.target_z_accel_mps2 = float(target_z_accel_mps2)
        self.tolerance_mps2 = float(tolerance_mps2)
        self.average_window_s = float(average_window_s)
        self.measurement_interval_s = float(measurement_interval_s)

        if self.tolerance_mps2 < 0:
            raise ValueError("tolerance_mps2は0以上にしてください")
        if self.average_window_s <= 0:
            raise ValueError("average_window_sは0より大きい値にしてください")
        if self.measurement_interval_s <= 0:
            raise ValueError("measurement_interval_sは0より大きい値にしてください")

    def get_z_acceleration(self) -> float:
        """sensor_manager.pyからZ軸加速度[m/s^2]を読む。"""
        imu = self.sensor_manager.get_imu()
        accel = imu["accel_mps2"]
        return float(accel[2])

    def average_z_acceleration(self) -> float:
        """指定した時間幅でZ軸加速度を測り、平均値を返す。"""
        start_time = time.monotonic()
        values: list[float] = []

        while time.monotonic() - start_time < self.average_window_s:
            values.append(self.get_z_acceleration())
            time.sleep(self.measurement_interval_s)

        if not values:
            raise RuntimeError("Z軸加速度を取得できませんでした")
        return sum(values) / len(values)

    def is_landed(self) -> bool:
        """Z軸加速度の時間平均が9.8付近に収まっていればTrueを返す。"""
        average_z = self.average_z_acceleration()
        return abs(average_z - self.target_z_accel_mps2) <= self.tolerance_mps2

    def wait_for_landing(self, timeout_s: Optional[float] = None) -> bool:
        """着地判定が成立するまで監視する。timeout_sを超えたらFalseを返す。"""
        start_time = time.monotonic()
        sample_count = max(1, int(self.average_window_s / self.measurement_interval_s))
        samples: deque[float] = deque(maxlen=sample_count)

        while timeout_s is None or time.monotonic() - start_time < timeout_s:
            samples.append(self.get_z_acceleration())

            if len(samples) == sample_count:
                average_z = sum(samples) / len(samples)
                if abs(average_z - self.target_z_accel_mps2) <= self.tolerance_mps2:
                    print(f"着地判定: Z軸加速度平均={average_z:.2f} m/s^2")
                    return True

            time.sleep(self.measurement_interval_s)

        return False


def judge_landing(
    sensor_manager: SensorManager,
    *,
    timeout_s: Optional[float] = None,
    tolerance_mps2: float = DEFAULT_TOLERANCE_MPS2,
    average_window_s: float = DEFAULT_AVERAGE_WINDOW_S,
    measurement_interval_s: float = DEFAULT_MEASUREMENT_INTERVAL_S,
) -> bool:
    """LandingJudgeを簡単に使うための関数。"""
    judge = LandingJudge(
        sensor_manager,
        tolerance_mps2=tolerance_mps2,
        average_window_s=average_window_s,
        measurement_interval_s=measurement_interval_s,
    )
    return judge.wait_for_landing(timeout_s=timeout_s)


def main() -> None:
    with SensorManager() as sensors:
        sensors.setup()
        landed = judge_landing(sensors, timeout_s=None)
        print("着地しました" if landed else "着地判定できませんでした")


if __name__ == "__main__":
    main()
