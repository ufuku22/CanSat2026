#!/usr/bin/env python3
"""CanSat2026の放出判定と着地判定を行うモジュール。"""

from __future__ import annotations

from collections import deque
import time
from typing import Optional

from logger import Logger
from sensor_manager import SensorManager


# 放出判定の調整値。実験結果に合わせてここを書き換える。
PRESSURE_RISE_THRESHOLD_HPA = 5.0
REQUIRED_CONSECUTIVE_COUNT = 3
PRESSURE_TIMEOUT_S = 60.0
MEASUREMENT_INTERVAL_S = 0.2

# 着地判定の調整値。実験結果に合わせてここを書き換える。
GRAVITY_MPS2 = 9.8
DEFAULT_TOLERANCE_MPS2 = 1.0
DEFAULT_AVERAGE_WINDOW_S = 10
DEFAULT_MEASUREMENT_INTERVAL_S = 0.5


def get_z_acceleration(sensor_manager: SensorManager) -> float:
    """sensor_manager.pyからZ軸加速度[m/s^2]を読む。"""
    imu = sensor_manager.get_imu()
    accel = imu["accel_mps2"]
    return float(accel[2])


def wait_for_pressure_rise(sensor_manager: SensorManager) -> bool:
    """気圧がしきい値以上に上昇するまで待つ。"""
    start_time = time.monotonic()
    base_pressure_hpa = float(sensor_manager.get_environment()["pressure_hpa"])
    consecutive_count = 0

    while time.monotonic() - start_time < PRESSURE_TIMEOUT_S:
        pressure_hpa = float(sensor_manager.get_environment()["pressure_hpa"])

        # 基準気圧からの上昇が連続して条件を満たすか確認する。
        if pressure_hpa - base_pressure_hpa >= PRESSURE_RISE_THRESHOLD_HPA:
            consecutive_count += 1
        else:
            consecutive_count = 0

        if consecutive_count >= REQUIRED_CONSECUTIVE_COUNT:
            return True

        time.sleep(MEASUREMENT_INTERVAL_S)

    return False


def judge_release(sensor_manager: SensorManager, logger: Logger | None = None) -> bool:
    """気圧上昇から放出を判定する。"""
    if logger is not None:
        logger.event("放出判定開始")

    if wait_for_pressure_rise(sensor_manager):
        if logger is not None:
            logger.event("放出成功")
        else:
            print("放出成功")
        return True

    if logger is not None:
        logger.event("放出判定失敗")

    return False


def average_z_acceleration(
    sensor_manager: SensorManager,
    *,
    average_window_s: float = DEFAULT_AVERAGE_WINDOW_S,
    measurement_interval_s: float = DEFAULT_MEASUREMENT_INTERVAL_S,
) -> float:
    """指定した時間幅でZ軸加速度を測り、平均値を返す。"""
    if average_window_s <= 0:
        raise ValueError("average_window_sは0より大きい値にしてください")
    if measurement_interval_s <= 0:
        raise ValueError("measurement_interval_sは0より大きい値にしてください")

    start_time = time.monotonic()
    values: list[float] = []

    while time.monotonic() - start_time < average_window_s:
        values.append(get_z_acceleration(sensor_manager))
        time.sleep(measurement_interval_s)

    if not values:
        raise RuntimeError("Z軸加速度を取得できませんでした")
    return sum(values) / len(values)


def is_landed(
    sensor_manager: SensorManager,
    *,
    target_z_accel_mps2: float = GRAVITY_MPS2,
    tolerance_mps2: float = DEFAULT_TOLERANCE_MPS2,
    average_window_s: float = DEFAULT_AVERAGE_WINDOW_S,
    measurement_interval_s: float = DEFAULT_MEASUREMENT_INTERVAL_S,
) -> bool:
    """Z軸加速度の時間平均が目標値付近に収まっていればTrueを返す。"""
    if tolerance_mps2 < 0:
        raise ValueError("tolerance_mps2は0以上にしてください")

    average_z = average_z_acceleration(
        sensor_manager,
        average_window_s=average_window_s,
        measurement_interval_s=measurement_interval_s,
    )
    return abs(average_z - target_z_accel_mps2) <= tolerance_mps2


def judge_landing(
    sensor_manager: SensorManager,
    *,
    logger: Logger | None = None,
    timeout_s: Optional[float] = None,
    target_z_accel_mps2: float = GRAVITY_MPS2,
    tolerance_mps2: float = DEFAULT_TOLERANCE_MPS2,
    average_window_s: float = DEFAULT_AVERAGE_WINDOW_S,
    measurement_interval_s: float = DEFAULT_MEASUREMENT_INTERVAL_S,
) -> bool:
    """着地判定が成立するまで監視する。timeout_sを超えたらFalseを返す。"""
    if tolerance_mps2 < 0:
        raise ValueError("tolerance_mps2は0以上にしてください")
    if average_window_s <= 0:
        raise ValueError("average_window_sは0より大きい値にしてください")
    if measurement_interval_s <= 0:
        raise ValueError("measurement_interval_sは0より大きい値にしてください")

    if logger is not None:
        logger.event("着地判定開始")

    start_time = time.monotonic()
    sample_count = max(1, int(average_window_s / measurement_interval_s))
    samples: deque[float] = deque(maxlen=sample_count)

    while timeout_s is None or time.monotonic() - start_time < timeout_s:
        samples.append(get_z_acceleration(sensor_manager))

        if len(samples) == sample_count:
            average_z = sum(samples) / len(samples)
            if abs(average_z - target_z_accel_mps2) <= tolerance_mps2:
                message = f"着地判定: Z軸加速度平均={average_z:.2f} m/s^2"
                if logger is not None:
                    logger.event(message)
                else:
                    print(message)
                return True

        time.sleep(measurement_interval_s)

    if logger is not None:
        logger.event("着地判定失敗")

    return False


def main() -> None:
    with SensorManager() as sensors:
        sensors.setup()
        landed = judge_landing(sensors, timeout_s=None)
        print("着地しました" if landed else "着地判定できませんでした")


if __name__ == "__main__":
    main()
