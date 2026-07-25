#!/usr/bin/env python3
"""CanSat2026の放出判定と着地判定を行うモジュール。"""

from __future__ import annotations

from collections import deque
import math
from statistics import median
import time
from typing import Callable, Literal, Optional

from logger import Logger
from sensor_manager import SensorManager


PRESSURE_MEASUREMENT_INTERVAL_S = 0.2
PRESSURE_RELEASE_TIMEOUT_S = 60.0
GRAVITY_MPS2 = 9.8
DEFAULT_TOLERANCE_MPS2 = 1.0
DEFAULT_CONTINUOUS_DURATION_S = 10.0
DEFAULT_MEASUREMENT_INTERVAL_S = 0.5
MIN_VALID_PRESSURE_HPA = 300.0
MAX_VALID_PRESSURE_HPA = 1100.0
PRESSURE_MEDIAN_SAMPLES = 3


def get_squared_acceleration(sensor_manager: SensorManager) -> float:
    """3軸加速度の二乗和 ax^2 + ay^2 + az^2 を返す。"""
    imu = sensor_manager.get_imu()
    ax, ay, az = (float(value) for value in imu["accel_mps2"])
    return ax * ax + ay * ay + az * az


PressureThresholdState = Literal["above", "below"]


def is_valid_pressure_hpa(pressure_hpa: float) -> bool:
    return (
        math.isfinite(pressure_hpa)
        and MIN_VALID_PRESSURE_HPA <= pressure_hpa <= MAX_VALID_PRESSURE_HPA
    )


def read_median_pressure_hpa(
    sensor_manager: SensorManager,
    *,
    measurement_interval_s: float = PRESSURE_MEASUREMENT_INTERVAL_S,
) -> float:
    """有効な気圧を3件取得し、その中央値を返す。"""
    pressures: list[float] = []
    while len(pressures) < PRESSURE_MEDIAN_SAMPLES:
        pressure_hpa = float(
            sensor_manager.get_environment()["pressure_hpa"]
        )
        if is_valid_pressure_hpa(pressure_hpa):
            pressures.append(pressure_hpa)
        if len(pressures) < PRESSURE_MEDIAN_SAMPLES:
            time.sleep(measurement_interval_s)
    return float(median(pressures))


def judge_release(
    sensor_manager: SensorManager,
    logger: Logger | None = None,
    *,
    ground_pressure_hpa: float,
    above_threshold_offsets_hpa: tuple[float, float],
    below_threshold_offsets_hpa: tuple[float, float],
    timeout_s: Optional[float] = PRESSURE_RELEASE_TIMEOUT_S,
    measurement_interval_s: float = PRESSURE_MEASUREMENT_INTERVAL_S,
    on_third_threshold: Callable[[float], None] | None = None,
) -> bool:
    """2閾値を下回った後に2閾値を上回ったら放出成功と判定する。"""
    logger = logger if logger is not None else Logger(log_to_file=False)
    logger.event("放出判定開始")
    if not is_valid_pressure_hpa(ground_pressure_hpa):
        logger.event(f"放出判定失敗: 基準気圧が外れ値 {ground_pressure_hpa} hPa")
        return False

    checks: tuple[tuple[float, PressureThresholdState], ...] = (
        (below_threshold_offsets_hpa[0], "below"),
        (below_threshold_offsets_hpa[1], "below"),
        (above_threshold_offsets_hpa[0], "above"),
        (above_threshold_offsets_hpa[1], "above"),
    )
    start_time = time.monotonic()
    pressure_history: deque[float] = deque(maxlen=PRESSURE_MEDIAN_SAMPLES)

    for check_number, (threshold_offset_hpa, expected_state) in enumerate(
        checks,
        start=1,
    ):
        while timeout_s is None or time.monotonic() - start_time < timeout_s:
            pressure_hpa = float(
                sensor_manager.get_environment()["pressure_hpa"]
            )
            if not is_valid_pressure_hpa(pressure_hpa):
                pressure_history.clear()
                logger.event(f"放出気圧判定: 外れ値を除外 {pressure_hpa} hPa")
                time.sleep(measurement_interval_s)
                continue

            pressure_history.append(pressure_hpa)
            if len(pressure_history) < PRESSURE_MEDIAN_SAMPLES:
                time.sleep(measurement_interval_s)
                continue

            median_pressure_hpa = float(median(pressure_history))
            threshold_pressure_hpa = (
                ground_pressure_hpa - threshold_offset_hpa
            )
            pressure_state: PressureThresholdState = (
                "above"
                if median_pressure_hpa >= threshold_pressure_hpa
                else "below"
            )
            if pressure_state == expected_state:
                logger.event(
                    f"放出気圧判定 {check_number}/4: "
                    f"{expected_state}, 閾値={threshold_pressure_hpa:.2f} hPa, "
                    f"中央値={median_pressure_hpa:.2f} hPa"
                )
                if check_number == 3 and on_third_threshold is not None:
                    on_third_threshold(median_pressure_hpa)
                break

            time.sleep(measurement_interval_s)
        else:
            logger.event(f"放出気圧判定 {check_number}/4: タイムアウト")
            logger.event("放出判定失敗")
            return False

    logger.event("放出成功")
    return True


def judge_landing(
    sensor_manager: SensorManager,
    *,
    logger: Logger | None = None,
    timeout_s: Optional[float] = None,
    target_accel_mps2: float = GRAVITY_MPS2,
    tolerance_mps2: float = DEFAULT_TOLERANCE_MPS2,
    continuous_duration_s: float = DEFAULT_CONTINUOUS_DURATION_S,
    measurement_interval_s: float = DEFAULT_MEASUREMENT_INTERVAL_S,
) -> bool:
    """3軸加速度が一定時間連続して許容範囲内なら着地と判定する。"""
    logger = logger if logger is not None else Logger(log_to_file=False)
    logger.event("着地判定開始")

    start_time = time.monotonic()
    within_range_since: float | None = None

    while timeout_s is None or time.monotonic() - start_time < timeout_s:
        accel_mps2 = math.sqrt(get_squared_acceleration(sensor_manager))
        measurement_time = time.monotonic()

        if abs(accel_mps2 - target_accel_mps2) <= tolerance_mps2:
            if within_range_since is None:
                within_range_since = measurement_time
            if measurement_time - within_range_since >= continuous_duration_s:
                message = f"着地判定: 3軸加速度={accel_mps2:.2f} m/s^2"
                logger.event(message)
                return True
        else:
            within_range_since = None

        time.sleep(measurement_interval_s)

    logger.event("着地判定失敗")

    return False


def main() -> None:
    logger = Logger(log_to_file=False)
    with SensorManager() as sensors:
        sensors.setup()
        landed = judge_landing(sensors, logger=logger, timeout_s=None)
        logger.event("着地しました" if landed else "着地判定できませんでした")


if __name__ == "__main__":
    main()
