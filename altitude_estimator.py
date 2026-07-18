#!/usr/bin/env python3
"""BME280とBNO055を使った相対高度推定。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from statistics import median
import time
from typing import Any

from sensor_manager import BME280_ADDR, SensorManager


IMU_INTERVAL_S = 0.02
PRESSURE_INTERVAL_S = 0.1
CALIBRATION_SECONDS = 3.0
ALTITUDE_CORRECTION_GAIN = 0.05
IMU_VELOCITY_DECAY_TIME_S = 2.0
PRESSURE_MEDIAN_SAMPLES = 5
BARO_VELOCITY_WINDOW_S = 3.0
BARO_VELOCITY_MIN_SPAN_S = 1.0
MIN_GRAVITY_NORM_MPS2 = 8.0
MAX_GRAVITY_NORM_MPS2 = 12.0
DRY_AIR_GAS_CONSTANT = 287.05  # J/(kg*K)
GRAVITY_MPS2 = 9.80665


@dataclass(frozen=True)
class AltitudeEstimate:
    """1回の更新後の高度推定値。"""

    pressure_hpa: float
    baro_altitude_raw_m: float
    baro_altitude_m: float
    baro_regression_velocity_mps: float
    fused_altitude_m: float
    vertical_velocity_mps: float
    vertical_accel_mps2: float
    motion: dict[str, Any] | None
    imu_valid: bool
    pressure_updated: bool


def relative_altitude_m(
    reference_pressure_hpa: float,
    pressure_hpa: float,
    air_temperature_c: float,
) -> float:
    """基準気圧からの相対高度を返す。"""
    temperature_k = air_temperature_c + 273.15
    return (
        DRY_AIR_GAS_CONSTANT
        * temperature_k
        / GRAVITY_MPS2
        * math.log(reference_pressure_hpa / pressure_hpa)
    )


def vertical_acceleration_mps2(motion: dict[str, Any]) -> float:
    """重力方向を鉛直方向として線形加速度を射影する。"""
    linear_accel = motion["linear_accel_mps2"]
    gravity = motion["gravity_mps2"]
    gravity_norm = math.sqrt(sum(value * value for value in gravity))
    if not MIN_GRAVITY_NORM_MPS2 <= gravity_norm <= MAX_GRAVITY_NORM_MPS2:
        raise RuntimeError(
            f"BNO055の重力ベクトルが異常です: {gravity_norm:.2f} m/s²"
        )
    return sum(a * g for a, g in zip(linear_accel, gravity)) / gravity_norm


def configure_bme280_for_altitude(sensors: SensorManager) -> None:
    """BME280を約10 Hzの高度測定向けに設定する。"""
    sensors.environment.setup()
    # 気圧x8・温度x1、待機62.5ms、IIRなしで約10Hzの変化を残す。
    sensors.bus.write_byte_data(BME280_ADDR, 0xF4, 0x30)  # SLEEP MODE
    time.sleep(0.01)
    sensors.bus.write_byte_data(BME280_ADDR, 0xF5, 0x20)
    sensors.bus.write_byte_data(BME280_ADDR, 0xF4, 0x33)  # NORMAL MODE


def calibrate_altitude(sensors: SensorManager) -> tuple[float, float, int]:
    """静止状態から基準気圧と鉛直加速度オフセットを求める。"""
    print(f"{CALIBRATION_SECONDS:.0f}秒間、機体を静止させてください。")
    pressures: list[float] = []
    vertical_accels: list[float] = []
    calibration = 0
    start_time = time.monotonic()
    next_pressure_time = start_time

    while time.monotonic() - start_time < CALIBRATION_SECONDS:
        loop_start = time.monotonic()
        motion = sensors.get_altitude_motion()
        try:
            vertical_accels.append(vertical_acceleration_mps2(motion))
        except RuntimeError:
            pass
        calibration = motion["calibration"]

        if loop_start >= next_pressure_time:
            pressures.append(sensors.get_environment()["pressure_hpa"])
            next_pressure_time += PRESSURE_INTERVAL_S

        sleep_s = IMU_INTERVAL_S - (time.monotonic() - loop_start)
        if sleep_s > 0:
            time.sleep(sleep_s)

    if not pressures or not vertical_accels:
        raise RuntimeError("基準値を取得できませんでした。")
    return (
        sum(pressures) / len(pressures),
        sum(vertical_accels) / len(vertical_accels),
        calibration,
    )


class AltitudeEstimator:
    """IMUの積分値を気圧高度で補正して相対高度を推定する。"""

    def __init__(
        self,
        sensors: SensorManager,
        air_temperature_c: float,
        reference_pressure_hpa: float,
        accel_bias_mps2: float,
        *,
        tolerate_read_errors: bool = False,
    ) -> None:
        self.sensors = sensors
        self.air_temperature_c = air_temperature_c
        self.reference_pressure_hpa = reference_pressure_hpa
        self.accel_bias_mps2 = accel_bias_mps2
        self.tolerate_read_errors = tolerate_read_errors

        now = time.monotonic()
        self._previous_time = now
        self._next_pressure_time = now
        self._pressure_hpa = reference_pressure_hpa
        self._baro_altitude_raw_m = 0.0
        self._baro_altitude_m = 0.0
        self._baro_regression_velocity_mps = 0.0
        self._fused_altitude_m = 0.0
        self._vertical_velocity_mps = 0.0
        self._vertical_accel_mps2 = 0.0
        self._motion: dict[str, Any] | None = None
        self._imu_valid = False
        self._pressure_samples: deque[float] = deque(
            maxlen=PRESSURE_MEDIAN_SAMPLES
        )
        self._baro_velocity_history: deque[tuple[float, float]] = deque()

    def update(self, now: float | None = None) -> AltitudeEstimate:
        """センサを読み取り、現在の推定値を返す。"""
        loop_time = time.monotonic() if now is None else now
        dt = loop_time - self._previous_time
        self._previous_time = loop_time

        self._update_from_imu(dt)
        pressure_updated = loop_time >= self._next_pressure_time
        if pressure_updated:
            self._update_from_pressure(loop_time)
            self._next_pressure_time += PRESSURE_INTERVAL_S
            if self._next_pressure_time < loop_time:
                self._next_pressure_time = loop_time + PRESSURE_INTERVAL_S

        return AltitudeEstimate(
            pressure_hpa=self._pressure_hpa,
            baro_altitude_raw_m=self._baro_altitude_raw_m,
            baro_altitude_m=self._baro_altitude_m,
            baro_regression_velocity_mps=self._baro_regression_velocity_mps,
            fused_altitude_m=self._fused_altitude_m,
            vertical_velocity_mps=self._vertical_velocity_mps,
            vertical_accel_mps2=self._vertical_accel_mps2,
            motion=self._motion,
            imu_valid=self._imu_valid,
            pressure_updated=pressure_updated,
        )

    def _update_from_imu(self, dt: float) -> None:
        try:
            self._motion = self.sensors.get_altitude_motion()
        except Exception:
            if not self.tolerate_read_errors:
                raise
            self._motion = None
            self._imu_valid = False
            self._vertical_accel_mps2 = 0.0
        else:
            try:
                self._vertical_accel_mps2 = (
                    vertical_acceleration_mps2(self._motion) - self.accel_bias_mps2
                )
                self._imu_valid = True
            except RuntimeError:
                self._vertical_accel_mps2 = 0.0
                self._imu_valid = False

        self._vertical_velocity_mps *= math.exp(
            -dt / IMU_VELOCITY_DECAY_TIME_S
        )
        self._fused_altitude_m += (
            self._vertical_velocity_mps * dt
            + 0.5 * self._vertical_accel_mps2 * dt * dt
        )
        if self._imu_valid:
            self._vertical_velocity_mps += self._vertical_accel_mps2 * dt

    def _update_from_pressure(self, now: float) -> None:
        try:
            pressure_hpa = float(self.sensors.get_environment()["pressure_hpa"])
        except Exception:
            if not self.tolerate_read_errors:
                raise
            return

        self._pressure_hpa = pressure_hpa
        self._baro_altitude_raw_m = relative_altitude_m(
            self.reference_pressure_hpa,
            pressure_hpa,
            self.air_temperature_c,
        )
        self._pressure_samples.append(self._baro_altitude_raw_m)
        self._baro_altitude_m = median(self._pressure_samples)
        self._baro_velocity_history.append((now, self._baro_altitude_m))
        while (
            self._baro_velocity_history
            and now - self._baro_velocity_history[0][0] > BARO_VELOCITY_WINDOW_S
        ):
            self._baro_velocity_history.popleft()
        self._baro_regression_velocity_mps = self._regression_velocity_mps()

        self._fused_altitude_m += ALTITUDE_CORRECTION_GAIN * (
            self._baro_altitude_m - self._fused_altitude_m
        )

    def _regression_velocity_mps(self) -> float:
        samples = self._baro_velocity_history
        if (
            len(samples) < 2
            or samples[-1][0] - samples[0][0] < BARO_VELOCITY_MIN_SPAN_S
        ):
            return 0.0
        mean_time = sum(sample[0] for sample in samples) / len(samples)
        mean_altitude = sum(sample[1] for sample in samples) / len(samples)
        denominator = sum((sample[0] - mean_time) ** 2 for sample in samples)
        if denominator == 0:
            return 0.0
        return sum(
            (sample[0] - mean_time) * (sample[1] - mean_altitude)
            for sample in samples
        ) / denominator
