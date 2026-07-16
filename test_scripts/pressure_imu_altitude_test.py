#!/usr/bin/env python3
"""BME280とBNO055を融合して相対高度をCSVへ記録する。"""

from __future__ import annotations

from collections import deque
import csv
from datetime import datetime
import math
from pathlib import Path
import select
from statistics import median
import sys
import time


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor_manager import BME280_ADDR, SensorManager


IMU_INTERVAL_S = 0.02
PRESSURE_INTERVAL_S = 0.1
DISPLAY_INTERVAL_S = 1.0
CALIBRATION_SECONDS = 3.0
ALTITUDE_CORRECTION_GAIN = 0.05
DEFAULT_VELOCITY_CORRECTION_GAIN = 0.0
IMU_VELOCITY_DECAY_TIME_S = 2.0
PRESSURE_MEDIAN_SAMPLES = 5
BARO_VELOCITY_WINDOW_S = 3.0
BARO_VELOCITY_MIN_SPAN_S = 1.0
MIN_GRAVITY_NORM_MPS2 = 8.0
MAX_GRAVITY_NORM_MPS2 = 12.0
DRY_AIR_GAS_CONSTANT = 287.05  # J/(kg*K)
GRAVITY_MPS2 = 9.80665
LOG_DIR = Path(__file__).resolve().parent / "altitude_csv_logs"

CSV_FIELDS = [
    "timestamp",
    "elapsed_s",
    "air_temperature_c",
    "velocity_correction_gain",
    "imu_velocity_decay_time_s",
    "reference_pressure_hpa",
    "accel_bias_mps2",
    "pressure_hpa",
    "baro_altitude_raw_m",
    "baro_altitude_m",
    "baro_regression_velocity_mps",
    "fused_altitude_m",
    "vertical_velocity_mps",
    "vertical_accel_mps2",
    "linear_accel_x_mps2",
    "linear_accel_y_mps2",
    "linear_accel_z_mps2",
    "gravity_x_mps2",
    "gravity_y_mps2",
    "gravity_z_mps2",
    "calibration",
    "imu_valid",
    "pressure_updated",
]


def input_air_temperature_c() -> float:
    while True:
        try:
            temperature_c = float(input("外気温 [°C] を入力してください: "))
        except ValueError:
            print("数値で入力してください。")
            continue
        if temperature_c <= -273.15:
            print("-273.15°Cより高い値を入力してください。")
            continue
        return temperature_c


def input_velocity_correction_gain() -> float:
    while True:
        raw = input(
            "気圧高度からIMU速度への補正係数 "
            f"[Enter={DEFAULT_VELOCITY_CORRECTION_GAIN}]: "
        ).strip()
        if not raw:
            return DEFAULT_VELOCITY_CORRECTION_GAIN
        try:
            gain = float(raw)
        except ValueError:
            print("数値で入力してください。")
            continue
        if gain < 0:
            print("0以上の値を入力してください。")
            continue
        return gain


def relative_altitude_m(
    reference_pressure_hpa: float,
    pressure_hpa: float,
    air_temperature_c: float,
) -> float:
    temperature_k = air_temperature_c + 273.15
    return (
        DRY_AIR_GAS_CONSTANT
        * temperature_k
        / GRAVITY_MPS2
        * math.log(reference_pressure_hpa / pressure_hpa)
    )


def vertical_acceleration_mps2(motion: dict) -> float:
    linear_accel = motion["linear_accel_mps2"]
    gravity = motion["gravity_mps2"]
    gravity_norm = math.sqrt(sum(value * value for value in gravity))
    if not MIN_GRAVITY_NORM_MPS2 <= gravity_norm <= MAX_GRAVITY_NORM_MPS2:
        raise RuntimeError(
            f"BNO055の重力ベクトルが異常です: {gravity_norm:.2f} m/s²"
        )
    return sum(a * g for a, g in zip(linear_accel, gravity)) / gravity_norm


def regression_velocity_mps(samples: deque[tuple[float, float]]) -> float:
    if len(samples) < 2 or samples[-1][0] - samples[0][0] < BARO_VELOCITY_MIN_SPAN_S:
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


def enter_pressed() -> bool:
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False
    sys.stdin.readline()
    return True


def configure_bme280_for_logging(sensors: SensorManager) -> None:
    sensors.environment.setup()
    # 気圧x8・温度x1、待機62.5ms、IIRなしで約10Hzの変化を残す。
    sensors.bus.write_byte_data(BME280_ADDR, 0xF4, 0x30)  # SLEEP MODE
    time.sleep(0.01)
    sensors.bus.write_byte_data(BME280_ADDR, 0xF5, 0x20)
    sensors.bus.write_byte_data(BME280_ADDR, 0xF4, 0x33)  # NORMAL MODE


def calibrate(sensors: SensorManager) -> tuple[float, float, int]:
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


def main() -> None:
    air_temperature_c = input_air_temperature_c()
    velocity_correction_gain = input_velocity_correction_gain()
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    output_path = LOG_DIR / f"altitude_{datetime.now():%Y%m%d_%H%M%S}.csv"

    with SensorManager() as sensors:
        configure_bme280_for_logging(sensors)
        sensors.imu.setup()
        time.sleep(1.0)
        reference_pressure_hpa, accel_bias_mps2, calibration = calibrate(sensors)

        print(f"基準気圧: {reference_pressure_hpa:.2f} hPa = 0.00 m")
        print(f"鉛直加速度オフセット: {accel_bias_mps2:+.4f} m/s²")
        print(f"BNO055校正値: 0x{calibration:02X}")
        print(f"気圧高度からIMU速度への補正係数: {velocity_correction_gain}")
        print("測定を開始します。Enterキーで終了します。")

        start_time = time.monotonic()
        previous_time = start_time
        previous_pressure_time = start_time
        next_pressure_time = start_time
        next_display_time = start_time
        pressure_hpa = reference_pressure_hpa
        baro_altitude_raw_m = 0.0
        baro_altitude_m = 0.0
        baro_regression_velocity_mps = 0.0
        fused_altitude_m = 0.0
        vertical_velocity_mps = 0.0
        baro_altitude_history: deque[float] = deque(
            [0.0] * PRESSURE_MEDIAN_SAMPLES,
            maxlen=PRESSURE_MEDIAN_SAMPLES,
        )
        baro_velocity_history: deque[tuple[float, float]] = deque()
        rows = 0

        with output_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()

            while not enter_pressed():
                loop_start = time.monotonic()
                dt = loop_start - previous_time
                previous_time = loop_start

                motion = sensors.get_altitude_motion()
                imu_valid = True
                try:
                    vertical_accel_mps2 = (
                        vertical_acceleration_mps2(motion) - accel_bias_mps2
                    )
                except RuntimeError:
                    imu_valid = False
                    vertical_accel_mps2 = 0.0
                vertical_velocity_mps *= math.exp(
                    -dt / IMU_VELOCITY_DECAY_TIME_S
                )
                fused_altitude_m += (
                    vertical_velocity_mps * dt
                    + 0.5 * vertical_accel_mps2 * dt * dt
                )
                if imu_valid:
                    vertical_velocity_mps += vertical_accel_mps2 * dt

                pressure_updated = loop_start >= next_pressure_time
                if pressure_updated:
                    pressure_hpa = sensors.get_environment()["pressure_hpa"]
                    baro_altitude_raw_m = relative_altitude_m(
                        reference_pressure_hpa,
                        pressure_hpa,
                        air_temperature_c,
                    )
                    baro_altitude_history.append(baro_altitude_raw_m)
                    baro_altitude_m = median(baro_altitude_history)
                    baro_velocity_history.append((loop_start, baro_altitude_m))
                    while (
                        baro_velocity_history
                        and loop_start - baro_velocity_history[0][0]
                        > BARO_VELOCITY_WINDOW_S
                    ):
                        baro_velocity_history.popleft()
                    baro_regression_velocity_mps = regression_velocity_mps(
                        baro_velocity_history
                    )
                    pressure_dt = max(loop_start - previous_pressure_time, PRESSURE_INTERVAL_S)
                    previous_pressure_time = loop_start
                    altitude_error_m = baro_altitude_m - fused_altitude_m
                    fused_altitude_m += ALTITUDE_CORRECTION_GAIN * altitude_error_m
                    vertical_velocity_mps += (
                        velocity_correction_gain * altitude_error_m / pressure_dt
                    )
                    next_pressure_time += PRESSURE_INTERVAL_S
                    if next_pressure_time < loop_start:
                        next_pressure_time = loop_start + PRESSURE_INTERVAL_S

                linear_accel = motion["linear_accel_mps2"]
                gravity = motion["gravity_mps2"]
                elapsed_s = loop_start - start_time
                writer.writerow(
                    {
                        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                        "elapsed_s": f"{elapsed_s:.4f}",
                        "air_temperature_c": air_temperature_c,
                        "velocity_correction_gain": velocity_correction_gain,
                        "imu_velocity_decay_time_s": IMU_VELOCITY_DECAY_TIME_S,
                        "reference_pressure_hpa": f"{reference_pressure_hpa:.4f}",
                        "accel_bias_mps2": f"{accel_bias_mps2:.4f}",
                        "pressure_hpa": f"{pressure_hpa:.4f}",
                        "baro_altitude_raw_m": f"{baro_altitude_raw_m:.4f}",
                        "baro_altitude_m": f"{baro_altitude_m:.4f}",
                        "baro_regression_velocity_mps": (
                            f"{baro_regression_velocity_mps:.4f}"
                        ),
                        "fused_altitude_m": f"{fused_altitude_m:.4f}",
                        "vertical_velocity_mps": f"{vertical_velocity_mps:.4f}",
                        "vertical_accel_mps2": f"{vertical_accel_mps2:.4f}",
                        "linear_accel_x_mps2": linear_accel[0],
                        "linear_accel_y_mps2": linear_accel[1],
                        "linear_accel_z_mps2": linear_accel[2],
                        "gravity_x_mps2": gravity[0],
                        "gravity_y_mps2": gravity[1],
                        "gravity_z_mps2": gravity[2],
                        "calibration": motion["calibration"],
                        "imu_valid": int(imu_valid),
                        "pressure_updated": int(pressure_updated),
                    }
                )
                rows += 1

                if loop_start >= next_display_time:
                    print(
                        f"{elapsed_s:7.1f} s | "
                        f"気圧高度 {baro_altitude_m:8.2f} m | "
                        f"融合高度 {fused_altitude_m:8.2f} m | "
                        f"IMU短期速度 {vertical_velocity_mps:7.2f} m/s | "
                        f"気圧3秒速度 {baro_regression_velocity_mps:7.2f} m/s"
                    )
                    csv_file.flush()
                    next_display_time += DISPLAY_INTERVAL_S

                sleep_s = IMU_INTERVAL_S - (time.monotonic() - loop_start)
                if sleep_s > 0:
                    time.sleep(sleep_s)

    print(f"測定を終了しました。{rows}行を保存しました。")
    print(f"CSV: {output_path}")


if __name__ == "__main__":
    main()
