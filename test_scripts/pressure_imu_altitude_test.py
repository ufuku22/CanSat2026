#!/usr/bin/env python3
"""BME280とBNO055を融合して相対高度をCSVへ記録する。"""

from __future__ import annotations

import csv
from datetime import datetime
import math
from pathlib import Path
import select
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
VELOCITY_CORRECTION_GAIN = 0.005
DRY_AIR_GAS_CONSTANT = 287.05  # J/(kg*K)
GRAVITY_MPS2 = 9.80665
LOG_DIR = Path(__file__).resolve().parent / "altitude_csv_logs"

CSV_FIELDS = [
    "timestamp",
    "elapsed_s",
    "air_temperature_c",
    "reference_pressure_hpa",
    "accel_bias_mps2",
    "pressure_hpa",
    "baro_altitude_m",
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
    if gravity_norm < 1.0:
        raise RuntimeError("BNO055の重力ベクトルを取得できません。")
    return sum(a * g for a, g in zip(linear_accel, gravity)) / gravity_norm


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
        vertical_accels.append(vertical_acceleration_mps2(motion))
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
        print("測定を開始します。Enterキーで終了します。")

        start_time = time.monotonic()
        previous_time = start_time
        previous_pressure_time = start_time
        next_pressure_time = start_time
        next_display_time = start_time
        pressure_hpa = reference_pressure_hpa
        baro_altitude_m = 0.0
        fused_altitude_m = 0.0
        vertical_velocity_mps = 0.0
        rows = 0

        with output_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()

            while not enter_pressed():
                loop_start = time.monotonic()
                dt = loop_start - previous_time
                previous_time = loop_start

                motion = sensors.get_altitude_motion()
                vertical_accel_mps2 = (
                    vertical_acceleration_mps2(motion) - accel_bias_mps2
                )
                fused_altitude_m += (
                    vertical_velocity_mps * dt
                    + 0.5 * vertical_accel_mps2 * dt * dt
                )
                vertical_velocity_mps += vertical_accel_mps2 * dt

                pressure_updated = loop_start >= next_pressure_time
                if pressure_updated:
                    pressure_hpa = sensors.get_environment()["pressure_hpa"]
                    baro_altitude_m = relative_altitude_m(
                        reference_pressure_hpa,
                        pressure_hpa,
                        air_temperature_c,
                    )
                    pressure_dt = max(loop_start - previous_pressure_time, PRESSURE_INTERVAL_S)
                    previous_pressure_time = loop_start
                    altitude_error_m = baro_altitude_m - fused_altitude_m
                    fused_altitude_m += ALTITUDE_CORRECTION_GAIN * altitude_error_m
                    vertical_velocity_mps += (
                        VELOCITY_CORRECTION_GAIN * altitude_error_m / pressure_dt
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
                        "reference_pressure_hpa": f"{reference_pressure_hpa:.4f}",
                        "accel_bias_mps2": f"{accel_bias_mps2:.4f}",
                        "pressure_hpa": f"{pressure_hpa:.4f}",
                        "baro_altitude_m": f"{baro_altitude_m:.4f}",
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
                        "pressure_updated": int(pressure_updated),
                    }
                )
                rows += 1

                if loop_start >= next_display_time:
                    print(
                        f"{elapsed_s:7.1f} s | "
                        f"気圧高度 {baro_altitude_m:8.2f} m | "
                        f"融合高度 {fused_altitude_m:8.2f} m | "
                        f"鉛直速度 {vertical_velocity_mps:7.2f} m/s"
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
