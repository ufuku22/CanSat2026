#!/usr/bin/env python3
"""BME280とBNO055を融合して相対高度をCSVへ記録する。"""

from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
import select
import sys
import time


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from altitude_estimator import (
    AltitudeEstimator,
    IMU_INTERVAL_S,
    IMU_VELOCITY_DECAY_TIME_S,
    calibrate_altitude,
    configure_bme280_for_altitude,
)
from sensor_manager import SensorManager


DISPLAY_INTERVAL_S = 1.0
LOG_DIR = Path(__file__).resolve().parent / "altitude_csv_logs"

CSV_FIELDS = [
    "timestamp",
    "elapsed_s",
    "air_temperature_c",
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


def enter_pressed() -> bool:
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False
    sys.stdin.readline()
    return True


def main() -> None:
    air_temperature_c = input_air_temperature_c()
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    output_path = LOG_DIR / f"altitude_{datetime.now():%Y%m%d_%H%M%S}.csv"

    with SensorManager() as sensors:
        configure_bme280_for_altitude(sensors)
        sensors.imu.setup()
        time.sleep(1.0)
        reference_pressure_hpa, accel_bias_mps2, calibration = calibrate_altitude(
            sensors
        )

        print(f"基準気圧: {reference_pressure_hpa:.2f} hPa = 0.00 m")
        print(f"鉛直加速度オフセット: {accel_bias_mps2:+.4f} m/s²")
        print(f"BNO055校正値: 0x{calibration:02X}")
        print("測定を開始します。Enterキーで終了します。")

        start_time = time.monotonic()
        next_display_time = start_time
        estimator = AltitudeEstimator(
            sensors,
            air_temperature_c,
            reference_pressure_hpa,
            accel_bias_mps2,
        )
        rows = 0

        with output_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()

            while not enter_pressed():
                loop_start = time.monotonic()
                estimate = estimator.update(loop_start)
                motion = estimate.motion
                if motion is None:
                    raise RuntimeError("BNO055の測定値を取得できませんでした。")
                linear_accel = motion["linear_accel_mps2"]
                gravity = motion["gravity_mps2"]
                elapsed_s = loop_start - start_time
                writer.writerow(
                    {
                        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                        "elapsed_s": f"{elapsed_s:.4f}",
                        "air_temperature_c": air_temperature_c,
                        "imu_velocity_decay_time_s": IMU_VELOCITY_DECAY_TIME_S,
                        "reference_pressure_hpa": f"{reference_pressure_hpa:.4f}",
                        "accel_bias_mps2": f"{accel_bias_mps2:.4f}",
                        "pressure_hpa": f"{estimate.pressure_hpa:.4f}",
                        "baro_altitude_raw_m": (
                            f"{estimate.baro_altitude_raw_m:.4f}"
                        ),
                        "baro_altitude_m": f"{estimate.baro_altitude_m:.4f}",
                        "baro_regression_velocity_mps": (
                            f"{estimate.baro_regression_velocity_mps:.4f}"
                        ),
                        "fused_altitude_m": f"{estimate.fused_altitude_m:.4f}",
                        "vertical_velocity_mps": (
                            f"{estimate.vertical_velocity_mps:.4f}"
                        ),
                        "vertical_accel_mps2": (
                            f"{estimate.vertical_accel_mps2:.4f}"
                        ),
                        "linear_accel_x_mps2": linear_accel[0],
                        "linear_accel_y_mps2": linear_accel[1],
                        "linear_accel_z_mps2": linear_accel[2],
                        "gravity_x_mps2": gravity[0],
                        "gravity_y_mps2": gravity[1],
                        "gravity_z_mps2": gravity[2],
                        "calibration": motion["calibration"],
                        "imu_valid": int(estimate.imu_valid),
                        "pressure_updated": int(estimate.pressure_updated),
                    }
                )
                rows += 1

                if loop_start >= next_display_time:
                    print(
                        f"{elapsed_s:7.1f} s | "
                        f"気圧高度 {estimate.baro_altitude_m:8.2f} m | "
                        f"融合高度 {estimate.fused_altitude_m:8.2f} m | "
                        f"IMU短期速度 {estimate.vertical_velocity_mps:7.2f} m/s | "
                        "気圧3秒速度 "
                        f"{estimate.baro_regression_velocity_mps:7.2f} m/s"
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
