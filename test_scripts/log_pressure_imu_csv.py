#!/usr/bin/env python3
"""Log BME280 pressure and BNO055 IMU values to a CSV file every 0.1 seconds."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import sys
import time
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor_manager import BME280_ADDR, SensorManager


DEFAULT_INTERVAL_S = 0.1
DEFAULT_LOG_DIR = PROJECT_ROOT / "sensor_csv_logs"

CSV_FIELDS = [
    "timestamp",
    "elapsed_s",
    "temperature_c",
    "pressure_hpa",
    "humidity_percent",
    "heading_deg",
    "roll_deg",
    "pitch_deg",
    "accel_x_mps2",
    "accel_y_mps2",
    "accel_z_mps2",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "calibration",
    "error",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Log pressure and 9-axis sensor values to CSV at a fixed interval."
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="CSV output path. Default: sensor_csv_logs/pressure_imu_YYYYMMDD_HHMMSS.csv",
    )
    parser.add_argument(
        "-i",
        "--interval",
        type=float,
        default=DEFAULT_INTERVAL_S,
        help=f"Logging interval in seconds. Default: {DEFAULT_INTERVAL_S}",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        help="Optional logging duration in seconds. If omitted, logs until Ctrl+C.",
    )
    return parser.parse_args()


def make_output_path(path: Path | None) -> Path:
    if path is not None:
        return path
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return DEFAULT_LOG_DIR / f"pressure_imu_{timestamp}.csv"


def setup_sensors(sensors: SensorManager) -> None:
    sensors.environment.setup()
    # Use a short BME280 standby time so pressure can update during 0.1 s logging.
    sensors.bus.write_byte_data(BME280_ADDR, 0xF5, 0x20)
    sensors.imu.setup()


def empty_row(start_time: float) -> dict[str, Any]:
    return {
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "elapsed_s": f"{time.monotonic() - start_time:.3f}",
        "temperature_c": "",
        "pressure_hpa": "",
        "humidity_percent": "",
        "heading_deg": "",
        "roll_deg": "",
        "pitch_deg": "",
        "accel_x_mps2": "",
        "accel_y_mps2": "",
        "accel_z_mps2": "",
        "gyro_x_dps": "",
        "gyro_y_dps": "",
        "gyro_z_dps": "",
        "calibration": "",
        "error": "",
    }


def read_row(sensors: SensorManager, start_time: float) -> dict[str, Any]:
    row = empty_row(start_time)
    errors: list[str] = []

    try:
        env = sensors.get_environment()
        row["temperature_c"] = env.get("temperature_c", "")
        row["pressure_hpa"] = env.get("pressure_hpa", "")
        row["humidity_percent"] = env.get("humidity_percent", "")
    except Exception as exc:
        errors.append(f"BME280 {type(exc).__name__}: {exc}")

    try:
        imu = sensors.get_imu()
        accel = imu.get("accel_mps2") or ("", "", "")
        gyro = imu.get("gyro_dps") or ("", "", "")
        row["heading_deg"] = imu.get("heading_deg", "")
        row["roll_deg"] = imu.get("roll_deg", "")
        row["pitch_deg"] = imu.get("pitch_deg", "")
        row["accel_x_mps2"], row["accel_y_mps2"], row["accel_z_mps2"] = accel[:3]
        row["gyro_x_dps"], row["gyro_y_dps"], row["gyro_z_dps"] = gyro[:3]
        row["calibration"] = imu.get("calibration", "")
    except Exception as exc:
        errors.append(f"BNO055 {type(exc).__name__}: {exc}")

    row["error"] = " | ".join(errors)
    return row


def main() -> None:
    args = parse_args()
    output_path = make_output_path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("Pressure + 9-axis CSV logger")
    print(f"Interval: {args.interval:.3f} s")
    print(f"Output: {output_path}")
    input("Press Enter to start logging...")

    with SensorManager() as sensors:
        print("Setting up BME280 and BNO055...")
        setup_sensors(sensors)
        print("Logging started. Press Ctrl+C to stop.")

        start_time = time.monotonic()
        next_sample_time = start_time
        rows = 0

        try:
            with output_path.open("w", newline="", encoding="utf-8-sig") as file:
                writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
                writer.writeheader()

                while True:
                    now = time.monotonic()
                    if args.duration is not None and now - start_time >= args.duration:
                        break

                    if now < next_sample_time:
                        time.sleep(next_sample_time - now)

                    writer.writerow(read_row(sensors, start_time))
                    file.flush()
                    rows += 1
                    next_sample_time += args.interval

                    if next_sample_time < time.monotonic():
                        next_sample_time = time.monotonic()

        except KeyboardInterrupt:
            print("\nLogging stopped by Ctrl+C.")

    print(f"Saved {rows} rows to {output_path}")


if __name__ == "__main__":
    main()
