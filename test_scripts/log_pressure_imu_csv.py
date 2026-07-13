#!/usr/bin/env python3
"""Log BME280 pressure and BNO055 IMU values to a CSV file every 0.1 seconds."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from logger import PressureImuCsvLogger
from sensor_manager import SensorManager


DEFAULT_INTERVAL_S = 0.1
DEFAULT_LOG_DIR = PROJECT_ROOT / "sensor_csv_logs"

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
        PressureImuCsvLogger.setup_sensors(sensors)
        print("Logging started. Press Ctrl+C to stop.")

        start_time = time.monotonic()
        next_sample_time = start_time
        rows = 0

        try:
            with PressureImuCsvLogger(sensors, output_path) as csv_logger:
                while True:
                    now = time.monotonic()
                    if args.duration is not None and now - start_time >= args.duration:
                        break

                    if now < next_sample_time:
                        time.sleep(next_sample_time - now)

                    csv_logger.write_row()
                    rows += 1
                    next_sample_time += args.interval

                    if next_sample_time < time.monotonic():
                        next_sample_time = time.monotonic()

        except KeyboardInterrupt:
            print("\nLogging stopped by Ctrl+C.")

    print(f"Saved {rows} rows to {output_path}")


if __name__ == "__main__":
    main()
