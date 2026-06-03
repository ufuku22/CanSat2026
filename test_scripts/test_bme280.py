#!/usr/bin/env python3
"""BME280 single-unit test for CanSat2026."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


sys.path.append(str(Path(__file__).resolve().parents[1]))

from sensor_manager import BME280, BME280_ADDR, I2C_BUS, SMBus  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read temperature, pressure, and humidity from BME280.")
    parser.add_argument("--bus", type=int, default=I2C_BUS, help="I2C bus number")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=BME280_ADDR, help="I2C address")
    parser.add_argument("--count", type=int, default=1, help="number of reads")
    parser.add_argument("--interval", type=float, default=1.0, help="seconds between reads")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if SMBus is None:
        print("ERROR: smbus2 or smbus is required on Raspberry Pi.", file=sys.stderr)
        return 2

    bus = SMBus(args.bus)
    try:
        print(f"BME280 test: bus={args.bus}, address=0x{args.address:02X}")
        sensor = BME280(bus, address=args.address)

        print("BME280 setup start")
        sensor.setup()
        print("BME280 setup OK")

        for index in range(args.count):
            print(f"BME280 read {index + 1}/{args.count} start")
            data = sensor.read()
            print(json.dumps(data, ensure_ascii=False))
            if index + 1 < args.count:
                time.sleep(args.interval)

        print("BME280 test OK")
        return 0
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        if hasattr(bus, "close"):
            bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
