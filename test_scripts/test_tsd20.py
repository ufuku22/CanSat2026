#!/usr/bin/env python3
"""TSD20 single-unit test for CanSat2026."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


sys.path.append(str(Path(__file__).resolve().parents[1]))

from sensor_manager import I2C_BUS, SMBus, TSD20, TSD20_ADDR  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read distance from TSD20 LiDAR.")
    parser.add_argument("--bus", type=int, default=I2C_BUS, help="I2C bus number")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=TSD20_ADDR, help="I2C address")
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
        print(f"TSD20 test: bus={args.bus}, address=0x{args.address:02X}")
        sensor = TSD20(bus, address=args.address)

        print("TSD20 setup start")
        sensor.setup()
        print("TSD20 setup OK")

        for index in range(args.count):
            print(f"TSD20 read {index + 1}/{args.count} start")
            distance_m = sensor.read_m()
            print(f"distance_m={distance_m}")
            if index + 1 < args.count:
                time.sleep(args.interval)

        print("TSD20 test OK")
        return 0
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        if hasattr(bus, "close"):
            bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
