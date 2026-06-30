#!/usr/bin/env python3
"""LC76G I2C sequence diagnostic through sensor_manager.

The LC76G driver owns the fragile 0x50 -> 0x54/0x58 sequencing. This script only
checks that those safe driver operations complete and shows the returned data.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor_manager import I2C_BUS, LC76G, SMBus


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check LC76G I2C driver sequencing.")
    parser.add_argument("--read-len", type=int, default=128, help="maximum NMEA bytes to print")
    parser.add_argument(
        "--command",
        default="",
        help="optional PAIR/PQTM command body to send through the driver",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if SMBus is None:
        raise SystemExit("smbus2 or smbus is required on Raspberry Pi.")

    bus = SMBus(I2C_BUS)
    gnss = LC76G(bus)
    try:
        print("=== LC76G I2C driver sequence test ===")
        nmea_length = gnss.available_nmea_length()
        write_free_length = gnss.write_free_length()
        print(f"available NMEA bytes : {nmea_length}")
        print(f"write buffer free    : {write_free_length}")

        if args.command:
            print(f"sending command      : {args.command}")
            gnss.write_nmea_command(args.command)
            print("command send         : OK")

        raw = gnss.read_nmea(max_length=args.read_len)
        if not raw:
            print("NMEA text            : empty")
            return 0

        print("NMEA text:")
        print(raw.rstrip())
        return 0
    finally:
        if hasattr(bus, "close"):
            bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
