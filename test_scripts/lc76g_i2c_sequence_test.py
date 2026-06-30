#!/usr/bin/env python3
"""Low-level LC76G I2C sequence diagnostic.

This follows the Quectel I2C application note addresses directly:
0x50 for command, 0x54 for read data, and 0x58 for write data.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor_manager import I2C_BUS, SMBus, i2c_msg


CMD_ADDR = 0x50
READ_ADDR = 0x54
WRITE_ADDR = 0x58


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run LC76G I2C application-note sequence checks.")
    parser.add_argument("--delay", type=float, default=0.05, help="delay after command writes")
    parser.add_argument("--read-len", type=int, default=128, help="NMEA bytes to read after length check")
    return parser.parse_args()


def raw_write(bus, address: int, data: list[int], label: str) -> bool:
    print(f"{label}: write {len(data)} bytes to 0x{address:02X}: {bytes(data).hex(' ')}")
    try:
        if i2c_msg is not None and hasattr(bus, "i2c_rdwr"):
            bus.i2c_rdwr(i2c_msg.write(address, data))
        else:
            bus.write_i2c_block_data(address, data[0], data[1:])
    except OSError as exc:
        print(f"{label}: NG {type(exc).__name__}: {exc}")
        return False
    print(f"{label}: OK")
    return True


def raw_read(bus, address: int, length: int, label: str) -> bytes | None:
    print(f"{label}: read {length} bytes from 0x{address:02X}")
    try:
        if i2c_msg is not None and hasattr(bus, "i2c_rdwr"):
            msg = i2c_msg.read(address, length)
            bus.i2c_rdwr(msg)
            data = bytes(msg)
        else:
            data = bytes(bus.read_i2c_block_data(address, 0x00, length))
    except OSError as exc:
        print(f"{label}: NG {type(exc).__name__}: {exc}")
        return None
    print(f"{label}: OK {data.hex(' ')}")
    return data


def le_words(word1: int, word2: int) -> list[int]:
    return list(word1.to_bytes(4, "little") + word2.to_bytes(4, "little"))


def main() -> int:
    args = parse_args()
    if SMBus is None:
        raise SystemExit("smbus2 or smbus is required on Raspberry Pi.")

    bus = SMBus(I2C_BUS)
    try:
        print("=== LC76G I2C sequence test ===")
        print("Expected application-note addresses: cmd=0x50 read=0x54 write=0x58")

        if not raw_write(bus, CMD_ADDR, le_words(0xAA510008, 4), "step1 length command"):
            return 1
        time.sleep(args.delay)

        length_data = raw_read(bus, READ_ADDR, 4, "step2 length read")
        if length_data is None:
            return 2

        nmea_len = int.from_bytes(length_data, "little")
        print(f"NMEA length from module: {nmea_len}")
        if nmea_len <= 0:
            print("No NMEA bytes are waiting. I2C sequence worked, but buffer is empty.")
            return 0

        read_len = min(nmea_len, max(1, args.read_len))
        if not raw_write(bus, CMD_ADDR, le_words(0xAA512000, read_len), "step3 NMEA command"):
            return 3
        time.sleep(args.delay)

        nmea_data = raw_read(bus, READ_ADDR, read_len, "step4 NMEA read")
        if nmea_data is None:
            return 4

        text = nmea_data.decode("ascii", errors="replace").replace("\x00", "")
        print("NMEA text:")
        print(text.rstrip())
        return 0
    finally:
        if hasattr(bus, "close"):
            bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
