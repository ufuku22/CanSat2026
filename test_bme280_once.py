#!/usr/bin/env python3
"""Simple BME280 connection test for Raspberry Pi."""

from __future__ import annotations

import argparse
import sys
import time

from smbus2 import SMBus


BME280_CHIP_ID = 0x60
REG_CHIP_ID = 0xD0
REG_CTRL_HUM = 0xF2
REG_CTRL_MEAS = 0xF4
REG_CONFIG = 0xF5
REG_DATA = 0xF7


def read_signed_16(data: list[int], index: int) -> int:
    value = data[index] | (data[index + 1] << 8)
    if value >= 32768:
        value -= 65536
    return value


def read_unsigned_16(data: list[int], index: int) -> int:
    return data[index] | (data[index + 1] << 8)


def main() -> int:
    parser = argparse.ArgumentParser(description="Check whether one BME280 measurement can be read.")
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x76)
    args = parser.parse_args()

    try:
        bus = SMBus(args.bus)
    except Exception as exc:
        print("BME280 test: FAILED")
        print(f"Could not open I2C bus {args.bus}: {exc}")
        return 1

    try:
        chip_id = bus.read_byte_data(args.address, REG_CHIP_ID)
        if chip_id != BME280_CHIP_ID:
            print("BME280 test: FAILED")
            print(f"Unexpected chip id: 0x{chip_id:02X}")
            print("Check the I2C address. BME280 is usually 0x76 or 0x77.")
            return 1

        calib = bus.read_i2c_block_data(args.address, 0x88, 24)
        dig_t1 = read_unsigned_16(calib, 0)
        dig_t2 = read_signed_16(calib, 2)
        dig_t3 = read_signed_16(calib, 4)
        dig_p1 = read_unsigned_16(calib, 6)

        bus.write_byte_data(args.address, REG_CTRL_HUM, 0x01)
        bus.write_byte_data(args.address, REG_CONFIG, 0xA0)
        bus.write_byte_data(args.address, REG_CTRL_MEAS, 0x27)
        time.sleep(0.1)

        data = bus.read_i2c_block_data(args.address, REG_DATA, 8)
        raw_pressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        raw_temperature = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        raw_humidity = (data[6] << 8) | data[7]

        var1 = (((raw_temperature >> 3) - (dig_t1 << 1)) * dig_t2) >> 11
        var2 = (((((raw_temperature >> 4) - dig_t1) * ((raw_temperature >> 4) - dig_t1)) >> 12) * dig_t3) >> 14
        temperature_c = (((var1 + var2) * 5 + 128) >> 8) / 100.0

        print("BME280 test: OK")
        print(f"Chip ID:         0x{chip_id:02X}")
        print(f"Temperature:     {temperature_c:.2f} C")
        print(f"Raw pressure:    {raw_pressure}")
        print(f"Raw humidity:    {raw_humidity}")
        print(f"Pressure calib:  {dig_p1}")
        return 0
    except Exception as exc:
        print("BME280 test: FAILED")
        print(f"Reason: {exc}")
        return 1
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
