#!/usr/bin/env python3
"""Simple BME280 connection test for Raspberry Pi."""

from __future__ import annotations

import argparse
import sys

from bme280_reader import BME280


def main() -> int:
    parser = argparse.ArgumentParser(description="Check whether one BME280 measurement can be read.")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number.")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x76, help="I2C address, such as 0x76 or 0x77.")
    args = parser.parse_args()

    sensor = BME280(bus=args.bus, address=args.address)
    try:
        values = sensor.read()
    except Exception as exc:
        print("BME280 test: FAILED")
        print(f"Reason: {exc}")
        return 1
    finally:
        sensor.close()

    print("BME280 test: OK")
    print(f"Temperature: {values['temperature_c']:.2f} C")
    print(f"Pressure:    {values['pressure_hpa']:.2f} hPa")
    print(f"Humidity:    {values['humidity_percent']:.2f} %")
    return 0


if __name__ == "__main__":
    sys.exit(main())
