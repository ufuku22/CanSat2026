#!/usr/bin/env python3

from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sensor_manager import SensorManager


COMMANDS = [
    "PAIR050,1000",
    "PAIR062,0,1",
    "PAIR062,4,1",
]


def main() -> None:
    with SensorManager() as sensors:
        sensors.gnss.setup()
        for command in COMMANDS:
            sensors.gnss.write_nmea_command(command)
            print(f"sent ${command}")
            time.sleep(0.2)


if __name__ == "__main__":
    main()
