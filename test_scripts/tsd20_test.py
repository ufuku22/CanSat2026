#!/usr/bin/env python3

from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor_manager import SensorManager


def main():
    with SensorManager() as sensors:
        sensors.distance.setup()
        while True:
            print(sensors.get_distance_m())
            time.sleep(1.0)


if __name__ == "__main__":
    main()
