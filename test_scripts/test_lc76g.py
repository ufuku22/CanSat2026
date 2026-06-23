#!/usr/bin/env python3

from pathlib import Path
import json
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sensor_manager import SensorManager


INTERVAL_SECONDS = 1.0


def main() -> None:
    with SensorManager() as sensors:
        sensors.gnss.setup()
        print("LC76G GPS test start. Stop with Ctrl+C.")
        while True:
            print(json.dumps(sensors.get_gnss(), indent=2, ensure_ascii=False))
            time.sleep(INTERVAL_SECONDS)


if __name__ == "__main__":
    main()
