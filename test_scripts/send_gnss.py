#!/usr/bin/env python3
"""Read GNSS/environment data and send it through TLM922S every 10 seconds."""

from __future__ import annotations

from pathlib import Path
import sys
import time


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from communication_manager import CommunicationManager
from sensor_manager import BME280, I2C_BUS, LC76G, SMBus


class QuietLogger:
    def event(self, message: str) -> None:
        return None


def main() -> None:
    if SMBus is None:
        raise SystemExit("smbus2 or smbus is required on Raspberry Pi.")

    bus = SMBus(I2C_BUS)
    environment = BME280(bus)
    gnss = LC76G(bus)
    environment.setup()
    gnss.setup()

    try:
        with CommunicationManager(logger=QuietLogger()) as comm:
            while True:
                gnss_data = gnss.read()
                env_data = environment.read()
                response = comm.send_telemetry({"gnss": gnss_data, "environment": env_data})
                ok = "OK" if "radio_tx_ok" in response else "NO radio_tx_ok"
                print(
                    f"{ok} lat={gnss_data.get('latitude_deg')} lon={gnss_data.get('longitude_deg')} "
                    f"alt={gnss_data.get('altitude_m')} sat={gnss_data.get('satellites')} "
                    f"fix={gnss_data.get('fix_quality')} temp={env_data.get('temperature_c')} "
                    f"press={env_data.get('pressure_hpa')} hum={env_data.get('humidity_percent')}"
                )
                time.sleep(10)
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        if hasattr(bus, "close"):
            bus.close()


if __name__ == "__main__":
    main()
