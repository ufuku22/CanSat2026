#!/usr/bin/env python3
"""Read GNSS/environment data and send it through TLM922S every 10 seconds."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
from pathlib import Path
import sys
import time


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from communication_manager import CommunicationManager
from sensor_manager import BME280, I2C_BUS, LC76G, SMBus


class QuietLogger:
    def event(self, message: str) -> None:
        return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read and optionally send GNSS/environment telemetry.")
    parser.add_argument("--interval", type=float, default=10.0, help="seconds between reads")
    parser.add_argument("--no-radio", action="store_true", help="read sensors but do not send telemetry")
    parser.add_argument("--no-env", action="store_true", help="skip BME280 setup/read")
    return parser.parse_args()


def format_status(ok: str, gnss_data: dict, env_data: dict | None) -> str:
    raw = gnss_data.get("raw", "")
    parts = [
        ok,
        f"lat={gnss_data.get('latitude_deg')}",
        f"lon={gnss_data.get('longitude_deg')}",
        f"alt={gnss_data.get('altitude_m')}",
        f"sat={gnss_data.get('satellites')}",
        f"fix={gnss_data.get('fix_quality')}",
        f"has_fix={gnss_data.get('has_fix')}",
        f"raw_len={len(raw)}",
        f"error={gnss_data.get('error')}",
    ]
    if env_data is not None:
        parts.extend(
            [
                f"temp={env_data.get('temperature_c')}",
                f"press={env_data.get('pressure_hpa')}",
                f"hum={env_data.get('humidity_percent')}",
            ]
        )
    return " ".join(parts)


def main() -> None:
    args = parse_args()
    if SMBus is None:
        raise SystemExit("smbus2 or smbus is required on Raspberry Pi.")

    bus = SMBus(I2C_BUS)
    environment = None if args.no_env else BME280(bus)
    gnss = LC76G(bus)
    if environment is not None:
        environment.setup()
    gnss.setup()

    try:
        comm_context = nullcontext(None) if args.no_radio else CommunicationManager(logger=QuietLogger())
        with comm_context as comm:
            while True:
                gnss_data = gnss.read()
                env_data = environment.read() if environment is not None else None

                if comm is None:
                    ok = "NO_RADIO"
                else:
                    telemetry = {"gnss": gnss_data}
                    if env_data is not None:
                        telemetry["environment"] = env_data
                    response = comm.send_telemetry(telemetry)
                    ok = "OK" if "radio_tx_ok" in response else "NO radio_tx_ok"

                print(format_status(ok, gnss_data, env_data))
                time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        if hasattr(bus, "close"):
            bus.close()


if __name__ == "__main__":
    main()
