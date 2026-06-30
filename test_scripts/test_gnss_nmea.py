#!/usr/bin/env python3
"""LC76G GNSS NMEA diagnostic test.

This script reads raw NMEA from the LC76G through the existing sensor_manager
driver and explains why latitude/longitude are or are not available.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor_manager import I2C_BUS, LC76G, SMBus


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check LC76G raw NMEA and GNSS fix status.")
    parser.add_argument("--count", type=int, default=0, help="number of reads; 0 means run forever")
    parser.add_argument("--no-setup", action="store_true", help="skip LC76G setup commands")
    return parser.parse_args()


def nmea_kind(line: str) -> str:
    parts = line.split(",", 1)
    if not parts or not parts[0].startswith("$"):
        return ""
    return parts[0][-3:]


def strip_checksum(value: str) -> str:
    return value.split("*", 1)[0]


def explain_gga(parts: list[str]) -> dict[str, Any]:
    fix_quality = int_or_none(parts[6]) if len(parts) > 6 else None
    satellites = int_or_none(parts[7]) if len(parts) > 7 else None
    lat = parts[2] if len(parts) > 2 else ""
    lon = parts[4] if len(parts) > 4 else ""

    if fix_quality is None:
        status = "GGA has no fix_quality field."
    elif fix_quality == 0:
        status = "No fix yet: GGA fix_quality is 0."
    elif lat and lon:
        status = "Position fix available from GGA."
    else:
        status = "GGA says fix exists, but latitude/longitude fields are empty."

    return {
        "fix_quality": fix_quality,
        "satellites": satellites,
        "lat_field": lat,
        "lon_field": lon,
        "status": status,
    }


def explain_rmc(parts: list[str]) -> dict[str, Any]:
    status_field = parts[2] if len(parts) > 2 else ""
    lat = parts[3] if len(parts) > 3 else ""
    lon = parts[5] if len(parts) > 5 else ""

    if status_field == "A" and lat and lon:
        status = "Position fix available from RMC."
    elif status_field == "V":
        status = "No valid fix yet: RMC status is V."
    elif not status_field:
        status = "RMC has no status field."
    else:
        status = "RMC status is not valid, or latitude/longitude fields are empty."

    return {
        "status_field": status_field,
        "lat_field": lat,
        "lon_field": lon,
        "status": status,
    }


def analyze_raw(raw: str) -> dict[str, Any]:
    lines = [line.strip() for line in raw.splitlines() if line.strip()]
    result: dict[str, Any] = {
        "sentence_count": len(lines),
        "kinds": {},
        "gga": [],
        "rmc": [],
        "has_position": False,
        "summary": "",
    }

    for line in lines:
        kind = nmea_kind(line)
        if kind:
            result["kinds"][kind] = result["kinds"].get(kind, 0) + 1

        parts = line.split(",")
        if kind == "GGA":
            gga = explain_gga(parts)
            result["gga"].append(gga)
            result["has_position"] = result["has_position"] or (
                gga["fix_quality"] not in (None, 0) and bool(gga["lat_field"]) and bool(gga["lon_field"])
            )
        elif kind == "RMC":
            if parts:
                parts[-1] = strip_checksum(parts[-1])
            rmc = explain_rmc(parts)
            result["rmc"].append(rmc)
            result["has_position"] = result["has_position"] or (
                rmc["status_field"] == "A" and bool(rmc["lat_field"]) and bool(rmc["lon_field"])
            )

    if not raw:
        result["summary"] = "NMEA is empty. I2C read, LC76G power, wiring, or module startup is suspicious."
    elif result["has_position"]:
        result["summary"] = "GNSS position is available."
    elif result["gga"] or result["rmc"]:
        result["summary"] = "NMEA is readable, but GNSS has not fixed its position yet."
    else:
        result["summary"] = "NMEA is readable, but this read did not include GGA/RMC position sentences."

    return result


def int_or_none(value: str) -> int | None:
    try:
        return int(value) if value else None
    except ValueError:
        return None


def print_analysis(gnss_data: dict[str, Any], analysis: dict[str, Any]) -> None:
    print(f"parsed latitude_deg : {gnss_data.get('latitude_deg')}")
    print(f"parsed longitude_deg: {gnss_data.get('longitude_deg')}")
    print(f"parsed altitude_m   : {gnss_data.get('altitude_m')}")
    print(f"parsed satellites   : {gnss_data.get('satellites')}")
    print(f"parsed fix_quality  : {gnss_data.get('fix_quality')}")
    print(f"NMEA sentence count : {analysis['sentence_count']}")
    print(f"NMEA sentence types : {analysis['kinds']}")
    print(f"diagnosis           : {analysis['summary']}")

    for gga in analysis["gga"]:
        print(
            "GGA                 : "
            f"fix_quality={gga['fix_quality']} satellites={gga['satellites']} "
            f"lat_field={gga['lat_field'] or '-'} lon_field={gga['lon_field'] or '-'}"
        )
        print(f"                      {gga['status']}")

    for rmc in analysis["rmc"]:
        print(
            "RMC                 : "
            f"status={rmc['status_field'] or '-'} "
            f"lat_field={rmc['lat_field'] or '-'} lon_field={rmc['lon_field'] or '-'}"
        )
        print(f"                      {rmc['status']}")

def main() -> None:
    args = parse_args()
    if SMBus is None:
        raise SystemExit("smbus2 or smbus is required on Raspberry Pi.")

    bus = SMBus(I2C_BUS)
    gnss = LC76G(bus)

    try:
        if not args.no_setup:
            print("Sending LC76G setup commands...")
            gnss.setup()

        read_index = 0
        while args.count <= 0 or read_index < args.count:
            input("\nPress Enter to read GNSS NMEA...")
            read_index += 1
            print(f"\n--- GNSS read #{read_index} {time.strftime('%Y-%m-%d %H:%M:%S')} ---")
            gnss_data = gnss.read()
            analysis = analyze_raw(gnss_data.get("raw", ""))
            print_analysis(gnss_data, analysis)
    except KeyboardInterrupt:
        print("\nGNSS NMEA test stopped.")
    finally:
        if hasattr(bus, "close"):
            bus.close()


if __name__ == "__main__":
    main()
