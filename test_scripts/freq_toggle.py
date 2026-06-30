#!/usr/bin/env python3
"""Toggle the TLM922S P2P frequency between two test values."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from communication_manager import Tlm922sUart


PRIMARY_FREQUENCY = 922500000
SECONDARY_FREQUENCY = 923200000


def ok_response(text: str) -> bool:
    return ">> Ok" in text


def parse_frequency(text: str) -> int | None:
    matches = re.findall(r"\b\d{6,}\b", text)
    if not matches:
        return None

    return int(matches[-1])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Toggle TLM922S P2P frequency.")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--primary", type=int, default=PRIMARY_FREQUENCY)
    parser.add_argument("--secondary", type=int, default=SECONDARY_FREQUENCY)
    parser.add_argument("--save", action="store_true", help="save the changed frequency to flash")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    with Tlm922sUart(args.port, args.baudrate) as radio:
        response = radio.command("p2p get_freq")
        current_frequency = parse_frequency(response)
        if current_frequency is None:
            print("ERROR: Could not read the current frequency.")
            return 1

        next_frequency = args.secondary if current_frequency == args.primary else args.primary
        response = radio.command(f"p2p set_freq {next_frequency}")
        if not ok_response(response):
            print("ERROR: Frequency change was not accepted.")
            return 1

        if args.save:
            response = radio.command("p2p save")
            if not ok_response(response):
                print("ERROR: Frequency was changed but save was not accepted.")
                return 1

    print(f"Frequency switched from {current_frequency} Hz to {next_frequency} Hz.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
