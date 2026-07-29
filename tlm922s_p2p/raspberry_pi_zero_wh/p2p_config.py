#!/usr/bin/env python3
"""TLM922S交換・初期化時だけ使う手動設定スクリプト。"""

import argparse
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from communication_manager import Tlm922sUart


# 受信側と同じ値。通常の送信コードからは呼び出さない。
P2P_COMMANDS = [
    "p2p set_freq 922500000",
    "p2p set_pwr 20",
    "p2p set_sf 12",
    "p2p set_bw 125",
    "p2p set_cr 4/6",
    "p2p set_prlen 16",
    "p2p set_crc on",
    "p2p set_iqi off",
    "p2p set_sync 12",
]

CHECK_COMMANDS = [
    "p2p get_freq",
    "p2p get_pwr",
    "p2p get_sf",
    "p2p get_bw",
    "p2p get_cr",
    "p2p get_prlen",
    "p2p get_crc",
    "p2p get_iqi",
    "p2p get_sync",
]


def ok_response(text: str) -> bool:
    return ">> Ok" in text


def print_response(text: str) -> None:
    cleaned = text.replace("\r", "\n").strip()
    print(cleaned if cleaned else "(no response)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Manually configure and save TLM922S P2P settings."
    )
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--save", dest="save", action="store_true", default=True, help="save P2P settings to flash")
    parser.add_argument("--no-save", dest="save", action="store_false", help="do not save P2P settings to flash")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    with Tlm922sUart(args.port, args.baudrate) as radio:
        print("Configuring P2P parameters...")
        for command in P2P_COMMANDS:
            print(f"\n> {command}")
            response = radio.command(command)
            print_response(response)
            if not ok_response(response):
                print("ERROR: command was not accepted.")
                return 1

        if args.save:
            print("\n> p2p save")
            print_response(radio.command("p2p save"))

        print("\nCurrent P2P settings:")
        for command in CHECK_COMMANDS:
            print(f"\n> {command}")
            print_response(radio.command(command))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
