#!/usr/bin/env python3
"""Send typed text through TLM922S P2P for quick communication tests."""

from __future__ import annotations

import argparse

from communication_manager import CommunicationManager


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send typed English text with TLM922S P2P.")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=115200)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    with CommunicationManager(port=args.port, baudrate=args.baudrate) as comm:
        print("Type English text and press Enter to send. Empty line exits.")
        while True:
            message = input("> ").strip()
            if not message:
                break

            response = comm.send_text(message)
            print(response.replace("\r", "\n").strip())

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
