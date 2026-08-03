#!/usr/bin/env python3
"""Send typed text through TLM922S P2P for quick communication tests."""

from __future__ import annotations

import argparse

from pathlib import Path
import sys
import time


# リポジトリ直下を読み込む。
sys.path.append(str(Path(__file__).resolve().parents[1]))

from communication_manager import CommunicationManager
from config import CommunicationConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send typed English text with TLM922S P2P.")
    parser.add_argument("--port", default=CommunicationConfig.UART_PORT)
    parser.add_argument(
        "--baudrate", type=int, default=CommunicationConfig.UART_BAUDRATE
    )
    parser.add_argument("--interval", type=float, default=0.5, help="seconds to wait after each send")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    with CommunicationManager(port=args.port, baudrate=args.baudrate) as comm:
        print("Type English text and press Enter to send. Empty line exits.")
        while True:
            message = input("> ").strip()
            if not message:
                break

            comm.send_text(message)
            if args.interval > 0:
                time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
