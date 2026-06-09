#!/usr/bin/env python3

from __future__ import annotations

import argparse

from pathlib import Path
import sys
import time


# リポジトリ直下を読み込む。
sys.path.append(str(Path(__file__).resolve().parents[1]))

from communication_manager import CommunicationManager


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send typed English text with TLM922S P2P.")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=19200)
    parser.add_argument("--interval", type=float, default=10, help="seconds to wait after each send")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    
    with CommunicationManager(port=args.port, baudrate=args.baudrate) as comm:
        text = "test"
        while True:
            response = comm.send_text(text)
            print(f"sent seq={comm.sequence} message={text}")
            print(response.replace("\r", "\n").strip())
            time.sleep(args.interval)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
