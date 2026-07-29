#!/usr/bin/env python3
"""Send typed text through TLM922S P2P for quick communication tests."""

from __future__ import annotations

import argparse

from pathlib import Path
import sys
import time


# リポジトリ直下を読み込む。
sys.path.append(str(Path(__file__).resolve().parents[1]))

from communication_manager import CommunicationManager, SerialPortInUseError


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send typed English text with TLM922S P2P.")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--interval", type=float, default=0.5, help="seconds to wait after each send")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        comm = CommunicationManager(port=args.port, baudrate=args.baudrate)
        print("Type English text and press Enter to send. Empty line exits.")
        while True:
            message = input("> ").strip()
            if not message:
                break

            # 入力待ち中はUARTを解放し、実際に送信する間だけ排他取得する。
            with comm:
                comm.send_text(message)
            if args.interval > 0:
                time.sleep(args.interval)
    except SerialPortInUseError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\nStopped.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
