#!/usr/bin/env python3
"""Send a simple text packet through CommunicationManager."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import tempfile


sys.path.append(str(Path(__file__).resolve().parents[1]))

from communication_manager import CommunicationManager


STATE_FILE = Path(tempfile.gettempdir()) / "cansat2026_dummy_communication_enabled"


def communication_enabled() -> bool:
    if not STATE_FILE.exists():
        return True

    return STATE_FILE.read_text(encoding="utf-8").strip().lower() != "disabled"


class QuietLogger:
    def event(self, message: str) -> None:
        return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send a simple text packet with TLM922S P2P.")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--message", default="CanSat2026 test")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not communication_enabled():
        print("Error: communication is currently unavailable.", file=sys.stderr)
        return 1

    with CommunicationManager(port=args.port, baudrate=args.baudrate, logger=QuietLogger()) as comm:
        response = comm.send_text(args.message)

    if "radio_tx_ok" not in response:
        print("Error: radio did not confirm transmission.", file=sys.stderr)
        if response.strip():
            print(response.strip(), file=sys.stderr)
        return 1

    print(args.message)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
