#!/usr/bin/env python3
"""Send dummy binary-like data through CommunicationManager."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import tempfile


sys.path.append(str(Path(__file__).resolve().parents[1]))

from communication_manager import DEFAULT_MAX_RADIO_PAYLOAD, CommunicationManager


STATE_FILE = Path(tempfile.gettempdir()) / "cansat2026_dummy_communication_enabled"


def communication_enabled() -> bool:
    if not STATE_FILE.exists():
        return True

    return STATE_FILE.read_text(encoding="utf-8").strip().lower() != "disabled"


def make_dummy_binary(length: int) -> bytes:
    return bytes(((index * 37 + 11) % 256 for index in range(length)))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send dummy binary-like data with TLM922S P2P.")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--length",
        type=int,
        default=8,
        help=f"dummy binary byte length, up to {DEFAULT_MAX_RADIO_PAYLOAD}",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not communication_enabled():
        print("Error: communication is currently unavailable.", file=sys.stderr)
        return 1

    if not 1 <= args.length <= DEFAULT_MAX_RADIO_PAYLOAD:
        print(f"Error: length must be between 1 and {DEFAULT_MAX_RADIO_PAYLOAD} bytes.", file=sys.stderr)
        return 1

    payload = make_dummy_binary(args.length)
    with CommunicationManager(port=args.port, baudrate=args.baudrate) as comm:
        if comm.radio is None:
            raise RuntimeError("CommunicationManager.setup() must be called before sending.")

        response = comm.radio.command(f"p2p tx {payload.hex()}", wait=comm.timeout, until="radio_tx_ok")

    if "radio_tx_ok" not in response:
        print("Error: radio did not confirm transmission.", file=sys.stderr)
        if response.strip():
            print(response.strip(), file=sys.stderr)
        return 1

    print(f"Sent {len(payload)} bytes of dummy binary data.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
