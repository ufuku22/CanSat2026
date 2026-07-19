#!/usr/bin/env python3
"""Toggle dummy communication availability for send.py."""

from __future__ import annotations

import argparse
from pathlib import Path
import tempfile


STATE_FILE = Path(tempfile.gettempdir()) / "cansat2026_dummy_communication_enabled"


def current_mode() -> str:
    if not STATE_FILE.exists():
        return "enabled"

    mode = STATE_FILE.read_text(encoding="utf-8").strip().lower()
    return "disabled" if mode == "disabled" else "enabled"


def write_mode(mode: str) -> None:
    STATE_FILE.write_text(mode + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Toggle dummy communication availability.")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--enable", action="store_true", help="make send.py able to communicate")
    group.add_argument("--disable", action="store_true", help="make send.py report communication unavailable")
    group.add_argument("--status", action="store_true", help="show current dummy communication mode")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.status:
        mode = current_mode()
    elif args.enable:
        mode = "enabled"
        write_mode(mode)
    elif args.disable:
        mode = "disabled"
        write_mode(mode)
    else:
        mode = "disabled" if current_mode() == "enabled" else "enabled"
        write_mode(mode)

    print(f"Communication mode: {mode}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
