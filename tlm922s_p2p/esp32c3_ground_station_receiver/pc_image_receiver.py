#!/usr/bin/env python3
"""ESP32-C3のUSBシリアル出力から画像パケットを集めてJPEG保存する。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from communication_manager import ImageReceiveStore  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive and save JPEG images from the ESP32-C3 ground station.")
    parser.add_argument("--port", default="COM4", help="ESP32-C3 serial port, for example COM4 or /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--output-dir", default="received_images")
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read lines from standard input instead of opening a serial port",
    )
    return parser.parse_args()


def serial_lines(port: str, baudrate: int) -> Iterable[str]:
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required for --port mode. Install it with: python -m pip install pyserial") from exc

    with serial.Serial(port, baudrate, timeout=1) as ser:
        while True:
            line = ser.readline().decode("utf-8", errors="replace")
            if line:
                yield line.strip()


def stdin_lines() -> Iterable[str]:
    for line in sys.stdin:
        yield line.strip()


def main() -> int:
    args = parse_args()

    store = ImageReceiveStore(args.output_dir)
    lines = stdin_lines() if args.stdin else serial_lines(args.port, args.baudrate)

    print(f"Saving received images to: {Path(args.output_dir).resolve()}")
    for line in lines:
        result = store.add_line(line)
        if result is None:
            continue

        if result.error is not None:
            print(f"ignored inconsistent packet for file_id={result.file_id:08x}: {result.error}")
            continue

        print(
            f"image {result.file_id:08x}: "
            f"{result.collected}/{result.required} packets collected "
            f"(received index {result.received_index + 1}/{result.total_packets})"
        )

        if result.saved_path is not None:
            print(f"saved {result.saved_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
