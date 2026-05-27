#!/usr/bin/env python3
"""ESP32-C3のUSBシリアル出力を監視し、通信ログとJPEG画像を保存する。"""

from __future__ import annotations

import argparse
import sys
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path
from typing import Iterable, TextIO


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from communication_manager import ImageReceiveStore  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor ESP32-C3 ground station serial output.")
    parser.add_argument("--port", default="COM4", help="ESP32-C3 serial port, for example COM4 or /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--image-dir", default="received_images", help="directory for recovered JPEG images")
    parser.add_argument("--log-dir", default="ground_station_logs", help="directory for communication logs")
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read lines from standard input instead of opening a serial port",
    )
    parser.add_argument("--quiet", action="store_true", help="do not echo raw serial lines to the console")
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


def timestamp_for_filename() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def timestamp_for_log() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def write_log(log_file: TextIO, line: str) -> None:
    log_file.write(f"{timestamp_for_log()} {line}\n")
    log_file.flush()


def main() -> int:
    args = parse_args()

    image_dir = Path(args.image_dir)
    log_dir = Path(args.log_dir)
    store = ImageReceiveStore(image_dir)
    lines = stdin_lines() if args.stdin else serial_lines(args.port, args.baudrate)

    print(f"Saving received images to: {image_dir.resolve()}")
    print(f"Saving communication logs to: {log_dir.resolve()}")
    stamp = timestamp_for_filename()
    raw_path = log_dir / f"raw_serial_{stamp}.log"
    text_path = log_dir / f"non_image_{stamp}.log"
    image_path = log_dir / f"image_transfer_{stamp}.log"
    log_dir.mkdir(parents=True, exist_ok=True)

    with ExitStack() as stack:
        raw_log = stack.enter_context(raw_path.open("a", encoding="utf-8"))
        text_log = stack.enter_context(text_path.open("a", encoding="utf-8"))
        image_log = stack.enter_context(image_path.open("a", encoding="utf-8"))
        for line in lines:
            if not line:
                continue

            write_log(raw_log, line)
            result = store.add_line(line)

            if result is None:
                write_log(text_log, line)
                if not args.quiet:
                    print(line)
                continue

            if result.error is not None:
                message = f"image {result.file_id:08x}: ignored packet: {result.error}"
                write_log(image_log, message)
                print(message)
                continue

            message = (
                f"image {result.file_id:08x}: "
                f"{result.collected}/{result.required} packets collected "
                f"(received index {result.received_index + 1}/{result.total_packets})"
            )
            write_log(image_log, message)
            print(message)

            if result.saved_path is not None:
                saved_message = f"saved {result.saved_path}"
                write_log(image_log, saved_message)
                print(saved_message)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
