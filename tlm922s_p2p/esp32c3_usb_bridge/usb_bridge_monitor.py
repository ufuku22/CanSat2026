#!/usr/bin/env python3
"""ESP32-C3 USBブリッジ経由でTLM922Sコマンドを送受信する。"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path
from typing import Iterable, TextIO


class SerialReadError(RuntimeError):
    """Raised when the serial port stops being readable while monitoring."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor ESP32-C3 USB bridge and send TLM922S commands.")
    parser.add_argument(
        "--port",
        help="ESP32-C3 serial port, for example COM4 or /dev/ttyACM0. Omit to auto-detect.",
    )
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--log-dir", default="usb_bridge_logs", help="directory for serial logs")
    parser.add_argument("--quiet", action="store_true", help="do not echo serial lines to the console")
    return parser.parse_args()


def import_serial():
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required. Install it with: python -m pip install pyserial") from exc
    return serial


def auto_detect_port() -> str:
    """ESP32-C3らしいUSBシリアルポートを1つ選ぶ。"""
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise SystemExit("pyserial is required. Install it with: python -m pip install pyserial") from exc

    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("No serial ports were found. Connect the ESP32-C3 or specify --port.")

    keywords = ("esp32", "usb serial", "cp210", "ch340", "wch", "jtag", "cdc", "uart")
    candidates = [
        port
        for port in ports
        if any(
            keyword in " ".join(
                str(value).lower()
                for value in (port.description, port.manufacturer, port.product, port.hwid)
                if value
            )
            for keyword in keywords
        )
    ]

    if len(candidates) == 1:
        return candidates[0].device
    if len(ports) == 1:
        return ports[0].device

    choices = "\n".join(f"  {port.device}: {port.description}" for port in ports)
    raise SystemExit(f"Could not auto-detect a single ESP32 serial port. Specify --port.\nAvailable ports:\n{choices}")


def timestamp_for_filename() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def timestamp_for_log() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def write_log(log_file: TextIO, line: str) -> None:
    log_file.write(f"{timestamp_for_log()} {line}\n")
    log_file.flush()


def read_command_input(command_queue: "queue.Queue[str]") -> None:
    for line in sys.stdin:
        command = line.strip()
        if command:
            command_queue.put(command)


def write_pending_commands(ser, command_queue: "queue.Queue[str]") -> None:
    while True:
        try:
            command = command_queue.get_nowait()
        except queue.Empty:
            return
        ser.write(f"{command}\n".encode("utf-8"))
        ser.flush()
        print(f"[pc] sent: {command}", flush=True)


def serial_lines(port: str, baudrate: int, command_queue: "queue.Queue[str]") -> Iterable[str]:
    serial = import_serial()

    try:
        with serial.Serial(port, baudrate, timeout=0.2) as ser:
            ser.dtr = True
            ser.rts = False
            while True:
                try:
                    write_pending_commands(ser, command_queue)
                    line = ser.readline().decode("utf-8", errors="replace")
                except (OSError, serial.SerialException) as exc:
                    raise SerialReadError(f"Lost access to serial port {port}: {exc}") from exc
                yield line.strip() if line else ""
    except serial.SerialException as exc:
        raise SerialReadError(f"Could not open serial port {port}: {exc}") from exc


def main() -> int:
    args = parse_args()

    log_dir = Path(args.log_dir)
    port = args.port or auto_detect_port()
    command_queue: queue.Queue[str] = queue.Queue()
    threading.Thread(target=read_command_input, args=(command_queue,), daemon=True).start()

    print(f"Saving USB bridge logs to: {log_dir.resolve()}")
    print(f"Using serial port: {port}")
    print("Type a TLM922S command and press Enter.")

    log_dir.mkdir(parents=True, exist_ok=True)
    raw_path = log_dir / f"raw_serial_{timestamp_for_filename()}.log"

    with ExitStack() as stack:
        raw_log = stack.enter_context(raw_path.open("a", encoding="utf-8"))
        try:
            for line in serial_lines(port, args.baudrate, command_queue):
                if not line:
                    continue
                write_log(raw_log, line)
                if not args.quiet:
                    print(line)
        except SerialReadError as exc:
            print(f"[serial] {exc}", file=sys.stderr)
            print(
                "[serial] Monitoring stopped. Reconnect/reset the ESP32-C3, close any other serial monitor, "
                "then run this command again.",
                file=sys.stderr,
            )
            return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
