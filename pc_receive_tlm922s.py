#!/usr/bin/env python3
"""Receive TLM922S-P01A P2P packets from a PC serial port.

The expected connection is:

    PC USB serial port <-> ESP32-S3 UART bridge <-> TLM922S-P01A UART

The ESP32-S3 firmware should forward bytes transparently between USB CDC
serial and the UART connected to the TLM922S-P01A.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


DEFAULT_BAUDRATE = 115200
DEFAULT_RX_WINDOW_MS = 3000
DEFAULT_TIMEOUT = 1.5


@dataclass(frozen=True)
class RadioPacket:
    payload: bytes
    rssi: int | None = None
    snr: int | None = None

    @property
    def hex(self) -> str:
        return self.payload.hex()

    @property
    def text(self) -> str:
        return self.payload.decode("utf-8", errors="replace")


def require_pyserial() -> None:
    if serial is None:
        raise SystemExit(
            "pyserial is required. Install it with: python -m pip install pyserial"
        )


def available_ports() -> list[str]:
    require_pyserial()
    return [
        f"{port.device}  {port.description}"
        for port in sorted(list_ports.comports(), key=lambda item: item.device)
    ]


def clean_text(data: bytes) -> str:
    return data.decode("ascii", errors="replace").replace("\r", "\n")


def read_for(ser: Any, seconds: float) -> str:
    deadline = time.monotonic() + seconds
    chunks: list[bytes] = []

    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting:
            chunks.append(ser.read(waiting))
        else:
            time.sleep(0.02)

    return clean_text(b"".join(chunks))


def send_command(ser: Any, command: str, wait: float) -> str:
    ser.reset_input_buffer()
    ser.write(command.encode("ascii") + b"\r")
    ser.flush()
    return read_for(ser, wait)


def parse_radio_rx(text: str) -> RadioPacket | None:
    """Parse a TLM922S response line like: >> radio_rx <hex> <rssi> <snr>."""
    for line in text.replace("\r", "\n").splitlines():
        line = line.strip()
        if not line.startswith(">> radio_rx "):
            continue

        parts = line.split()
        for index, part in enumerate(parts):
            if is_hex(part) and index + 2 < len(parts):
                return RadioPacket(
                    payload=bytes.fromhex(part),
                    rssi=to_int(parts[index + 1]),
                    snr=to_int(parts[index + 2]),
                )

    return None


def is_hex(value: str) -> bool:
    if not value or len(value) % 2:
        return False

    try:
        bytes.fromhex(value)
    except ValueError:
        return False
    return True


def to_int(value: str) -> int | None:
    try:
        return int(value)
    except ValueError:
        return None


def decode_json_payload(packet: RadioPacket) -> dict[str, Any] | None:
    try:
        value = json.loads(packet.text)
    except json.JSONDecodeError:
        return None

    return value if isinstance(value, dict) else None


def packet_record(packet: RadioPacket) -> dict[str, Any]:
    record: dict[str, Any] = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "payload_hex": packet.hex,
        "payload_text": packet.text,
        "rssi": packet.rssi,
        "snr": packet.snr,
    }

    json_payload = decode_json_payload(packet)
    if json_payload is not None:
        record["payload_json"] = json_payload

    return record


def print_packet(packet: RadioPacket) -> None:
    record = packet_record(packet)
    print(json.dumps(record, ensure_ascii=False), flush=True)


def append_jsonl(path: Path, packet: RadioPacket) -> None:
    with path.open("a", encoding="utf-8") as file:
        file.write(json.dumps(packet_record(packet), ensure_ascii=False) + "\n")


def receive_loop(args: argparse.Namespace) -> int:
    require_pyserial()

    with serial.Serial(
        port=args.port,
        baudrate=args.baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=args.timeout,
        write_timeout=args.timeout,
    ) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print(
            f"Listening on {args.port} at {args.baudrate}-8N1 "
            f"(rx window {args.rx_window_ms} ms). Press Ctrl+C to stop.",
            flush=True,
        )

        if args.boot_wait > 0:
            boot_text = read_for(ser, args.boot_wait).strip()
            if boot_text and args.raw:
                print("[boot/idle]")
                print(boot_text, flush=True)

        for command in args.init_command:
            response = send_command(ser, command, wait=args.timeout)
            if args.raw:
                print(f"> {command}")
                print(response.strip() or "(no response)", flush=True)

        while True:
            wait = (args.rx_window_ms / 1000.0) + args.timeout
            response = send_command(ser, f"p2p rx {args.rx_window_ms}", wait=wait)

            if args.raw and response.strip():
                print(response.strip(), flush=True)

            packet = parse_radio_rx(response)
            if packet:
                print_packet(packet)
                if args.output:
                    append_jsonl(args.output, packet)
                if args.once:
                    return 0
            elif args.once:
                return 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive TLM922S-P01A P2P packets through a PC serial port."
    )
    parser.add_argument(
        "-p",
        "--port",
        help="Serial port connected to ESP32-S3, for example COM5 on Windows.",
    )
    parser.add_argument(
        "-b",
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"UART baudrate, default: {DEFAULT_BAUDRATE}",
    )
    parser.add_argument(
        "--rx-window-ms",
        type=int,
        default=DEFAULT_RX_WINDOW_MS,
        help=f"TLM922S p2p rx window in milliseconds, default: {DEFAULT_RX_WINDOW_MS}",
    )
    parser.add_argument(
        "-t",
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT,
        help=f"Extra seconds to wait for command responses, default: {DEFAULT_TIMEOUT}",
    )
    parser.add_argument(
        "--boot-wait",
        type=float,
        default=0.5,
        help="Seconds to read idle text after opening the port, default: 0.5",
    )
    parser.add_argument(
        "--init-command",
        action="append",
        default=[],
        help="Command to send before receiving. Can be repeated.",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Print raw TLM922S command responses for debugging.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Exit after the first received packet, or after one empty rx window.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Append received packets as JSON Lines to this file.",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="Show serial ports and exit.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.list_ports:
        ports = available_ports()
        print("\n".join(ports) if ports else "No serial ports found.")
        return 0

    if not args.port:
        print("ERROR: --port is required. Use --list-ports to find it.", file=sys.stderr)
        return 2

    try:
        return receive_loop(args)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    except serial.SerialException as exc:
        print(f"ERROR: Serial port error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
