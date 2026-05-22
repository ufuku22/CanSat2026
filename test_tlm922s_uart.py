#!/usr/bin/env python3
import argparse
import os
import select
import sys
import termios
import time


DEFAULT_PORT = "/dev/serial0"
DEFAULT_BAUDRATES = [115200, 9600, 57600, 19200]
READ_ONLY_COMMANDS = [
    "mod get_ver",
    "mod get_hw_model",
    "mod get_hw_deveui",
]


BAUD_MAP = {
    9600: termios.B9600,
    19200: termios.B19200,
    57600: termios.B57600,
    115200: termios.B115200,
}


class Uart:
    def __init__(self, port, baudrate, timeout):
        if baudrate not in BAUD_MAP:
            raise ValueError(f"Unsupported baudrate: {baudrate}")
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.fd = None

    def __enter__(self):
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)

        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = BAUD_MAP[self.baudrate]
        attrs[5] = BAUD_MAP[self.baudrate]
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0

        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def write_line(self, command):
        os.write(self.fd, command.encode("ascii") + b"\r")

    def read_available(self, duration):
        end_time = time.monotonic() + duration
        chunks = []
        while time.monotonic() < end_time:
            ready, _, _ = select.select([self.fd], [], [], 0.05)
            if not ready:
                continue
            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            if data:
                chunks.append(data)
        return b"".join(chunks)

    def transact(self, command):
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.write_line(command)
        return self.read_available(self.timeout)


def clean_text(data):
    return data.decode("ascii", errors="replace").replace("\r", "\n")


def response_lines(text):
    return [line.strip() for line in text.splitlines() if line.strip()]


def looks_ok(text):
    lines = response_lines(text)
    return any(line.startswith(">>") and "Invalid" not in line for line in lines)


def test_once(port, baudrate, timeout, commands, boot_wait):
    print(f"\n=== UART test: port={port}, baudrate={baudrate}, 8N1 ===")
    with Uart(port, baudrate, timeout) as uart:
        boot_text = clean_text(uart.read_available(boot_wait))
        if boot_text.strip():
            print("[boot/idle]")
            print(boot_text.strip())
        else:
            print("[boot/idle] no text received")

        passed = 0
        for command in commands:
            print(f"\n> {command}")
            text = clean_text(uart.transact(command))
            if text.strip():
                print(text.strip())
            else:
                print("(no response)")

            if looks_ok(text):
                passed += 1

        print(f"\nResult: {passed}/{len(commands)} commands returned a valid-looking response.")
        return passed == len(commands)


def parse_args():
    parser = argparse.ArgumentParser(
        description="TLM922S UART command-interface self test for Raspberry Pi."
    )
    parser.add_argument(
        "-p",
        "--port",
        default=DEFAULT_PORT,
        help=f"serial port device, default: {DEFAULT_PORT}",
    )
    parser.add_argument(
        "-b",
        "--baudrate",
        type=int,
        default=115200,
        choices=sorted(BAUD_MAP),
        help="UART baudrate. TLM922S-P01A default is 115200; ADB922 shield may be 9600.",
    )
    parser.add_argument(
        "--scan",
        action="store_true",
        help="try 115200, 9600, 57600, and 19200 until the module answers",
    )
    parser.add_argument(
        "-t",
        "--timeout",
        type=float,
        default=1.5,
        help="seconds to wait for each command response",
    )
    parser.add_argument(
        "--boot-wait",
        type=float,
        default=1.0,
        help="seconds to listen for the module prompt or boot banner before commands",
    )
    parser.add_argument(
        "-c",
        "--command",
        action="append",
        help="command to send; can be repeated. Defaults to safe read-only module commands.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    commands = args.command or READ_ONLY_COMMANDS
    baudrates = DEFAULT_BAUDRATES if args.scan else [args.baudrate]

    print("TLM922S UART single-unit test")
    print("Wiring check:")
    print("  Raspberry Pi TXD(GPIO14/pin 8)  -> TLM922S RXD")
    print("  Raspberry Pi RXD(GPIO15/pin 10) -> TLM922S TXD")
    print("  Raspberry Pi GND                -> TLM922S GND")
    print("  Use 3.3 V UART logic. Do not connect RS-232 voltage directly.")

    for baudrate in baudrates:
        try:
            if test_once(args.port, baudrate, args.timeout, commands, args.boot_wait):
                print(f"\nOK: TLM922S answered on {args.port} at {baudrate}-8N1.")
                return 0
        except PermissionError:
            print(f"\nERROR: Permission denied opening {args.port}.")
            print("Try: sudo usermod -aG dialout $USER")
            print("Then reboot or log in again. For a quick test, run this script with sudo.")
            return 2
        except FileNotFoundError:
            print(f"\nERROR: Serial port was not found: {args.port}")
            print("On Raspberry Pi, enable serial port and try /dev/serial0 or /dev/ttyAMA0.")
            return 2
        except OSError as exc:
            print(f"\nERROR: Could not use {args.port}: {exc}")
            return 2

    print("\nNG: No valid TLM922S response was detected.")
    print("Check power, GND, TX/RX crossing, UART enabled, and the baudrate.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
