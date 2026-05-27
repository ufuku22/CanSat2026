#!/usr/bin/env python3
import argparse
import time

from tlm922s_uart import Tlm922sUart, print_response, text_to_hex


def parse_args():
    parser = argparse.ArgumentParser(description="Send test messages with TLM922S P2P.")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--interval", type=float, default=2.0, help="seconds between packets")
    parser.add_argument("--count", type=int, default=5, help="number of packets to send")
    parser.add_argument("--message", default="Hello from Raspberry Pi")
    return parser.parse_args()


def main():
    args = parse_args()

    with Tlm922sUart(args.port, args.baudrate, timeout=4.0) as radio:
        for i in range(1, args.count + 1):
            message = f"{args.message} #{i}"
            payload_hex = text_to_hex(message)
            command = f"p2p tx {payload_hex}"

            print(f"\nSending: {message}")
            print(f"> {command}")
            response = radio.command(command, wait=4.0)
            print_response(response)

            if "radio_tx_ok" not in response:
                print("WARNING: transmit success was not confirmed.")

            time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
