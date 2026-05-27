#!/usr/bin/env python3
import argparse

from tlm922s_uart import Tlm922sUart, hex_to_text, print_response


def parse_radio_rx(text):
    """'radio_rx ...' 応答から (payload_hex, rssi, snr) を取り出す。"""
    for line in text.replace("\r", "\n").splitlines():
        line = line.strip()
        if not line.startswith(">> radio_rx "):
            continue

        parts = line.split()
        # 資料では radio_rx <data> <rssi> <snr>。
        # ファームウェア例によっては <data> の前にバイト数が付く場合もある。
        if len(parts) >= 5 and all(c in "0123456789abcdefABCDEF" for c in parts[2]):
            return parts[2], parts[3], parts[4]
        if len(parts) >= 6 and all(c in "0123456789abcdefABCDEF" for c in parts[3]):
            return parts[3], parts[4], parts[5]

    return None


def parse_args():
    parser = argparse.ArgumentParser(description="Receive test messages with TLM922S P2P.")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--window-ms", type=int, default=10000, help="receive window in milliseconds")
    return parser.parse_args()


def main():
    args = parse_args()

    with Tlm922sUart(args.port, args.baudrate, timeout=(args.window_ms / 1000) + 2) as radio:
        print("Receiving P2P packets. Stop with Ctrl+C.")
        while True:
            command = f"p2p rx {args.window_ms}"
            print(f"\n> {command}")
            response = radio.command(command, wait=(args.window_ms / 1000) + 2)
            print_response(response)

            parsed = parse_radio_rx(response)
            if parsed is None:
                continue

            payload_hex, rssi, snr = parsed
            print(f"Decoded text: {hex_to_text(payload_hex)}")
            print(f"RSSI={rssi}, SNR={snr}")


if __name__ == "__main__":
    raise SystemExit(main())
