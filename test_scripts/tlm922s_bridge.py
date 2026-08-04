#!/usr/bin/env python3
"""TLM922Sへ手入力したコマンドを送り、UART応答をそのまま表示する。"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import threading


# リポジトリ直下の通信ドライバーと設定を読み込む。
sys.path.append(str(Path(__file__).resolve().parents[1]))

from communication_manager import Tlm922sUart
from config import CommunicationConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactively send commands to a TLM922S and display its responses."
    )
    parser.add_argument("--port", default=CommunicationConfig.UART_PORT)
    parser.add_argument(
        "--baudrate", type=int, default=CommunicationConfig.UART_BAUDRATE
    )
    return parser.parse_args()


def display_responses(radio: Tlm922sUart, stop_event: threading.Event) -> None:
    """TLM922Sから届いた文字列を、コマンド入力中も継続して表示する。"""
    while not stop_event.is_set():
        response = radio.read_for(0.2)
        if response:
            print(response, end="", flush=True)


def main() -> int:
    args = parse_args()
    stop_event = threading.Event()

    print(f"Port: {args.port} ({args.baudrate} baud)")
    print("TLM922Sコマンドを入力してEnterを押してください。")
    print("終了するには quit または exit を入力してください。")

    try:
        with Tlm922sUart(port=args.port, baudrate=args.baudrate) as radio:
            reader = threading.Thread(
                target=display_responses,
                args=(radio, stop_event),
                daemon=True,
            )
            reader.start()

            try:
                while True:
                    command = input("tlm922s> ").strip()
                    if command.lower() in {"quit", "exit"}:
                        break
                    if not command:
                        continue

                    try:
                        radio.send_command(command)
                    except UnicodeEncodeError:
                        print("コマンドにはASCII文字だけを使用してください。")
            finally:
                stop_event.set()
                reader.join(timeout=1.0)
    except KeyboardInterrupt:
        print("\n終了します。")
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"UARTエラー: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
