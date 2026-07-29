#!/usr/bin/env python3
"""受信側と同じP2P設定を送信側TLM922Sへ適用する。"""

import argparse
import sys
from pathlib import Path
from typing import Callable


sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from communication_manager import Tlm922sUart


# esp32c3_ground_station_receiver/src/main.cpp のP2P_SETTINGSと同じ値。
# (項目名, 取得コマンド, 設定コマンド, 期待値)
P2P_SETTINGS = (
    ("freq", "p2p get_freq", "p2p set_freq 922500000", "922500000"),
    ("pwr", "p2p get_pwr", "p2p set_pwr 20", "20"),
    ("sf", "p2p get_sf", "p2p set_sf 12", "12"),
    ("bw", "p2p get_bw", "p2p set_bw 125", "125"),
    ("cr", "p2p get_cr", "p2p set_cr 4/6", "4/6"),
    ("prlen", "p2p get_prlen", "p2p set_prlen 16", "16"),
    ("crc", "p2p get_crc", "p2p set_crc on", "on"),
    ("iqi", "p2p get_iqi", "p2p set_iqi off", "off"),
    ("sync", "p2p get_sync", "p2p set_sync 12", "12"),
)
COMMAND_WAIT_SECONDS = 1.0


def ok_response(text: str) -> bool:
    return ">> Ok" in text


def first_radio_value(response: str) -> str:
    """TLM922S応答から、Ok以外の最初の値を取り出す。"""
    for line in response.replace("\r", "\n").splitlines():
        line = line.strip()
        if not line.startswith(">>"):
            continue
        value = line[2:].strip()
        if value and value.lower() != "ok":
            return value
    return ""


def print_response(text: str) -> None:
    cleaned = text.replace("\r", "\n").strip()
    print(cleaned if cleaned else "(no response)")


def configure_radio(
    radio: Tlm922sUart,
    *,
    save: bool = True,
    report: Callable[[str], None] = print,
) -> bool:
    """受信側と照合し、異なる設定だけ更新する。

    戻り値は、少なくとも1項目を変更した場合にTrue。
    """
    changed = False
    report("P2P自動設定: 受信側との照合を開始")

    for label, get_command, set_command, expected in P2P_SETTINGS:
        response = radio.command(get_command, wait=COMMAND_WAIT_SECONDS)
        actual = first_radio_value(response)
        if not actual:
            raise RuntimeError(f"P2P設定を取得できません: {label}")

        report(f"P2P自動設定: {label}={actual}")
        if actual.lower() == expected.lower():
            continue

        report(f"P2P自動設定: {label}を{expected}へ変更")
        response = radio.command(set_command, wait=COMMAND_WAIT_SECONDS)
        if not ok_response(response):
            raise RuntimeError(f"P2P設定コマンドが失敗しました: {set_command}")
        changed = True

    if changed and save:
        response = radio.command("p2p save", wait=COMMAND_WAIT_SECONDS)
        if not ok_response(response):
            raise RuntimeError("P2P設定の保存に失敗しました")
        report("P2P自動設定: 変更値をTLM922Sへ保存")
    elif changed:
        report("P2P自動設定: 変更値は未保存")
    else:
        report("P2P自動設定: 受信側と一致、変更不要")

    return changed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Match the sender TLM922S P2P settings to the receiver."
    )
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--save", dest="save", action="store_true", default=True, help="save P2P settings to flash")
    parser.add_argument("--no-save", dest="save", action="store_false", help="do not save P2P settings to flash")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    with Tlm922sUart(args.port, args.baudrate) as radio:
        configure_radio(radio, save=args.save)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
