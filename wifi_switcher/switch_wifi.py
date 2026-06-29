#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Raspberry Pi Wi-Fi switcher for USB-SSH operation.

外部パッケージを追加せず、OS標準の NetworkManager / wpa_supplicant 系ツールを使う。
"""

from __future__ import annotations

import argparse
import getpass
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass
class AccessPoint:
    ssid: str
    signal: str
    security: str


def die(message: str) -> None:
    print(f"ERROR: {message}", file=sys.stderr)
    raise SystemExit(1)


def has_cmd(name: str) -> bool:
    return shutil.which(name) is not None


def run(
    args: list[str],
    *,
    check: bool = False,
    capture: bool = False,
    input_text: str | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        check=check,
        text=True,
        input=input_text,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.PIPE if capture else None,
    )


def need_root() -> None:
    if os.geteuid() == 0:
        return
    if has_cmd("sudo"):
        os.execvp("sudo", ["sudo", sys.executable, *sys.argv])
    die("root権限が必要です。sudo python3 switch_wifi.py で実行してください。")


def detect_iface(requested: str | None) -> str:
    if requested:
        return requested

    if Path("/sys/class/net/wlan0").exists():
        return "wlan0"

    for path in Path("/sys/class/net").glob("wl*"):
        return path.name

    die("Wi-Fiインターフェースが見つかりません。例: sudo WIFI_IFACE=wlan0 python3 switch_wifi.py")


def detect_backend() -> str:
    if has_cmd("nmcli"):
        result = run(["nmcli", "-t", "-f", "RUNNING", "general"], capture=True)
        if result.returncode == 0 and result.stdout.strip() == "running":
            return "nmcli"

    if has_cmd("wpa_cli") and has_cmd("wpa_passphrase"):
        return "wpa"

    die("nmcli または wpa_cli/wpa_passphrase が見つかりません。")


def print_header(iface: str, backend: str) -> None:
    print()
    print("=== Raspberry Pi Wi-Fi Switcher ===")
    print(f"interface: {iface}")
    print(f"backend  : {backend}")
    print("USB-SSH接続中に使う前提です。Wi-Fi側のSSH接続から実行すると切断されます。")
    print()


def parse_nmcli_line(line: str) -> AccessPoint | None:
    parts = line.rstrip("\n").split(":")
    if len(parts) < 3:
        return None
    ssid, signal, security = parts[0], parts[1], ":".join(parts[2:])
    if not ssid:
        return None
    return AccessPoint(ssid=ssid, signal=signal, security=security)


def scan_nmcli(iface: str) -> list[AccessPoint]:
    run(["nmcli", "radio", "wifi", "on"], capture=True)
    run(["nmcli", "device", "wifi", "rescan", "ifname", iface], capture=True)
    time.sleep(2)
    result = run(
        ["nmcli", "-t", "--escape", "no", "-f", "SSID,SIGNAL,SECURITY", "device", "wifi", "list", "ifname", iface],
        capture=True,
    )

    access_points: list[AccessPoint] = []
    seen: set[str] = set()
    for line in result.stdout.splitlines():
        ap = parse_nmcli_line(line)
        if ap and ap.ssid not in seen:
            access_points.append(ap)
            seen.add(ap.ssid)
    return access_points


def scan_wpa(iface: str) -> list[AccessPoint]:
    run(["ip", "link", "set", iface, "up"], capture=True)
    run(["wpa_cli", "-i", iface, "scan"], capture=True)
    time.sleep(3)
    result = run(["wpa_cli", "-i", iface, "scan_results"], capture=True)

    access_points: list[AccessPoint] = []
    seen: set[str] = set()
    for line in result.stdout.splitlines()[2:]:
        parts = line.split(None, 4)
        if len(parts) < 5:
            continue
        _, _, signal, flags, ssid = parts
        if ssid and ssid not in seen:
            access_points.append(AccessPoint(ssid=ssid, signal=signal, security=flags))
            seen.add(ssid)
    return access_points


def scan_networks(iface: str, backend: str) -> list[AccessPoint]:
    print("APを探索しています...")
    access_points = scan_nmcli(iface) if backend == "nmcli" else scan_wpa(iface)

    if not access_points:
        print("APが見つかりませんでした。SSIDを手入力できます。")
        return []

    print()
    print("見つかったAP:")
    for index, ap in enumerate(access_points, start=1):
        print(f"  {index:2d}) {ap.ssid[:32]:32s} signal:{ap.signal:4s} security:{ap.security}")
    return access_points


def select_ssid(access_points: list[AccessPoint]) -> str:
    print()
    choice = input("接続する番号、またはSSIDを直接入力してください: ").strip()
    if not choice:
        die("SSIDが空です。")

    if choice.isdigit() and access_points:
        index = int(choice)
        if 1 <= index <= len(access_points):
            return access_points[index - 1].ssid
        die("指定された番号のAPがありません。")

    return choice


def connect_nmcli(iface: str, ssid: str, password: str) -> None:
    delete_nmcli_profiles(ssid)
    args = ["nmcli", "device", "wifi", "connect", ssid]
    if password:
        args.extend(["password", password])
    args.extend(["ifname", iface])
    result = run(args, capture=True)
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        die(f"接続に失敗しました。{detail}")


def split_nmcli_escaped(line: str) -> list[str]:
    parts: list[str] = []
    current = []
    escaped = False
    for char in line:
        if escaped:
            current.append(char)
            escaped = False
        elif char == "\\":
            escaped = True
        elif char == ":":
            parts.append("".join(current))
            current = []
        else:
            current.append(char)
    parts.append("".join(current))
    return parts


def delete_nmcli_profiles(ssid: str) -> None:
    result = run(["nmcli", "-t", "-f", "NAME,TYPE", "connection", "show"], capture=True)
    if result.returncode != 0:
        return

    for line in result.stdout.splitlines():
        parts = split_nmcli_escaped(line)
        if len(parts) < 2:
            continue
        name, conn_type = parts[0], parts[1]
        if name == ssid and conn_type in ("802-11-wireless", "wifi"):
            run(["nmcli", "connection", "delete", name], capture=True)


def remove_existing_network_block(conf_text: str, ssid: str) -> str:
    pattern = re.compile(r"(^[ \t]*network=\{.*?^[ \t]*\}\s*)", re.MULTILINE | re.DOTALL)
    kept_blocks: list[str] = []
    last_end = 0
    output = []

    for match in pattern.finditer(conf_text):
        output.append(conf_text[last_end : match.start()])
        block = match.group(1)
        if f'ssid="{ssid}"' not in block:
            kept_blocks.append(block)
            output.append(block)
        last_end = match.end()

    output.append(conf_text[last_end:])
    return "".join(output).rstrip() + "\n"


def make_wpa_block(ssid: str, password: str) -> str:
    if not password:
        return f'\nnetwork={{\n    ssid="{ssid}"\n    key_mgmt=NONE\n}}\n'

    result = run(["wpa_passphrase", ssid], input_text=f"{password}\n", capture=True)
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        die(f"Wi-Fi設定の生成に失敗しました。{detail}")
    return "\n" + result.stdout


def connect_wpa(iface: str, ssid: str, password: str) -> None:
    conf = Path("/etc/wpa_supplicant/wpa_supplicant.conf")
    if not conf.exists():
        die(f"{conf} が見つかりません。")

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    backup = conf.with_name(f"{conf.name}.bak.{timestamp}")
    shutil.copy2(conf, backup)

    conf_text = conf.read_text(encoding="utf-8")
    new_text = remove_existing_network_block(conf_text, ssid) + make_wpa_block(ssid, password)

    with tempfile.NamedTemporaryFile("w", encoding="utf-8", delete=False) as temp_file:
        temp_file.write(new_text)
        temp_name = temp_file.name

    os.chmod(temp_name, 0o600)
    shutil.move(temp_name, conf)

    run(["wpa_cli", "-i", iface, "reconfigure"], capture=True)
    if has_cmd("dhclient"):
        run(["dhclient", "-r", iface], capture=True)
        run(["dhclient", iface], capture=True)
    elif has_cmd("systemctl"):
        run(["systemctl", "restart", "dhcpcd"], capture=True)


def show_status(iface: str, backend: str) -> None:
    print()
    print("接続状態:")
    if backend == "nmcli":
        result = run(["nmcli", "-f", "GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS", "device", "show", iface], capture=True)
        print(result.stdout.strip())
        return

    status = run(["wpa_cli", "-i", iface, "status"], capture=True)
    for line in status.stdout.splitlines():
        if line.startswith(("wpa_state=", "ssid=", "ip_address=")):
            print(line)

    ip_addr = run(["ip", "-4", "addr", "show", iface], capture=True)
    for line in ip_addr.stdout.splitlines():
        stripped = line.strip()
        if stripped.startswith("inet "):
            print(f"ip_address={stripped.split()[1]}")


def has_ipv4(iface: str) -> bool:
    result = run(["ip", "-4", "addr", "show", iface], capture=True)
    return " inet " in result.stdout


def wait_for_connection(iface: str, backend: str) -> int:
    print()
    print("接続完了を待っています...")
    for _ in range(10):
        if has_ipv4(iface):
            print("IPアドレスを取得しました。")
            show_status(iface, backend)
            return 0
        time.sleep(2)

    print("まだIPアドレスを確認できません。パスワードや電波状況を確認してください。")
    show_status(iface, backend)
    return 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="USB-SSH中のRaspberry PiでWi-Fi接続先を対話式に切り替えます。",
    )
    parser.add_argument(
        "-i",
        "--iface",
        default=os.environ.get("WIFI_IFACE"),
        help="Wi-Fiインターフェース名。省略時は wlan0 または wl* を自動検出します。",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    need_root()

    iface = detect_iface(args.iface)
    backend = detect_backend()
    print_header(iface, backend)

    access_points = scan_networks(iface, backend)
    ssid = select_ssid(access_points)
    password = getpass.getpass("パスワードを入力してください（オープンAPなら空Enter）: ")

    print()
    print(f"接続を切り替えます: {ssid}")
    if backend == "nmcli":
        connect_nmcli(iface, ssid, password)
    else:
        connect_wpa(iface, ssid, password)

    return wait_for_connection(iface, backend)


if __name__ == "__main__":
    raise SystemExit(main())
