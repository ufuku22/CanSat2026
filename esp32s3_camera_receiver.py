#!/usr/bin/env python3
"""ESP32S3カメラからJPEGを1枚受信する最小構成の受信スクリプト。"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import os
import socket
import subprocess


# 必要に応じてここだけ書き換える。
WIFI_INTERFACE = "wlan0"
AP_CONNECTION = "cansat-camera-ap"
AP_SSID = "CanSat-Camera"
AP_PASSWORD = "cansat2026"
AP_IP_CIDR = "192.168.42.1/24"
AP_CHANNEL = "6"
RESTORE_CONNECTION = "netplan-wlan0-KimuraLab_StudentRoom"

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 5000
TIMEOUT_SEC = 120.0
IMAGE_DIR = Path("raw_images")

BUFFER_SIZE = 4096
COMMAND_TIMEOUT_SEC = 30.0


def log(message: str) -> None:
    """時刻付きで進行状況を表示する。"""
    now = datetime.now().strftime("%H:%M:%S")
    print(f"[{now}] {message}", flush=True)


def run_command(*command: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    """nmcliなどの外部コマンドを実行する。固まらないように短いタイムアウトを付ける。"""
    log("+ " + " ".join(command))
    try:
        return subprocess.run(command, text=True, check=check, timeout=COMMAND_TIMEOUT_SEC)
    except subprocess.TimeoutExpired as exc:
        log(f"ERROR COMMAND_TIMEOUT: {' '.join(command)}")
        if check:
            raise
        return subprocess.CompletedProcess(command, 124, "", str(exc))


def ensure_root() -> None:
    """Wi-Fiを切り替えるため、Linuxではsudo実行を必須にする。"""
    if os.name == "posix" and os.geteuid() != 0:
        raise SystemExit("Wi-Fiを切り替えるため sudo で実行してください。")


def connection_exists(name: str) -> bool:
    """NetworkManagerに指定した接続設定があるか確認する。"""
    result = subprocess.run(
        ["nmcli", "connection", "show", name],
        capture_output=True,
        text=True,
        check=False,
        timeout=COMMAND_TIMEOUT_SEC,
    )
    return result.returncode == 0


def start_ap() -> None:
    """ESP32S3が接続するためのラズパイ側APを起動する。"""
    if not connection_exists(AP_CONNECTION):
        run_command(
            "nmcli",
            "connection",
            "add",
            "type",
            "wifi",
            "ifname",
            WIFI_INTERFACE,
            "con-name",
            AP_CONNECTION,
            "ssid",
            AP_SSID,
        )

    run_command(
        "nmcli",
        "connection",
        "modify",
        AP_CONNECTION,
        "802-11-wireless.mode",
        "ap",
        "802-11-wireless.band",
        "bg",
        "802-11-wireless.channel",
        AP_CHANNEL,
        "ipv4.method",
        "shared",
        "ipv4.addresses",
        AP_IP_CIDR,
        "connection.autoconnect",
        "no",
        "wifi-sec.key-mgmt",
        "wpa-psk",
        "wifi-sec.psk",
        AP_PASSWORD,
    )
    run_command("nmcli", "connection", "up", AP_CONNECTION)
    log(f"AP started: {AP_SSID}")


def restore_wifi() -> None:
    """APを停止し、普段使うWi-Fiへ戻す。"""
    run_command("nmcli", "connection", "down", AP_CONNECTION, check=False)
    run_command("nmcli", "connection", "up", RESTORE_CONNECTION, check=False)
    log(f"Restored Wi-Fi connection: {RESTORE_CONNECTION}")


def receive_line(connection: socket.socket) -> str:
    """改行までの1行をTCPから読む。"""
    data = bytearray()
    while True:
        chunk = connection.recv(1)
        if not chunk:
            raise ConnectionError("Connection closed while reading line")
        if chunk == b"\n":
            return data.decode("utf-8").strip()
        data.extend(chunk)


def send_line(connection: socket.socket, line: str) -> None:
    """ESP32S3へ1行送る。"""
    connection.sendall((line + "\n").encode("utf-8"))


def receive_exact(connection: socket.socket, size: int) -> bytes:
    """指定バイト数の画像データを受信する。"""
    data = bytearray()
    while len(data) < size:
        chunk = connection.recv(min(BUFFER_SIZE, size - len(data)))
        if not chunk:
            raise ConnectionError("Connection closed while receiving image")
        data.extend(chunk)
    return bytes(data)


def save_image(image: bytes) -> Path:
    """受信したJPEGを時刻付きファイル名で保存する。"""
    IMAGE_DIR.mkdir(parents=True, exist_ok=True)
    path = IMAGE_DIR / datetime.now().strftime("selfie_%Y%m%d_%H%M%S.jpg")
    path.write_bytes(image)
    return path


def wait_for_esp() -> socket.socket:
    """ESP32S3からのTCP接続を待つ。"""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.settimeout(TIMEOUT_SEC)
    server_socket.bind((SERVER_HOST, SERVER_PORT))
    server_socket.listen(1)
    log(f"Waiting for ESP32S3 on TCP port {SERVER_PORT}")
    try:
        connection, address = server_socket.accept()
        connection.settimeout(TIMEOUT_SEC)
        log(f"ESP32S3 connected: {address}")
        return connection
    finally:
        server_socket.close()


def receive_one_image() -> Path:
    """ESP32S3へ撮影を指示し、JPEGを1枚受信する。"""
    with wait_for_esp() as connection:
        if receive_line(connection) != "READY":
            raise RuntimeError("ESP32S3 is not ready")

        send_line(connection, "CAPTURE")
        size_line = receive_line(connection)
        if not size_line.startswith("SIZE "):
            raise RuntimeError(f"Unexpected response: {size_line}")

        image_size = int(size_line.removeprefix("SIZE "))
        send_line(connection, "OK")
        image = receive_exact(connection, image_size)
        path = save_image(image)
        send_line(connection, "COMPLETE")
        log(f"Saved image: {path}")
        return path


def run_capture_sequence() -> Path:
    """AP起動から撮影、Wi-Fi復帰までを一通り実行する。"""
    ensure_root()
    try:
        start_ap()
        return receive_one_image()
    finally:
        restore_wifi()


def main() -> None:
    run_capture_sequence()


if __name__ == "__main__":
    main()
