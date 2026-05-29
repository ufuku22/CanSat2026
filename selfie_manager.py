#!/usr/bin/env python3
"""自撮りカメラを扱うための管理クラス。"""

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


class SelfieManager:
    """自撮りカメラの展開、撮影通信、収納をまとめるクラス。"""

    def __init__(
        self,
        *,
        wifi_interface: str = WIFI_INTERFACE,
        ap_connection: str = AP_CONNECTION,
        ap_ssid: str = AP_SSID,
        ap_password: str = AP_PASSWORD,
        ap_ip_cidr: str = AP_IP_CIDR,
        ap_channel: str = AP_CHANNEL,
        restore_connection: str = RESTORE_CONNECTION,
        host: str = SERVER_HOST,
        port: int = SERVER_PORT,
        timeout_sec: float = TIMEOUT_SEC,
        image_dir: Path | str = IMAGE_DIR,
    ) -> None:
        self.wifi_interface = wifi_interface
        self.ap_connection = ap_connection
        self.ap_ssid = ap_ssid
        self.ap_password = ap_password
        self.ap_ip_cidr = ap_ip_cidr
        self.ap_channel = ap_channel
        self.restore_connection = restore_connection
        self.host = host
        self.port = port
        self.timeout_sec = timeout_sec
        self.image_dir = Path(image_dir)
        self._restore_needed = False

    def __enter__(self) -> "SelfieManager":
        return self

    def __exit__(self, *_: object) -> None:
        self.restore_wifi()

    def expand(self) -> None:
        """自撮りカメラを展開する。将来ここにモーター制御を追加する。"""
        pass

    def retract(self) -> None:
        """自撮りカメラを収納する。将来ここにモーター制御を追加する。"""
        pass

    def capture(self) -> Path:
        """AP起動から撮影、Wi-Fi復帰までを一通り実行する。"""
        self._ensure_root()
        try:
            self.start_ap()
            return self._receive_one_image()
        finally:
            self.restore_wifi()

    def start_ap(self) -> None:
        """ESP32S3が接続するためのラズパイ側APを起動する。"""
        if not self._connection_exists(self.ap_connection):
            self._run_command(
                "nmcli",
                "connection",
                "add",
                "type",
                "wifi",
                "ifname",
                self.wifi_interface,
                "con-name",
                self.ap_connection,
                "ssid",
                self.ap_ssid,
            )

        self._run_command(
            "nmcli",
            "connection",
            "modify",
            self.ap_connection,
            "802-11-wireless.mode",
            "ap",
            "802-11-wireless.band",
            "bg",
            "802-11-wireless.channel",
            self.ap_channel,
            "ipv4.method",
            "shared",
            "ipv4.addresses",
            self.ap_ip_cidr,
            "connection.autoconnect",
            "no",
            "wifi-sec.key-mgmt",
            "wpa-psk",
            "wifi-sec.psk",
            self.ap_password,
        )
        self._run_command("nmcli", "connection", "up", self.ap_connection)
        self._restore_needed = True
        log(f"AP started: {self.ap_ssid}")

    def restore_wifi(self) -> None:
        """APを停止し、普段使うWi-Fiへ戻す。"""
        if not self._restore_needed:
            return
        self._run_command("nmcli", "connection", "down", self.ap_connection, check=False)
        self._run_command("nmcli", "connection", "up", self.restore_connection, check=False)
        self._restore_needed = False
        log(f"Restored Wi-Fi connection: {self.restore_connection}")

    def _receive_one_image(self) -> Path:
        """ESP32S3へ撮影を指示し、JPEGを1枚受信する。"""
        with self._wait_for_esp() as connection:
            if self._receive_line(connection) != "READY":
                raise RuntimeError("ESP32S3 is not ready")

            self._send_line(connection, "CAPTURE")
            size_line = self._receive_line(connection)
            if not size_line.startswith("SIZE "):
                raise RuntimeError(f"Unexpected response: {size_line}")

            image_size = int(size_line.removeprefix("SIZE "))
            self._send_line(connection, "OK")
            image = self._receive_exact(connection, image_size)
            path = self._save_image(image)
            self._send_line(connection, "COMPLETE")
            log(f"Saved image: {path}")
            return path

    def _wait_for_esp(self) -> socket.socket:
        """ESP32S3からのTCP接続を待つ。"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.settimeout(self.timeout_sec)
        server_socket.bind((self.host, self.port))
        server_socket.listen(1)
        log(f"Waiting for ESP32S3 on TCP port {self.port}")
        try:
            connection, address = server_socket.accept()
            connection.settimeout(self.timeout_sec)
            log(f"ESP32S3 connected: {address}")
            return connection
        finally:
            server_socket.close()

    def _save_image(self, image: bytes) -> Path:
        """受信したJPEGを時刻付きファイル名で保存する。"""
        self.image_dir.mkdir(parents=True, exist_ok=True)
        path = self.image_dir / datetime.now().strftime("selfie_%Y%m%d_%H%M%S.jpg")
        path.write_bytes(image)
        return path

    def _connection_exists(self, name: str) -> bool:
        """NetworkManagerに指定した接続設定があるか確認する。"""
        result = subprocess.run(
            ["nmcli", "connection", "show", name],
            capture_output=True,
            text=True,
            check=False,
            timeout=COMMAND_TIMEOUT_SEC,
        )
        return result.returncode == 0

    def _run_command(self, *command: str, check: bool = True) -> subprocess.CompletedProcess[str]:
        """nmcliなどの外部コマンドを実行する。固まらないように短いタイムアウトを付ける。"""
        log("+ " + " ".join(command))
        try:
            return subprocess.run(command, text=True, check=check, timeout=COMMAND_TIMEOUT_SEC)
        except subprocess.TimeoutExpired as exc:
            log(f"ERROR COMMAND_TIMEOUT: {' '.join(command)}")
            if check:
                raise
            return subprocess.CompletedProcess(command, 124, "", str(exc))

    @staticmethod
    def _ensure_root() -> None:
        """Wi-Fiを切り替えるため、Linuxではsudo実行を必須にする。"""
        if os.name == "posix" and os.geteuid() != 0:
            raise SystemExit("Wi-Fiを切り替えるため sudo で実行してください。")

    @staticmethod
    def _receive_line(connection: socket.socket) -> str:
        """改行までの1行をTCPから読む。"""
        data = bytearray()
        while True:
            chunk = connection.recv(1)
            if not chunk:
                raise ConnectionError("Connection closed while reading line")
            if chunk == b"\n":
                return data.decode("utf-8").strip()
            data.extend(chunk)

    @staticmethod
    def _send_line(connection: socket.socket, line: str) -> None:
        """ESP32S3へ1行送る。"""
        connection.sendall((line + "\n").encode("utf-8"))

    @staticmethod
    def _receive_exact(connection: socket.socket, size: int) -> bytes:
        """指定バイト数の画像データを受信する。"""
        data = bytearray()
        while len(data) < size:
            chunk = connection.recv(min(BUFFER_SIZE, size - len(data)))
            if not chunk:
                raise ConnectionError("Connection closed while receiving image")
            data.extend(chunk)
        return bytes(data)


def main() -> None:
    with SelfieManager() as selfie:
        selfie.capture()


if __name__ == "__main__":
    main()
