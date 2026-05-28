#!/usr/bin/env python3
"""ESP32S3 SenseからWi-Fi/TCPでJPEGを1枚受け取る。"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import argparse
import socket
import subprocess


BUFFER_SIZE = 4096


@dataclass(frozen=True)
class WifiApConfig:
    interface: str = "wlan0"
    ap_connection: str = "cansat-camera-ap"
    ap_ssid: str = "CanSat-Camera"
    ap_password: str = "cansat2026"
    ap_ip_cidr: str = "192.168.42.1/24"
    original_connection: str | None = None


@dataclass(frozen=True)
class CameraServerConfig:
    host: str = "0.0.0.0"
    port: int = 5000
    timeout_sec: float = 120.0
    image_dir: Path = Path("raw_images")


class Esp32S3CameraReceiver:
    def __init__(
        self,
        wifi: WifiApConfig = WifiApConfig(),
        server: CameraServerConfig = CameraServerConfig(),
        *,
        manage_wifi: bool = True,
    ) -> None:
        self.wifi = wifi
        self.server = server
        self.manage_wifi = manage_wifi

    def start_ap(self) -> None:
        """ESP32S3が探すラズパイ側APを起動する。"""
        if not self.manage_wifi:
            print("Wi-Fi AP start skipped")
            return

        if not self._connection_exists(self.wifi.ap_connection):
            self._run(
                "nmcli",
                "connection",
                "add",
                "type",
                "wifi",
                "ifname",
                self.wifi.interface,
                "con-name",
                self.wifi.ap_connection,
                "ssid",
                self.wifi.ap_ssid,
            )

        self._run(
            "nmcli",
            "connection",
            "modify",
            self.wifi.ap_connection,
            "802-11-wireless.mode",
            "ap",
            "802-11-wireless.band",
            "bg",
            "ipv4.method",
            "shared",
            "ipv4.addresses",
            self.wifi.ap_ip_cidr,
            "wifi-sec.key-mgmt",
            "wpa-psk",
            "wifi-sec.psk",
            self.wifi.ap_password,
            "connection.autoconnect",
            "no",
        )
        self._run("nmcli", "connection", "up", self.wifi.ap_connection)
        print(f"AP started: {self.wifi.ap_ssid}")

    def stop_ap(self) -> None:
        if not self.manage_wifi:
            return
        self._run("nmcli", "connection", "down", self.wifi.ap_connection, check=False)
        print("AP stopped")

    def restore_original_wifi(self) -> None:
        """撮影処理の後、ラズパイを普段使うWi-Fiへ戻す。"""
        if not self.manage_wifi:
            print("Wi-Fi restore skipped")
            return

        self.stop_ap()
        if self.wifi.original_connection:
            self._run("nmcli", "connection", "up", self.wifi.original_connection, check=False)
            print(f"Restored Wi-Fi connection: {self.wifi.original_connection}")
            return

        self._run("nmcli", "device", "set", self.wifi.interface, "autoconnect", "yes", check=False)
        self._run("nmcli", "device", "connect", self.wifi.interface, check=False)
        print("Requested reconnect to saved Wi-Fi")

    def run_capture_sequence(self) -> Path:
        try:
            # ここから最後まで自動で実行する。途中で失敗してもfinallyでWi-Fiを戻す。
            self.start_ap()
            with self.wait_for_esp() as connection:
                self.wait_ready(connection)
                self.send_line(connection, "CAPTURE")
                image_size = self.receive_image_size(connection)
                self.send_line(connection, "OK")
                image = self.receive_exact(connection, image_size)
                path = self.save_image(image)
                if not self.validate_jpeg(path):
                    print("ERROR INVALID_JPEG")
                self.send_line(connection, "COMPLETE")
                print(f"Saved image: {path}")
                return path
        finally:
            self.restore_original_wifi()

    def wait_for_esp(self) -> socket.socket:
        # ESP32S3はラズパイAPを見つけた後、このTCPポートへ接続してくる。
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.settimeout(self.server.timeout_sec)
        server_socket.bind((self.server.host, self.server.port))
        server_socket.listen(1)
        print(f"Waiting for ESP32S3 on TCP port {self.server.port}")
        try:
            connection, address = server_socket.accept()
            connection.settimeout(self.server.timeout_sec)
            print(f"ESP32S3 connected: {address}")
            return connection
        finally:
            server_socket.close()

    def wait_ready(self, connection: socket.socket) -> None:
        # READYが来たら、ESP32S3が撮影コマンドを受け取れる状態。
        line = self.receive_line(connection)
        if line != "READY":
            raise RuntimeError(f"Expected READY, got {line!r}")

    def receive_image_size(self, connection: socket.socket) -> int:
        # JPEG本体を読む前に、何バイト届くかをESP32S3から受け取る。
        line = self.receive_line(connection)
        if line.startswith("ERROR "):
            raise RuntimeError(line)
        prefix = "SIZE "
        if not line.startswith(prefix):
            raise RuntimeError(f"Expected SIZE, got {line!r}")
        size = int(line[len(prefix) :])
        if size <= 0:
            raise RuntimeError(f"Invalid image size: {size}")
        return size

    def save_image(self, image: bytes) -> Path:
        # 後で最新画像を選びやすいように、日時入りのファイル名で保存する。
        self.server.image_dir.mkdir(parents=True, exist_ok=True)
        filename = datetime.now().strftime("selfie_%Y%m%d_%H%M%S.jpg")
        path = self.server.image_dir / filename
        path.write_bytes(image)
        return path

    @staticmethod
    def validate_jpeg(path: Path) -> bool:
        # 試験用の簡易確認。壊れていても保存自体は残す。
        data = path.read_bytes()
        return len(data) >= 4 and data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9"

    @staticmethod
    def receive_line(connection: socket.socket) -> str:
        data = bytearray()
        while True:
            chunk = connection.recv(1)
            if not chunk:
                raise ConnectionError("Connection closed while reading line")
            if chunk == b"\n":
                return data.decode("utf-8").strip()
            data.extend(chunk)

    @staticmethod
    def send_line(connection: socket.socket, line: str) -> None:
        connection.sendall((line + "\n").encode("utf-8"))

    @staticmethod
    def receive_exact(connection: socket.socket, size: int) -> bytes:
        # TCPは分割されて届くので、指定サイズになるまで繰り返し読む。
        data = bytearray()
        while len(data) < size:
            chunk = connection.recv(min(BUFFER_SIZE, size - len(data)))
            if not chunk:
                raise ConnectionError("Connection closed while receiving image")
            data.extend(chunk)
        return bytes(data)

    def _connection_exists(self, name: str) -> bool:
        result = subprocess.run(
            ["nmcli", "connection", "show", name],
            capture_output=True,
            text=True,
            check=False,
        )
        return result.returncode == 0

    @staticmethod
    def _run(*command: str, check: bool = True) -> subprocess.CompletedProcess[str]:
        print("+ " + " ".join(command))
        return subprocess.run(command, text=True, check=check)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive one JPEG from ESP32S3 Sense.")
    parser.add_argument("--no-wifi", action="store_true", help="Do not switch Wi-Fi; useful for bench tests.")
    parser.add_argument("--restore-only", action="store_true", help="Only restore the saved Wi-Fi connection.")
    parser.add_argument("--original-connection", help="NetworkManager connection name to restore after capture.")
    parser.add_argument("--ap-ssid", default="CanSat-Camera")
    parser.add_argument("--ap-password", default="cansat2026")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--image-dir", type=Path, default=Path("raw_images"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    receiver = Esp32S3CameraReceiver(
        wifi=WifiApConfig(
            ap_ssid=args.ap_ssid,
            ap_password=args.ap_password,
            original_connection=args.original_connection,
        ),
        server=CameraServerConfig(port=args.port, timeout_sec=args.timeout, image_dir=args.image_dir),
        manage_wifi=not args.no_wifi,
    )
    if args.restore_only:
        receiver.restore_original_wifi()
    else:
        receiver.run_capture_sequence()


if __name__ == "__main__":
    main()
