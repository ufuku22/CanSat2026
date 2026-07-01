#!/usr/bin/env python3
"""自撮りカメラを扱うための管理クラス。"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import os
import socket
import subprocess
import time

from logger import Logger

SCRIPT_DIR = Path(__file__).resolve().parent

# 必要に応じてここだけ書き換える。
WIFI_INTERFACE = "wlan0"
AP_CONNECTION = "cansat-camera-ap"
AP_SSID = "CanSat-Camera"
AP_PASSWORD = "cansat2026"
AP_IP_CIDR = "192.168.42.1/24"
AP_CHANNEL = "6"
RESTORE_CONNECTION = "KimuraLab_StudentRoom"

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 5000
TIMEOUT_SEC = 180.0
PING_TIMEOUT_SEC = 5.0
IMAGE_DIR = Path("SCRIPT_DIR/raw_images")

BUFFER_SIZE = 16384
COMMAND_TIMEOUT_SEC = 30.0

MOTOR_PH_PIN = 5
MOTOR_EN_PIN = 13
MOTOR_SLEEP_PIN = 6
MOTOR_PWM_FREQUENCY_HZ = 1000
ARM_MOTOR_SPEED = 1.0
ARM_EXPAND_SECONDS = 20.0
ARM_RETRACT_SECONDS = 15.0


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
        ping_timeout_sec: float = PING_TIMEOUT_SEC,
        image_dir: Path | str = IMAGE_DIR,
        motor_ph_pin: int = MOTOR_PH_PIN,
        motor_en_pin: int = MOTOR_EN_PIN,
        motor_sleep_pin: int = MOTOR_SLEEP_PIN,
        logger: Logger | None = None,
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
        self.ping_timeout_sec = ping_timeout_sec
        self.image_dir = Path(image_dir)
        self.motor_ph_pin = motor_ph_pin
        self.motor_en_pin = motor_en_pin
        self.motor_sleep_pin = motor_sleep_pin
        self.logger = logger if logger is not None else Logger(log_to_file=False)
        self._restore_needed = False
        self.connection: socket.socket | None = None

    def __enter__(self) -> "SelfieManager":
        return self

    def __exit__(self, *_: object) -> None:
        self.close_connection()
        self.restore_wifi()

    def expand(self) -> None:
        """自撮りカメラを展開する。"""
        self._run_motor(ph_value=False, speed=ARM_MOTOR_SPEED, run_seconds=ARM_EXPAND_SECONDS)

    def retract(self) -> None:
        """自撮りカメラを収納する。"""
        self._run_motor(ph_value=True, speed=ARM_MOTOR_SPEED, run_seconds=ARM_RETRACT_SECONDS)

    def _run_motor(self, *, ph_value: bool, speed: float, run_seconds: float) -> None:
        from gpiozero import OutputDevice, PWMOutputDevice

        speed = max(0.0, min(float(speed), 1.0))
        run_seconds = float(run_seconds)
        self.logger.event(
            "Arm motor start: "
            f"PH_GPIO={self.motor_ph_pin}, EN_GPIO={self.motor_en_pin}, "
            f"SLEEP_GPIO={self.motor_sleep_pin}, ph={ph_value}, pwm={speed:g}, "
            f"seconds={run_seconds:g}"
        )

        ph = OutputDevice(self.motor_ph_pin, active_high=True, initial_value=False)
        en = PWMOutputDevice(
            self.motor_en_pin,
            active_high=True,
            initial_value=0.0,
            frequency=MOTOR_PWM_FREQUENCY_HZ,
        )
        sleep = OutputDevice(self.motor_sleep_pin, active_high=True, initial_value=False)

        try:
            sleep.on()
            time.sleep(0.002)
            ph.value = ph_value
            en.value = speed
            time.sleep(run_seconds)
        finally:
            en.value = 0.0
            ph.off()
            sleep.off()
            en.close()
            ph.close()
            sleep.close()
            self.logger.event("Arm motor stopped")

    def capture(self) -> Path:
        """テスト用。AP起動、接続、撮影、Wi-Fi復帰までを1回だけ実行する。"""
        try:
            self.start_ap()
            self.wait_connection()
            return self.capture_connected()
        finally:
            self.close_connection()
            self.restore_wifi()

    def start_ap(self) -> None:
        """ESP32S3が接続するためのラズパイ側APを起動する。"""
        self._ensure_root()
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
        self.logger.event(f"AP started: {self.ap_ssid}")

    def wait_connection(self) -> None:
        """ESP32S3からのTCP接続を待ち、READYを受け取る。"""
        self.close_connection()
        self.connection = self._wait_for_esp()
        if self._receive_line(self.connection) != "READY":
            self.close_connection()
            raise RuntimeError("ESP32S3 is not ready")

    def close_connection(self) -> None:
        """TCP接続だけを閉じる。APは落とさない。"""
        if self.connection is None:
            return
        self.connection.close()
        self.connection = None

    def ping(self) -> bool:
        """撮影前にESP32S3とのTCP接続が生きているか確認する。"""
        if self.connection is None:
            return False
        try:
            self.connection.settimeout(self.ping_timeout_sec)
            self._send_line(self.connection, "PING")
            return self._receive_line(self.connection) == "PONG"
        except (ConnectionError, OSError, socket.timeout):
            return False
        finally:
            if self.connection is not None:
                self.connection.settimeout(self.timeout_sec)

    def ensure_connection(self) -> None:
        """接続が切れていれば、APは維持したままESP32S3の再接続を待つ。"""
        if self.ping():
            return
        self.close_connection()
        self.wait_connection()

    def capture_connected(self) -> Path:
        """接続済みのESP32S3へ撮影を指示し、JPEGを1枚受信する。"""
        self.ensure_connection()
        if self.connection is None:
            raise RuntimeError("ESP32S3 is not connected")

        try:
            self._send_line(self.connection, "CAPTURE")
            size_line = self._receive_line(self.connection)
            if not size_line.startswith("SIZE "):
                raise RuntimeError(f"Unexpected response: {size_line}")

            image_size = int(size_line.removeprefix("SIZE "))
            self._send_line(self.connection, "OK")
            image = self._receive_exact(self.connection, image_size)
            path = self._save_image(image)
            self._send_line(self.connection, "COMPLETE")

            if self._receive_line(self.connection) != "READY":
                self.close_connection()
            self.logger.event(f"Saved image: {path}")
            return path
        except (ConnectionError, OSError, socket.timeout):
            self.close_connection()
            raise

    def restore_wifi(self) -> None:
        """APを停止し、普段使うWi-Fiへ戻す。"""
        if not self._restore_needed:
            return
        self.close_connection()
        self._run_command("nmcli", "connection", "down", self.ap_connection, check=False)
        result = self._run_command(
            "nmcli",
            "connection",
            "up",
            self.restore_connection,
            "ifname",
            self.wifi_interface,
            check=False,
        )
        self._restore_needed = False
        if result.returncode == 0:
            self.logger.event(f"Restored Wi-Fi connection: {self.restore_connection}")
        else:
            self.logger.event(f"Failed to restore Wi-Fi connection: {self.restore_connection}")

    def _wait_for_esp(self) -> socket.socket:
        """ESP32S3からのTCP接続を待つ。"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.settimeout(self.timeout_sec)
        server_socket.bind((self.host, self.port))
        server_socket.listen(1)
        self.logger.event(f"Waiting for ESP32S3 on TCP port {self.port}")
        try:
            connection, address = server_socket.accept()
            connection.settimeout(self.timeout_sec)
            self.logger.event(f"ESP32S3 connected: {address}")
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
        self.logger.event("+ " + " ".join(command))
        try:
            return subprocess.run(command, text=True, check=check, timeout=COMMAND_TIMEOUT_SEC)
        except subprocess.TimeoutExpired as exc:
            self.logger.event(f"ERROR COMMAND_TIMEOUT: {' '.join(command)}")
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
