#!/usr/bin/env python3
"""自撮りカメラを扱うための管理クラス。"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import os
import socket
import subprocess
import sys
import threading
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

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 5000
TIMEOUT_SEC = 120.0
PING_TIMEOUT_SEC = 5.0
CAPTURE_TIMEOUT_SEC = 30.0
IMAGE_DIR = Path(SCRIPT_DIR/"raw_images")

BUFFER_SIZE = 16384
COMMAND_TIMEOUT_SEC = 30.0

SELFIE_EV_VALUES = (-1.0, -0.5, 0.0, 0.5, 1.0)

MOTOR_PH_PIN = 6
MOTOR_EN_PIN = 13
MOTOR_SLEEP_PIN = 5
MOTOR_PWM_FREQUENCY_HZ = 1000
ARM_MOTOR_SPEED = 1.0
ARM_EXPAND_SECONDS = 6.0
ARM_RETRACT_SECONDS = 5.5


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
        host: str = SERVER_HOST,
        port: int = SERVER_PORT,
        timeout_sec: float = TIMEOUT_SEC,
        ping_timeout_sec: float = PING_TIMEOUT_SEC,
        capture_timeout_sec: float = CAPTURE_TIMEOUT_SEC,
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
        self.host = host
        self.port = port
        self.timeout_sec = timeout_sec
        self.ping_timeout_sec = ping_timeout_sec
        self.capture_timeout_sec = capture_timeout_sec
        self.image_dir = Path(image_dir)
        self.motor_ph_pin = motor_ph_pin
        self.motor_en_pin = motor_en_pin
        self.motor_sleep_pin = motor_sleep_pin
        self.logger = logger if logger is not None else Logger(log_to_file=False)
        self._restore_needed = False
        self._previous_wifi_connection: str | None = None
        self.connection: socket.socket | None = None
        self.server_socket: socket.socket | None = None
        self._connection_lock = threading.RLock()
        self._connection_event = threading.Event()
        self._server_stop_event = threading.Event()
        self._accept_thread: threading.Thread | None = None

    def __enter__(self) -> "SelfieManager":
        return self

    def __exit__(self, *_: object) -> None:
        self.close_server()
        self.restore_wifi()

    def expand(self) -> None:
        """自撮りカメラを展開する。"""
        self._run_motor(ph_value=True, speed=ARM_MOTOR_SPEED, run_seconds=ARM_EXPAND_SECONDS)

    def retract(self) -> None:
        """自撮りカメラを収納する。"""
        self._run_motor(ph_value=False, speed=ARM_MOTOR_SPEED, run_seconds=ARM_RETRACT_SECONDS)

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
            self.start_server()
            return self.capture_connected()
        finally:
            self.close_server()
            self.restore_wifi()

    def start_ap(self) -> None:
        """ESP32S3が接続するためのラズパイ側APを起動する。"""
        self._ensure_root()
        if not self._restore_needed:
            self._previous_wifi_connection = self._current_wifi_connection()
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
            "wifi-sec.proto",
            "rsn",
            "wifi-sec.pairwise",
            "ccmp",
            "wifi-sec.group",
            "ccmp",
            "wifi-sec.psk",
            self.ap_password,
        )
        self._run_command("nmcli", "connection", "up", self.ap_connection)
        self._restore_needed = True
        self.logger.event(f"AP started: {self.ap_ssid}")

    def start_server(self) -> None:
        """TCPサーバーを起動し、ESP32S3の接続をバックグラウンドで待つ。"""
        if self._accept_thread is not None and self._accept_thread.is_alive():
            return

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.host, self.port))
        server_socket.listen(2)
        server_socket.settimeout(1.0)
        self.server_socket = server_socket
        self._server_stop_event.clear()
        self._accept_thread = threading.Thread(
            target=self._accept_connections,
            name="selfie-tcp-server",
            daemon=True,
        )
        self._accept_thread.start()
        actual_port = server_socket.getsockname()[1]
        self.logger.event(f"Selfie TCP server started on port {actual_port}")

    def wait_connection(self, timeout_sec: float | None = None) -> None:
        """撮影時にESP32S3の接続を指定時間だけ待つ。"""
        self.start_server()
        timeout_sec = self.timeout_sec if timeout_sec is None else float(timeout_sec)
        if not self._connection_event.wait(timeout_sec):
            raise TimeoutError("Timed out waiting for ESP32S3 connection")

        with self._connection_lock:
            if self.connection is None:
                raise ConnectionError("ESP32S3 connection was closed")

    def close_connection(self) -> None:
        """TCP接続だけを閉じる。APは落とさない。"""
        with self._connection_lock:
            if self.connection is not None:
                self.connection.close()
                self.connection = None
            self._connection_event.clear()

    def close_server(self) -> None:
        """TCPサーバーと現在のESP32S3接続を閉じる。"""
        self._server_stop_event.set()
        server_socket = self.server_socket
        self.server_socket = None
        if server_socket is not None:
            server_socket.close()

        thread = self._accept_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=self.ping_timeout_sec + 1.0)
        self._accept_thread = None
        self.close_connection()

    def ping(self) -> bool:
        """撮影前にESP32S3とのTCP接続が生きているか確認する。"""
        with self._connection_lock:
            connection = self.connection
            if connection is None:
                return False
            try:
                connection.settimeout(self.ping_timeout_sec)
                self._send_line(connection, "PING")
                return self._receive_line(connection) == "PONG"
            except (ConnectionError, OSError, socket.timeout):
                return False
            finally:
                try:
                    connection.settimeout(self.timeout_sec)
                except OSError:
                    pass

    def ensure_connection(self) -> None:
        """接続が切れていれば、APは維持したままESP32S3の再接続を待つ。"""
        deadline = time.monotonic() + self.timeout_sec
        while not self.ping():
            self.close_connection()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError("Timed out waiting for ESP32S3 connection")
            self.wait_connection(timeout_sec=remaining)

    def capture_connected(
        self,
        ev: float | None = None,
    ) -> Path:
        """接続済みESP32S3へ撮影を指示し、JPEGを1枚受信する。

        evを省略した場合は従来どおり自動露出で1枚撮影する。
        指定する場合は-1.0、-0.5、0.0、0.5、1.0のいずれかとする。
        """
        if ev is not None:
            ev = float(ev)
            if ev not in SELFIE_EV_VALUES:
                raise ValueError(f"ev must be one of {SELFIE_EV_VALUES}")

        self.ensure_connection()
        with self._connection_lock:
            return self._capture_connected(ev)

    def capture_exposure_series(self) -> list[Path]:
        """露出を変えた5枚を連続撮影し、通信失敗時は1回だけ再試行する。"""
        self.ensure_connection()
        series_dir = self.image_dir / datetime.now().strftime(
            "selfie_%Y%m%d_%H%M%S"
        )
        for attempt in range(1, 3):
            try:
                with self._connection_lock:
                    return [
                        self._capture_connected(ev, save_dir=series_dir)
                        for ev in SELFIE_EV_VALUES
                    ]
            except (ConnectionError, OSError, socket.timeout) as exc:
                if attempt >= 2:
                    raise
                self.logger.event(
                    f"Selfie capture series communication failed; retrying 1/1 "
                    f"({type(exc).__name__}: {exc})"
                )
                self.ensure_connection()

        raise RuntimeError("Selfie capture retry loop ended unexpectedly")

    def _capture_connected(
        self,
        ev: float | None,
        *,
        save_dir: Path | None = None,
    ) -> Path:
        if self.connection is None:
            raise RuntimeError("ESP32S3 is not connected")

        connection = self.connection
        stage = "capture request"
        try:
            connection.settimeout(self.capture_timeout_sec)
            capture_command = (
                "CAPTURE" if ev is None else f"CAPTURE {round(ev * 2):d}"
            )
            self.logger.event(
                f"Selfie capture request: ev={ev}, timeout={self.capture_timeout_sec:.1f}s"
            )
            self._send_line(connection, capture_command)

            stage = "size response"
            size_line = self._receive_line(connection)
            if not size_line.startswith("SIZE "):
                raise RuntimeError(f"Unexpected response: {size_line}")

            image_size = int(size_line.removeprefix("SIZE "))
            self.logger.event(f"Selfie image size received: {image_size} bytes")
            self._send_line(connection, "OK")

            stage = "image data"
            self.logger.event(f"Selfie image receive started: {image_size} bytes")
            image = self._receive_exact(connection, image_size)
            self.logger.event(f"Selfie image receive completed: {len(image)} bytes")
            path = self._save_image(image, ev=ev, save_dir=save_dir)

            stage = "complete notification"
            self._send_line(connection, "COMPLETE")

            stage = "ready response"
            ready_response = self._receive_line(connection)
            self.logger.event(f"Selfie ready response received: {ready_response}")
            if ready_response != "READY":
                self.close_connection()
            self.logger.event(f"Saved image: {path}")
            return path
        except (ConnectionError, OSError, socket.timeout) as exc:
            self.logger.event(
                f"Selfie capture communication failed: ev={ev}, stage={stage} "
                f"({type(exc).__name__}: {exc})"
            )
            self.close_connection()
            raise
        finally:
            try:
                connection.settimeout(self.timeout_sec)
            except OSError:
                pass

    def restore_wifi(self) -> None:
        """APを停止し、普段使うWi-Fiへ戻す。"""
        if not self._restore_needed:
            return
        self.close_server()
        self._run_command("nmcli", "connection", "down", self.ap_connection, check=False)
        restore_connection = self._previous_wifi_connection
        self._restore_needed = False
        self._previous_wifi_connection = None
        if restore_connection is None:
            self.logger.event("No previous Wi-Fi connection to restore")
            return

        result = self._run_command(
            "nmcli",
            "connection",
            "up",
            restore_connection,
            "ifname",
            self.wifi_interface,
            check=False,
        )
        if result.returncode == 0:
            self.logger.event(f"Restored Wi-Fi connection: {restore_connection}")
        else:
            self.logger.event(f"Failed to restore Wi-Fi connection: {restore_connection}")

    def _current_wifi_connection(self) -> str | None:
        """現在wlan0で使っているNetworkManager接続名を返す。"""
        result = subprocess.run(
            ["nmcli", "-g", "GENERAL.CONNECTION", "device", "show", self.wifi_interface],
            capture_output=True,
            text=True,
            check=False,
            timeout=COMMAND_TIMEOUT_SEC,
        )
        if result.returncode != 0:
            return None
        connection = result.stdout.strip()
        if not connection or connection == "--" or connection == self.ap_connection:
            return None
        return connection

    def _accept_connections(self) -> None:
        """ESP32S3の初回接続と再接続を受け付け続ける。"""
        while not self._server_stop_event.is_set():
            server_socket = self.server_socket
            if server_socket is None:
                return
            try:
                connection, address = server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                return

            try:
                connection.settimeout(self.ping_timeout_sec)
                if self._receive_line(connection) != "READY":
                    connection.close()
                    continue
                connection.settimeout(self.timeout_sec)
            except (ConnectionError, OSError, socket.timeout):
                connection.close()
                continue

            with self._connection_lock:
                previous_connection = self.connection
                self.connection = connection
                self._connection_event.set()
            if previous_connection is not None:
                previous_connection.close()
            self.logger.event(f"ESP32S3 connected: {address}")

    def _save_image(
        self,
        image: bytes,
        *,
        ev: float | None = None,
        save_dir: Path | None = None,
    ) -> Path:
        """受信したJPEGを時刻付きファイル名で保存する。"""
        output_dir = self.image_dir if save_dir is None else Path(save_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        ev_suffix = "" if ev is None else f"_ev{ev:+.1f}"
        path = output_dir / f"selfie_{timestamp}{ev_suffix}.jpg"
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
            result = subprocess.run(
                command,
                text=True,
                check=check,
                timeout=COMMAND_TIMEOUT_SEC,
                capture_output=True,
            )
            self._write_command_output(result.stdout, sys.stdout)
            self._write_command_output(result.stderr, sys.stderr)
            return result
        except subprocess.CalledProcessError as exc:
            self._write_command_output(exc.stdout, sys.stdout)
            self._write_command_output(exc.stderr, sys.stderr)
            raise
        except subprocess.TimeoutExpired as exc:
            self._write_command_output(exc.stdout, sys.stdout)
            self._write_command_output(exc.stderr, sys.stderr)
            self.logger.event(f"ERROR COMMAND_TIMEOUT: {' '.join(command)}")
            if check:
                raise
            return subprocess.CompletedProcess(command, 124, "", str(exc))

    @staticmethod
    def _write_command_output(output: str | bytes | None, stream) -> None:
        """外部コマンドの出力を画面とconsoleログへ流す。"""
        if not output:
            return
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        stream.write(output)
        if not output.endswith("\n"):
            stream.write("\n")
        stream.flush()

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
