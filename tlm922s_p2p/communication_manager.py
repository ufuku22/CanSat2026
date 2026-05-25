#!/usr/bin/env python3
"""TLM922SのP2P通信を扱う高レベル通信クラス。

このファイルは、以下の両方で使うことを想定しています。

* Raspberry Pi Zero WH: TLM922SをUARTで直接接続する送信側。
* ノートPC: ESP32-C3 USBブリッジ経由でTLM922Sに接続する受信側。

pyserialを使うため、Linuxの /dev/serial0 とWindowsの COM5 のような
シリアルポートを同じコードで開けます。
"""

from __future__ import annotations

import base64
import json
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import serial
except ImportError:  # 実行環境にpyserialがない場合は、通信開始時に分かりやすい例外を出す。
    serial = None


DEFAULT_RADIO_BAUDRATE = 115200
DEFAULT_GPS_BAUDRATE = 9600
DEFAULT_RX_WINDOW_MS = 3000
DEFAULT_CHUNK_SIZE = 72
MAX_FRAME_BYTES = 220


@dataclass(frozen=True)
class RadioPacket:
    """TLM922Sで受信した1つの無線パケットを表すデータ。"""

    payload: bytes
    rssi: int | None = None
    snr: int | None = None

    def text(self) -> str:
        """受信したバイト列をUTF-8文字列として読み出す。"""
        return self.payload.decode("utf-8", errors="replace")


@dataclass(frozen=True)
class GpsFix:
    """LC76Gから取得したGPS測位情報を表すデータ。"""

    latitude: float | None
    longitude: float | None
    altitude_m: float | None
    speed_knots: float | None
    course_deg: float | None
    timestamp_utc: str | None
    source_sentence: str

    def as_dict(self) -> dict[str, Any]:
        """GPS測位情報を送信用の辞書形式に変換する。"""
        return {
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude_m": self.altitude_m,
            "speed_knots": self.speed_knots,
            "course_deg": self.course_deg,
            "timestamp_utc": self.timestamp_utc,
            "source_sentence": self.source_sentence,
        }


class CommunicationManager:
    """TLM922Sの接続確認、画像送信、GPS送信をまとめて管理するクラス。"""

    def __init__(
        self,
        radio_port: str,
        radio_baudrate: int = DEFAULT_RADIO_BAUDRATE,
        timeout: float = 1.5,
        rx_window_ms: int = DEFAULT_RX_WINDOW_MS,
        node_id: str | None = None,
    ) -> None:
        self.radio_port = radio_port
        self.radio_baudrate = radio_baudrate
        self.timeout = timeout
        self.rx_window_ms = rx_window_ms
        self.node_id = node_id or uuid.uuid4().hex[:8]
        self.radio: Any | None = None

    def __enter__(self) -> "CommunicationManager":
        """with文で使ったときにシリアルポートを開く。"""
        self.open()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        """with文を抜けるときにシリアルポートを閉じる。"""
        self.close()

    def open(self) -> None:
        """TLM922Sにつながるシリアルポートを開く。"""
        _require_pyserial()
        if self.radio and self.radio.is_open:
            return

        self.radio = serial.Serial(
            port=self.radio_port,
            baudrate=self.radio_baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        self.radio.reset_input_buffer()
        self.radio.reset_output_buffer()

    def close(self) -> None:
        """開いているシリアルポートを閉じる。"""
        if self.radio and self.radio.is_open:
            self.radio.close()

    def command(self, command: str, wait: float | None = None) -> str:
        """TLM922SへASCIIコマンドを送り、一定時間内の応答文字列を返す。"""
        radio = self._require_radio()
        radio.reset_input_buffer()
        radio.write(command.encode("ascii") + b"\r")
        radio.flush()
        return self._read_for(self.timeout if wait is None else wait)

    def establish_connection(self, timeout: float = 15.0) -> bool:
        """ラズパイ側など送信側からHELLOを送り、PC側との接続確認を行う。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            session_id = uuid.uuid4().hex[:8]
            self.send_message("HELLO", {"session_id": session_id, "node_id": self.node_id})

            packet = self.receive_message(window_ms=self.rx_window_ms)
            if not packet:
                continue

            message_type, payload = packet
            if message_type == "HELLO_ACK" and payload.get("session_id") == session_id:
                return True

        return False

    def wait_for_connection(self, timeout: float | None = None) -> dict[str, Any] | None:
        """PC側など受信側でHELLOを待ち、受け取ったらHELLO_ACKを返す。"""
        deadline = None if timeout is None else time.monotonic() + timeout
        while deadline is None or time.monotonic() < deadline:
            packet = self.receive_message(window_ms=self.rx_window_ms)
            if not packet:
                continue

            message_type, payload = packet
            if message_type != "HELLO":
                continue

            self.send_message(
                "HELLO_ACK",
                {
                    "session_id": payload.get("session_id"),
                    "node_id": self.node_id,
                },
            )
            return payload

        return None

    def send_message(self, message_type: str, payload: dict[str, Any] | None = None) -> None:
        """種類付きメッセージをJSON化し、TLM922Sの1パケットとして送信する。"""
        frame = {
            "v": 1,
            "type": message_type,
            "from": self.node_id,
            "payload": payload or {},
        }
        self.send_bytes(json.dumps(frame, separators=(",", ":")).encode("utf-8"))

    def receive_message(self, window_ms: int | None = None) -> tuple[str, dict[str, Any]] | None:
        """TLM922Sで1パケット受信し、JSONメッセージとして取り出す。"""
        packet = self.receive_packet(window_ms=window_ms)
        if not packet:
            return None

        try:
            frame = json.loads(packet.text())
        except json.JSONDecodeError:
            return None

        if frame.get("v") != 1:
            return None

        message_type = frame.get("type")
        payload = frame.get("payload", {})
        if not isinstance(message_type, str) or not isinstance(payload, dict):
            return None

        return message_type, payload

    def send_image_file(
        self,
        image_path: str | Path,
        image_id: str | None = None,
        chunk_size: int = DEFAULT_CHUNK_SIZE,
    ) -> str:
        """画像ファイルを小さなチャンクに分割し、複数パケットで送信する。"""
        if chunk_size <= 0:
            raise ValueError("chunk_sizeは1以上にしてください")

        path = Path(image_path)
        image_bytes = path.read_bytes()
        image_id = image_id or uuid.uuid4().hex
        total_chunks = (len(image_bytes) + chunk_size - 1) // chunk_size

        self.send_message(
            "IMAGE_START",
            {
                "image_id": image_id,
                "filename": path.name,
                "size": len(image_bytes),
                "total_chunks": total_chunks,
            },
        )

        for index in range(total_chunks):
            chunk = image_bytes[index * chunk_size : (index + 1) * chunk_size]
            self.send_message(
                "IMAGE_CHUNK",
                {
                    "image_id": image_id,
                    "index": index,
                    "data": base64.b64encode(chunk).decode("ascii"),
                },
            )

        self.send_message("IMAGE_END", {"image_id": image_id})
        return image_id

    def receive_image_file(
        self,
        output_dir: str | Path,
        timeout: float = 120.0,
        initial_start: dict[str, Any] | None = None,
    ) -> Path | None:
        """分割送信された画像を受信し、結合して指定フォルダへ保存する。"""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        started: dict[str, Any] | None = initial_start
        chunks: dict[int, bytes] = {}
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            packet = self.receive_message(window_ms=self.rx_window_ms)
            if not packet:
                continue

            message_type, payload = packet
            if message_type == "IMAGE_START":
                started = payload
                chunks.clear()
                continue

            if not started:
                continue

            if payload.get("image_id") != started.get("image_id"):
                continue

            if message_type == "IMAGE_CHUNK":
                index = int(payload["index"])
                chunks[index] = base64.b64decode(payload["data"])
                continue

            if message_type == "IMAGE_END":
                total_chunks = int(started["total_chunks"])
                missing = [i for i in range(total_chunks) if i not in chunks]
                if missing:
                    raise RuntimeError(f"画像チャンクが不足しています: {missing}")

                filename = Path(str(started["filename"])).name
                image_bytes = b"".join(chunks[i] for i in range(total_chunks))
                if len(image_bytes) != int(started["size"]):
                    raise RuntimeError("受信した画像サイズがメタデータと一致しません")

                destination = output_path / filename
                destination.write_bytes(image_bytes)
                return destination

        return None

    def read_gps_fix(
        self,
        gps_port: str,
        gps_baudrate: int = DEFAULT_GPS_BAUDRATE,
        timeout: float = 10.0,
    ) -> GpsFix | None:
        """LC76GのNMEA出力を読み、使えるGGA/RMC文からGPS情報を取り出す。"""
        _require_pyserial()
        deadline = time.monotonic() + timeout
        with serial.Serial(gps_port, gps_baudrate, timeout=1.0) as gps:
            while time.monotonic() < deadline:
                line = gps.readline().decode("ascii", errors="ignore").strip()
                fix = parse_nmea_fix(line)
                if fix and fix.latitude is not None and fix.longitude is not None:
                    return fix
        return None

    def send_gps_fix(self, fix: GpsFix) -> None:
        """取得済みのGPS情報をPC側へ送信する。"""
        self.send_message(
            "GPS_FIX",
            {
                "latitude": fix.latitude,
                "longitude": fix.longitude,
                "altitude_m": fix.altitude_m,
                "speed_knots": fix.speed_knots,
                "course_deg": fix.course_deg,
                "timestamp_utc": fix.timestamp_utc,
            },
        )

    def send_current_gps_fix(
        self,
        gps_port: str,
        gps_baudrate: int = DEFAULT_GPS_BAUDRATE,
        timeout: float = 10.0,
    ) -> GpsFix | None:
        """LC76Gから現在のGPS情報を読み取り、そのままPC側へ送信する。"""
        fix = self.read_gps_fix(gps_port, gps_baudrate, timeout)
        if fix:
            self.send_gps_fix(fix)
        return fix

    def send_bytes(self, payload: bytes) -> None:
        """バイト列を16進文字列に変換し、TLM922Sのp2p txで送信する。"""
        if len(payload) > MAX_FRAME_BYTES:
            raise ValueError(
                f"送信データは{len(payload)}バイトです。上限は{MAX_FRAME_BYTES}バイトです。"
                "大きいデータは分割送信してください。"
            )

        response = self.command(f"p2p tx {payload.hex()}", wait=4.0)
        if "radio_tx_ok" not in response:
            raise RuntimeError(f"TLM922Sから送信成功応答が返りませんでした: {response!r}")

    def receive_packet(self, window_ms: int | None = None) -> RadioPacket | None:
        """TLM922Sのp2p rxで1つの無線パケットを受信する。"""
        rx_window_ms = self.rx_window_ms if window_ms is None else window_ms
        response = self.command(
            f"p2p rx {rx_window_ms}",
            wait=(rx_window_ms / 1000.0) + self.timeout,
        )
        return parse_radio_rx(response)

    def _read_for(self, seconds: float) -> str:
        """指定秒数だけシリアル入力を読み続け、受信文字列をまとめて返す。"""
        radio = self._require_radio()
        deadline = time.monotonic() + seconds
        chunks: list[bytes] = []

        while time.monotonic() < deadline:
            waiting = radio.in_waiting
            if waiting:
                chunks.append(radio.read(waiting))
            else:
                time.sleep(0.02)

        return b"".join(chunks).decode("ascii", errors="replace")

    def _require_radio(self) -> Any:
        """シリアルポートが開いているか確認し、開いていなければ例外を出す。"""
        if not self.radio or not self.radio.is_open:
            raise RuntimeError("CommunicationManager is not open")
        return self.radio


def _require_pyserial() -> None:
    """pyserialがインストールされているか確認する。"""
    if serial is None:
        raise RuntimeError(
            "シリアル通信にはpyserialが必要です。"
            "次のコマンドでインストールしてください: python -m pip install pyserial"
        )


def parse_radio_rx(text: str) -> RadioPacket | None:
    """TLM922Sの 'radio_rx' 応答からデータ本体、RSSI、SNRを取り出す。"""
    for line in text.replace("\r", "\n").splitlines():
        line = line.strip()
        if not line.startswith(">> radio_rx "):
            continue

        parts = line.split()
        for index, part in enumerate(parts):
            if _is_hex(part) and index + 2 < len(parts):
                return RadioPacket(
                    payload=bytes.fromhex(part),
                    rssi=_to_int(parts[index + 1]),
                    snr=_to_int(parts[index + 2]),
                )

    return None


def parse_nmea_fix(sentence: str) -> GpsFix | None:
    """NMEAのGGA/RMC文を解析して緯度・経度などのGPS情報に変換する。"""
    if not sentence.startswith("$"):
        return None

    data = sentence.split("*", 1)[0]
    parts = data.split(",")
    sentence_type = parts[0][-3:]

    if sentence_type == "GGA" and len(parts) >= 10:
        return GpsFix(
            latitude=_parse_nmea_coord(parts[2], parts[3]),
            longitude=_parse_nmea_coord(parts[4], parts[5]),
            altitude_m=_to_float(parts[9]),
            speed_knots=None,
            course_deg=None,
            timestamp_utc=parts[1] or None,
            source_sentence=sentence,
        )

    if sentence_type == "RMC" and len(parts) >= 10 and parts[2] == "A":
        return GpsFix(
            latitude=_parse_nmea_coord(parts[3], parts[4]),
            longitude=_parse_nmea_coord(parts[5], parts[6]),
            altitude_m=None,
            speed_knots=_to_float(parts[7]),
            course_deg=_to_float(parts[8]),
            timestamp_utc=parts[1] or None,
            source_sentence=sentence,
        )

    return None


def _parse_nmea_coord(value: str, hemisphere: str) -> float | None:
    """NMEA形式の緯度経度を10進数の度に変換する。"""
    if not value or not hemisphere:
        return None

    dot = value.find(".")
    if dot < 0:
        return None

    degree_digits = dot - 2
    degrees = float(value[:degree_digits])
    minutes = float(value[degree_digits:])
    coord = degrees + minutes / 60.0

    if hemisphere in {"S", "W"}:
        coord *= -1

    return coord


def _is_hex(value: str) -> bool:
    """文字列が偶数桁の16進文字列か確認する。"""
    return bool(value) and len(value) % 2 == 0 and all(
        c in "0123456789abcdefABCDEF" for c in value
    )


def _to_int(value: str) -> int | None:
    """文字列をintへ変換し、失敗したらNoneを返す。"""
    try:
        return int(value)
    except ValueError:
        return None


def _to_float(value: str) -> float | None:
    """文字列をfloatへ変換し、失敗したらNoneを返す。"""
    try:
        return float(value)
    except ValueError:
        return None
