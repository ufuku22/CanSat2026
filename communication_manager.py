#!/usr/bin/env python3
"""TLM922S-PO1 を使った PC と Raspberry Pi Zero WH 間の通信管理クラス。

このクラスは以下の用途を想定しています。

* PC 側: USB シリアルなどで TLM922S-PO1 に接続して受信する
* Raspberry Pi 側: UART などで TLM922S-PO1 に接続して送信する
* GPS: Raspberry Pi に接続した LC76G から NMEA 文を読み取る

TLM922S-PO1 の P2P 通信では一度に送れるデータ量が小さいため、画像は小さな
チャンクに分割して送信します。
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
except ImportError:
    # pyserial が入っていない環境でも import だけは成功させ、
    # 実際にシリアル通信を始めるタイミングで分かりやすい例外を出します。
    serial = None


DEFAULT_RADIO_BAUDRATE = 115200
DEFAULT_RX_WINDOW_MS = 3000
DEFAULT_CHUNK_SIZE = 72
MAX_FRAME_BYTES = 220
LC76G_CMD_ADDR = 0x50
LC76G_READ_ADDR = 0x54
LC76G_MAX_NMEA_BYTES = 1024


@dataclass(frozen=True)
class RadioPacket:
    """TLM922S-PO1 から受信した 1 パケット分のデータ。"""

    payload: bytes
    rssi: int | None = None
    snr: int | None = None

    def text(self) -> str:
        """受信データを UTF-8 文字列として取り出します。"""
        return self.payload.decode("utf-8", errors="replace")


@dataclass(frozen=True)
class GpsFix:
    """LC76G から取得した GPS 測位情報。"""

    latitude_deg: float | None
    longitude_deg: float | None
    altitude_m: float | None
    satellites: int | None
    fix_quality: int | None
    raw: str = ""

    def as_dict(self) -> dict[str, Any]:
        """無線送信用に辞書形式へ変換します。"""
        return {
            "latitude_deg": self.latitude_deg,
            "longitude_deg": self.longitude_deg,
            "altitude_m": self.altitude_m,
            "satellites": self.satellites,
            "fix_quality": self.fix_quality,
            "raw": self.raw,
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "GpsFix":
        """受信した GPS 辞書から GpsFix を復元します。"""
        return cls(
            latitude_deg=_to_float_or_none(payload.get("latitude_deg")),
            longitude_deg=_to_float_or_none(payload.get("longitude_deg")),
            altitude_m=_to_float_or_none(payload.get("altitude_m")),
            satellites=_to_int_or_none(payload.get("satellites")),
            fix_quality=_to_int_or_none(payload.get("fix_quality")),
            raw=_to_str_or_none(payload.get("raw")) or "",
        )


class CommunicationManager:
    """TLM922S-PO1 の接続確認、画像送信、GPS 送信をまとめて扱うクラス。"""

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
        """with 文で使うときにシリアルポートを開きます。"""
        self.open()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        """with 文を抜けるときにシリアルポートを閉じます。"""
        self.close()

    def open(self) -> None:
        """TLM922S-PO1 に接続しているシリアルポートを開きます。"""
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
        """開いているシリアルポートを閉じます。"""
        if self.radio and self.radio.is_open:
            self.radio.close()

    def command(self, command: str, wait: float | None = None) -> str:
        """TLM922S-PO1 に ASCII コマンドを送り、一定時間分の応答を返します。"""
        radio = self._require_radio()
        radio.reset_input_buffer()
        radio.write(command.encode("ascii") + b"\r")
        radio.flush()
        return self._read_for(self.timeout if wait is None else wait)

    def establish_connection(self, timeout: float = 15.0) -> bool:
        """相手ノードへ HELLO を送り、HELLO_ACK が返れば接続成功とします。"""
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
        """相手からの HELLO を待ち、受信したら HELLO_ACK を返します。"""
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
        """種類付きメッセージを JSON 化し、TLM922S-PO1 の 1 パケットで送ります。"""
        frame = {
            "v": 1,
            "type": message_type,
            "from": self.node_id,
            "payload": payload or {},
        }
        data = json.dumps(frame, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
        self.send_bytes(data)

    def receive_message(self, window_ms: int | None = None) -> tuple[str, dict[str, Any]] | None:
        """1 パケットを受信し、JSON メッセージとして取り出します。"""
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
        """画像ファイルを分割し、複数パケットで送信します。"""
        if chunk_size <= 0:
            raise ValueError("chunk_size は 1 以上にしてください")

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
        """分割送信された画像を受信し、結合して指定フォルダへ保存します。"""
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
        bus: Any,
        timeout: float = 10.0,
    ) -> GpsFix | None:
        """LC76G の I2C 出力を読み、有効な GGA/RMC 文から GPS 情報を取得します。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw = read_lc76g_nmea(bus)
            fix = parse_nmea_fix(raw)
            if fix and fix.latitude_deg is not None and fix.longitude_deg is not None:
                return fix
            time.sleep(0.1)
        return None

    def send_gps_fix(self, fix: GpsFix) -> None:
        """取得済みの GPS 情報を PC 側へ送信します。"""
        # 生の NMEA 文は長いため、無線では SensorManager と同じ主要項目だけを送ります。
        self.send_message(
            "GPS_FIX",
            {
                "latitude_deg": fix.latitude_deg,
                "longitude_deg": fix.longitude_deg,
                "altitude_m": fix.altitude_m,
                "satellites": fix.satellites,
                "fix_quality": fix.fix_quality,
            },
        )

    def send_current_gps_fix(
        self,
        bus: Any,
        timeout: float = 10.0,
    ) -> GpsFix | None:
        """LC76G から現在の GPS 情報を読み取り、そのまま PC 側へ送信します。"""
        fix = self.read_gps_fix(bus, timeout)
        if fix:
            self.send_gps_fix(fix)
        return fix

    def receive_gps_fix(self, timeout: float = 30.0) -> GpsFix | None:
        """PC 側で GPS_FIX メッセージを待ち、受信できたら GpsFix として返します。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            packet = self.receive_message(window_ms=self.rx_window_ms)
            if not packet:
                continue

            message_type, payload = packet
            if message_type == "GPS_FIX":
                return GpsFix.from_dict(payload)

        return None

    def send_bytes(self, payload: bytes) -> None:
        """バイト列を 16 進文字列に変換し、TLM922S-PO1 の p2p tx で送信します。"""
        if len(payload) > MAX_FRAME_BYTES:
            raise ValueError(
                f"送信データは {len(payload)} バイトです。上限は {MAX_FRAME_BYTES} バイトです。"
                "大きいデータは分割して送信してください。"
            )

        response = self.command(f"p2p tx {payload.hex()}", wait=4.0)
        if "radio_tx_ok" not in response:
            raise RuntimeError(f"TLM922S-PO1 から送信成功応答が返りませんでした: {response!r}")

    def receive_packet(self, window_ms: int | None = None) -> RadioPacket | None:
        """TLM922S-PO1 の p2p rx で 1 パケットを受信します。"""
        rx_window_ms = self.rx_window_ms if window_ms is None else window_ms
        response = self.command(
            f"p2p rx {rx_window_ms}",
            wait=(rx_window_ms / 1000.0) + self.timeout,
        )
        return parse_radio_rx(response)

    def _read_for(self, seconds: float) -> str:
        """指定秒数だけシリアル入力を読み続け、受信文字列をまとめて返します。"""
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
        """シリアルポートが開いていることを確認します。"""
        if not self.radio or not self.radio.is_open:
            raise RuntimeError("CommunicationManager is not open")
        return self.radio


def _require_pyserial() -> None:
    """pyserial が利用可能か確認します。"""
    if serial is None:
        raise RuntimeError(
            "シリアル通信には pyserial が必要です。"
            "次のコマンドでインストールしてください: python -m pip install pyserial"
        )


def parse_radio_rx(text: str) -> RadioPacket | None:
    """TLM922S-PO1 の 'radio_rx' 応答からデータ本体、RSSI、SNR を取り出します。"""
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


def read_lc76g_nmea(bus: Any) -> str:
    """LC76G の Quectel I2C 仕様に従い、蓄積された NMEA 文字列を読み出します。"""
    length = _read_lc76g_length(bus)
    if length <= 0:
        return ""

    length = min(length, LC76G_MAX_NMEA_BYTES)
    _write_lc76g_words(bus, 0xAA512000, length)

    data: list[int] = []
    while len(data) < length:
        block_size = min(32, length - len(data))
        data += bus.read_i2c_block_data(LC76G_READ_ADDR, 0x00, block_size)

    return bytes(data).decode("ascii", errors="ignore").replace("\x00", "")


def parse_nmea_fix(nmea_text: str) -> GpsFix | None:
    """NMEA の GGA/RMC 文を解析し、SensorManager と同じ GPS 情報へ変換します。"""
    if not nmea_text:
        return None

    fix = GpsFix(
        latitude_deg=None,
        longitude_deg=None,
        altitude_m=None,
        satellites=None,
        fix_quality=None,
        raw=nmea_text,
    )

    for line in nmea_text.splitlines():
        data = line.split("*", 1)[0]
        parts = data.split(",")
        sentence_type = parts[0][-3:] if parts and parts[0].startswith("$") else ""

        if sentence_type == "GGA" and len(parts) > 9:
            fix = GpsFix(
                latitude_deg=_parse_nmea_coord(parts[2], parts[3]),
                longitude_deg=_parse_nmea_coord(parts[4], parts[5]),
                altitude_m=_to_float(parts[9]),
                satellites=_to_int(parts[7]),
                fix_quality=_to_int(parts[6]),
                raw=nmea_text,
            )
            continue

        if sentence_type == "RMC" and len(parts) > 6 and parts[2] == "A":
            fix = GpsFix(
                latitude_deg=fix.latitude_deg or _parse_nmea_coord(parts[3], parts[4]),
                longitude_deg=fix.longitude_deg or _parse_nmea_coord(parts[5], parts[6]),
                altitude_m=fix.altitude_m,
                satellites=fix.satellites,
                fix_quality=fix.fix_quality,
                raw=nmea_text,
            )

    if fix.latitude_deg is None and fix.longitude_deg is None and fix.raw == nmea_text:
        return fix if any(line.startswith("$") for line in nmea_text.splitlines()) else None

    return fix


def _read_lc76g_length(bus: Any) -> int:
    """LC76G の送信バッファにある NMEA バイト数を読みます。"""
    _write_lc76g_words(bus, 0xAA510008, 4)
    data = bus.read_i2c_block_data(LC76G_READ_ADDR, 0x00, 4)
    return data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)


def _write_lc76g_words(bus: Any, word1: int, word2: int) -> None:
    """LC76G の I2C コマンドアドレスへ 32bit word を 2 個送ります。"""
    data = list(word1.to_bytes(4, "little") + word2.to_bytes(4, "little"))
    bus.write_i2c_block_data(LC76G_CMD_ADDR, data[0], data[1:])
    time.sleep(0.01)


def _parse_nmea_coord(value: str, hemisphere: str) -> float | None:
    """NMEA の ddmm.mmmm / dddmm.mmmm 形式を 10 進度へ変換します。"""
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
    """文字列が偶数桁の 16 進文字列か確認します。"""
    return bool(value) and len(value) % 2 == 0 and all(
        c in "0123456789abcdefABCDEF" for c in value
    )


def _to_int(value: str) -> int | None:
    """文字列を int へ変換し、失敗したら None を返します。"""
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _to_float(value: str) -> float | None:
    """文字列を float へ変換し、失敗したら None を返します。"""
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _to_float_or_none(value: Any) -> float | None:
    """受信ペイロードの値を float または None に整えます。"""
    if value is None:
        return None
    return _to_float(str(value))


def _to_int_or_none(value: Any) -> int | None:
    """受信ペイロードの値を int または None に整えます。"""
    if value is None:
        return None
    return _to_int(str(value))


def _to_str_or_none(value: Any) -> str | None:
    """受信ペイロードの値を str または None に整えます。"""
    if value is None:
        return None
    return str(value)
