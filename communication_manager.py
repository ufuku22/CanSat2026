#!/usr/bin/env python3
"""CanSat 側から TLM922S で地上局へデータを送るための小さな入口。"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional, Protocol
import json
import re
import time

from image_transfer import (
    DEFAULT_MAX_RADIO_PAYLOAD,
    ImageSession as _ImageSession,
    build_image_packets as _build_image_packets,
    session_from_packet_hex as _session_from_packet_hex,
)


IMAGE_PACKET_LINE = re.compile(r"(?:IMG_PACKET|radio_rx)\s+([0-9A-Fa-f]+)")


@dataclass(frozen=True)
class ImageSendResult:
    """画像送信の結果を、呼び出し側がログに残しやすい形でまとめる。"""

    image_path: Path
    file_id: int
    file_size: int
    k: int
    m: int
    block_size: int
    responses: list[str]

    @property
    def radio_tx_ok_count(self) -> int:
        return sum("radio_tx_ok" in response for response in self.responses)

    @property
    def all_radio_tx_ok(self) -> bool:
        return self.radio_tx_ok_count == len(self.responses)


@dataclass(frozen=True)
class ImageReceiveResult:
    """受信側で1行処理した結果。保存できたときだけ saved_path が入る。"""

    file_id: int
    collected: int
    required: int
    total_packets: int
    received_index: int
    saved_path: Path | None = None
    error: str | None = None

    @property
    def saved(self) -> bool:
        return self.saved_path is not None


@dataclass
class ImageReceiveStore:
    """PC側で画像パケットを集め、復元できたらJPEGとして保存する受信箱。"""

    output_dir: Path | str = "received_images"
    sessions: dict[int, _ImageSession] = field(default_factory=dict)
    saved_file_ids: set[int] = field(default_factory=set)

    def __post_init__(self) -> None:
        self.output_dir = Path(self.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def add_line(self, line: str) -> ImageReceiveResult | None:
        """ESP32が出力した1行から画像パケットを拾って処理する。"""
        match = IMAGE_PACKET_LINE.search(line)
        if match is None:
            return None
        return self.add_payload_hex(match.group(1))

    def add_payload_hex(self, payload_hex: str) -> ImageReceiveResult | None:
        """16進文字列の画像パケットを追加し、復元できたら保存する。"""
        try:
            packet = _session_from_packet_hex(payload_hex)
        except ValueError:
            return None

        if packet.file_id in self.saved_file_ids:
            return ImageReceiveResult(
                file_id=packet.file_id,
                collected=packet.k,
                required=packet.k,
                total_packets=packet.m,
                received_index=packet.index,
                saved_path=self.output_dir / f"{packet.file_id:08x}.jpg",
            )

        session = self.sessions.setdefault(packet.file_id, _ImageSession.from_packet(packet))
        try:
            session.add(packet)
        except ValueError as exc:
            return ImageReceiveResult(
                file_id=packet.file_id,
                collected=len(session.blocks),
                required=session.k,
                total_packets=session.m,
                received_index=packet.index,
                error=str(exc),
            )

        result = ImageReceiveResult(
            file_id=packet.file_id,
            collected=len(session.blocks),
            required=session.k,
            total_packets=session.m,
            received_index=packet.index,
        )
        if not session.can_recover():
            return result

        try:
            image = session.recover()
        except ValueError as exc:
            return ImageReceiveResult(
                file_id=packet.file_id,
                collected=len(session.blocks),
                required=session.k,
                total_packets=session.m,
                received_index=packet.index,
                error=str(exc),
            )

        output_path = self.output_dir / f"{packet.file_id:08x}.jpg"
        output_path.write_bytes(image)
        self.saved_file_ids.add(packet.file_id)
        self.sessions.pop(packet.file_id, None)
        return ImageReceiveResult(
            file_id=packet.file_id,
            collected=result.collected,
            required=result.required,
            total_packets=result.total_packets,
            received_index=result.received_index,
            saved_path=output_path,
        )


class RadioTransport(Protocol):
    """TLM922S UART ドライバが持つ最小限の操作だけを表す型。"""

    def command(self, command: str, wait: Optional[float] = None) -> str:
        ...


class CommunicationManager:
    """SensorManager と同じ感覚で使える、通信送信用の管理クラス。"""

    def __init__(
        self,
        port: str = "/dev/serial0",
        baudrate: int = 115200,
        timeout: float = 4.0,
        radio: Optional[RadioTransport] = None,
    ) -> None:
        # radio を外から渡すとテスト用の偽物に差し替えられる。
        # 渡されなかった場合は setup() で実機の UART 接続を開く。
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.radio = radio
        self.sequence = 0
        self._owned_radio: Any = None

    def setup(self) -> None:
        """TLM922S との UART 接続を開く。すでに接続済みなら何もしない。"""
        if self.radio is not None:
            return
        from tlm922s_p2p.raspberry_pi_zero_wh.tlm922s_uart import Tlm922sUart

        self._owned_radio = Tlm922sUart(self.port, self.baudrate, timeout=self.timeout)
        self.radio = self._owned_radio.__enter__()

    def close(self) -> None:
        """このクラスが開いた UART 接続だけを閉じる。"""
        if self._owned_radio is not None:
            self._owned_radio.__exit__(None, None, None)
            self._owned_radio = None
            self.radio = None

    def __enter__(self) -> "CommunicationManager":
        self.setup()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def send_text(self, message: str) -> str:
        return self.send_packet("text", {"message": message})

    def send_gnss(self, gnss: dict[str, Any]) -> str:
        return self.send_packet("gnss", {"gnss": compact_gnss(gnss)})

    def send_telemetry(self, telemetry: dict[str, Any]) -> str:
        return self.send_packet("tlm", compact_telemetry(telemetry))

    def send_image(
        self,
        image_path: str | Path,
        *,
        max_radio_payload: int = DEFAULT_MAX_RADIO_PAYLOAD,
        inter_packet_delay: float = 0.2,
    ) -> ImageSendResult:
        """JPEG画像をFEC付きの複数パケットに分けて送信する。

        ACKなしの最小構成なので、各パケットには復元に必要な情報を毎回入れる。
        地上局PCは任意のk個を受け取れれば、自律的に画像を復元・保存できる。
        """
        if self.radio is None:
            raise RuntimeError("CommunicationManager.setup() must be called before sending.")

        packets = _build_image_packets(image_path, max_radio_payload=max_radio_payload)
        responses: list[str] = []
        for packet in packets:
            responses.append(self.radio.command(f"p2p tx {packet.to_bytes().hex()}", wait=self.timeout))
            if inter_packet_delay > 0:
                # LoRaの送信完了後、受信側が次の待受へ戻る時間を少し確保する。
                time.sleep(inter_packet_delay)
        first = packets[0]
        return ImageSendResult(
            image_path=Path(image_path),
            file_id=first.file_id,
            file_size=first.file_size,
            k=first.k,
            m=first.m,
            block_size=first.block_size,
            responses=responses,
        )

    def send_packet(self, packet_type: str, data: dict[str, Any]) -> str:
        if self.radio is None:
            raise RuntimeError("CommunicationManager.setup() must be called before sending.")

        self.sequence += 1
        # 地上局側が扱いやすいように、全ての送信データを同じ封筒に入れる。
        # v: 形式のバージョン、type: データ種別、seq: 送信順、time: UTC時刻。
        packet = {
            "v": 1,
            "type": packet_type,
            "seq": self.sequence,
            "time": now_iso(),
            "data": normalize(data),
        }
        # TLM922S の p2p tx は 16進文字列を送るため、JSON を UTF-8 bytes にして hex 化する。
        payload_hex = json.dumps(packet, separators=(",", ":")).encode("utf-8").hex()
        return self.radio.command(f"p2p tx {payload_hex}", wait=self.timeout)


def compact_telemetry(telemetry: dict[str, Any]) -> dict[str, Any]:
    """SensorManager の大きな辞書から、無線で送りたい値だけを短いキーで抜き出す。"""
    data: dict[str, Any] = {}
    if "gnss" in telemetry:
        data["gnss"] = compact_gnss(telemetry["gnss"])
    if "environment" in telemetry:
        env = telemetry["environment"]
        data["env"] = {
            "temp": env.get("temperature_c"),
            "press": env.get("pressure_hpa"),
            "hum": env.get("humidity_percent"),
        }
    if "imu" in telemetry:
        imu = telemetry["imu"]
        data["imu"] = {
            "head": imu.get("heading_deg"),
            "roll": imu.get("roll_deg"),
            "pitch": imu.get("pitch_deg"),
            "cal": imu.get("calibration"),
        }
    if "distance_m" in telemetry:
        data["dist"] = telemetry["distance_m"]
    return data


def compact_gnss(gnss: dict[str, Any]) -> dict[str, Any]:
    """GNSS 情報を地上局で見る最低限の項目に絞る。"""
    return {
        "lat": gnss.get("latitude_deg"),
        "lon": gnss.get("longitude_deg"),
        "alt": gnss.get("altitude_m"),
        "sat": gnss.get("satellites"),
        "fix": gnss.get("fix_quality"),
    }


def normalize(value: Any) -> Any:
    """JSON 化しやすい形に整え、float は長くなりすぎないよう丸める。"""
    if isinstance(value, dict):
        return {str(k): normalize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [normalize(v) for v in value]
    if isinstance(value, float):
        return round(value, 6)
    return value


def now_iso() -> str:
    """ログで比較しやすい UTC の ISO8601 文字列を作る。"""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
