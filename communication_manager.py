#!/usr/bin/env python3
"""Small CanSat-side interface for sending data through TLM922S P2P."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional, Protocol
import json
import time

from image_transfer import DEFAULT_MAX_RADIO_PAYLOAD, build_image_packets


BAUDRATES = {9600, 19200, 57600, 115200}


@dataclass(frozen=True)
class ImageSendResult:
    """Result summary for a JPEG image transfer."""

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


class RadioTransport(Protocol):
    """Minimal interface provided by the TLM922S UART driver."""

    def command(self, command: str, wait: Optional[float] = None) -> str:
        ...


class Tlm922sUart:
    """UART driver for TLM922S ASCII commands."""

    def __init__(self, port: str = "/dev/serial0", baudrate: int = 115200, timeout: float = 1.5) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.fd: int | None = None

    def __enter__(self) -> "Tlm922sUart":
        if self.baudrate not in BAUDRATES:
            raise ValueError(f"Unsupported baudrate: {self.baudrate}")

        import os
        import termios

        baud = getattr(termios, f"B{self.baudrate}")
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = baud
        attrs[5] = baud
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        if self.fd is None:
            return

        import os

        os.close(self.fd)
        self.fd = None

    def send_command(self, command: str) -> None:
        if self.fd is None:
            raise RuntimeError("UART is not open.")

        import os
        import select
        import termios

        data = command.encode("ascii") + b"\r"
        written = 0
        while written < len(data):
            _, ready, _ = select.select([], [self.fd], [], self.timeout)
            if not ready:
                raise TimeoutError("Timed out waiting for UART to become writable.")

            try:
                count = os.write(self.fd, data[written:])
            except BlockingIOError:
                continue

            if count == 0:
                raise OSError("UART write returned 0 bytes.")
            written += count

        termios.tcdrain(self.fd)

    def read_for(self, seconds: float) -> str:
        if self.fd is None:
            raise RuntimeError("UART is not open.")

        import os
        import select

        end_time = time.monotonic() + seconds
        chunks: list[bytes] = []
        while time.monotonic() < end_time:
            ready, _, _ = select.select([self.fd], [], [], 0.05)
            if not ready:
                continue

            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                continue

            if data:
                chunks.append(data)

        return b"".join(chunks).decode("ascii", errors="replace")

    def command(self, command: str, wait: Optional[float] = None) -> str:
        if self.fd is None:
            raise RuntimeError("UART is not open.")

        import termios

        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.send_command(command)
        return self.read_for(self.timeout if wait is None else wait)


class CommunicationManager:
    """CanSat-side sender for telemetry, text, GNSS, and JPEG images."""

    def __init__(
        self,
        port: str = "/dev/serial0",
        baudrate: int = 115200,
        timeout: float = 4.0,
        radio: Optional[RadioTransport] = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.radio = radio
        self.sequence = 0
        self._owned_radio: Any = None

    def setup(self) -> None:
        if self.radio is not None:
            return

        self._owned_radio = Tlm922sUart(self.port, self.baudrate, timeout=self.timeout)
        self.radio = self._owned_radio.__enter__()

    def close(self) -> None:
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
        if self.radio is None:
            raise RuntimeError("CommunicationManager.setup() must be called before sending.")

        packets = build_image_packets(image_path, max_radio_payload=max_radio_payload)
        responses: list[str] = []
        for packet in packets:
            responses.append(self.radio.command(f"p2p tx {packet.to_bytes().hex()}", wait=self.timeout))
            if inter_packet_delay > 0:
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
        packet = {
            "v": 1,
            "type": packet_type,
            "seq": self.sequence,
            "time": now_iso(),
            "data": normalize(data),
        }
        payload_hex = json.dumps(packet, separators=(",", ":")).encode("utf-8").hex()
        return self.radio.command(f"p2p tx {payload_hex}", wait=self.timeout)


def compact_telemetry(telemetry: dict[str, Any]) -> dict[str, Any]:
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
    return {
        "lat": gnss.get("latitude_deg"),
        "lon": gnss.get("longitude_deg"),
        "alt": gnss.get("altitude_m"),
        "sat": gnss.get("satellites"),
        "fix": gnss.get("fix_quality"),
    }


def normalize(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): normalize(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [normalize(v) for v in value]
    if isinstance(value, float):
        return round(value, 6)
    return value


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
