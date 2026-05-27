#!/usr/bin/env python3
"""Minimal radio sender entry point for the CanSat rover."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Optional, Protocol
import json


class RadioTransport(Protocol):
    def command(self, command: str, wait: Optional[float] = None) -> str:
        ...


class CommunicationManager:
    """SensorManager-like entry point for sending data with TLM922S."""

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
        from tlm922s_p2p.raspberry_pi_zero_wh.tlm922s_uart import Tlm922sUart

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
