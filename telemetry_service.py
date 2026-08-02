#!/usr/bin/env python3
"""ミッション中のテレメトリをバックグラウンド送信する。"""

from __future__ import annotations

from pathlib import Path
import threading
from typing import Any

from communication_manager import CommunicationManager, ImageSendResult
from logger import Logger
from sensor_manager import SensorManager


class TelemetryService:
    """定期テレメトリと通常の無線送信を直列に実行する。"""

    def __init__(
        self,
        sensors: SensorManager,
        logger: Logger,
        *,
        interval_s: float,
        communication: CommunicationManager | None = None,
    ) -> None:
        self.sensors = sensors
        self.logger = logger
        self.interval_s = float(interval_s)
        self.communication = communication or CommunicationManager(logger=logger)
        self.phase = "startup"
        self._send_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="telemetry",
            daemon=True,
        )
        self._thread.start()

    def set_phase(self, phase: str) -> None:
        self.phase = str(phase)

    def send_text(self, message: str) -> bool:
        try:
            with self._send_lock:
                self.communication.setup()
                response = self.communication.send_text(message)
            return "radio_tx_ok" in response
        except Exception as exc:
            self.logger.event(
                f"無線テキスト送信失敗 ({type(exc).__name__}: {exc})"
            )
            return False

    def send_image(self, image_path: str | Path) -> ImageSendResult | None:
        try:
            with self._send_lock:
                self.communication.setup()
                return self.communication.send_image(image_path)
        except Exception as exc:
            self.logger.event(
                f"画像送信失敗 ({type(exc).__name__}: {exc})"
            )
            return None

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        with self._send_lock:
            self.communication.close()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                telemetry: dict[str, Any] = self.sensors.read_all()
                telemetry["phase"] = self.phase
                with self._send_lock:
                    self.communication.setup()
                    self.communication.send_telemetry(telemetry)
            except Exception as exc:
                self.logger.event(
                    f"テレメトリ送信失敗 ({type(exc).__name__}: {exc})"
                )

            if self._stop_event.wait(self.interval_s):
                break
