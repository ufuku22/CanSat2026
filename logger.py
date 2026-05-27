#!/usr/bin/env python3
"""CanSat2026用の簡易ロガー。"""

from __future__ import annotations

from pathlib import Path
from time import monotonic
from typing import Any


class Logger:
    """センサログとイベントログをテキスト形式で保存するクラス。"""

    def __init__(
        self,
        sensor_manager: Any | None = None,
        log_dir: str | Path = "logs",
        filename: str = "log.txt",
    ) -> None:
        self.sensor_manager = sensor_manager
        self.log_path = Path(log_dir) / filename
        self.start_time = monotonic()

    def reset_timer(self) -> None:
        """経過時間の基準を現在時刻に戻す。"""
        self.start_time = monotonic()

    def get_log(self, sensor_data: dict[str, Any] | None = None) -> str:
        """センサ値を1行のログ文字列にして返す。"""
        data = sensor_data or self._read_sensors()
        pressure = data.get("environment", {}).get("pressure_hpa")
        accel = data.get("imu", {}).get("accel_mps2", (None, None, None))
        ax, ay, az = accel[:3] if accel and len(accel) >= 3 else (None, None, None)
        gnss = data.get("gnss") or {}
        lat = gnss.get("latitude_deg")
        lon = gnss.get("longitude_deg")
        alt = gnss.get("altitude_m")

        return (
            f"t:{self._num(self.elapsed_time())} | "
            f"p:{self._num(pressure)} | "
            f"ax:{self._num(ax)} | "
            f"ay:{self._num(ay)} | "
            f"az:{self._num(az)} | "
            f"lat:{self._num(lat, digits=6)} | "
            f"lon:{self._num(lon, digits=6)} | "
            f"alt:{self._num(alt)} |"
        )

    def get_event_log(self, text: str) -> str:
        """任意のイベント文字列を1行のログ文字列にして返す。"""
        return f"t:{self._num(self.elapsed_time())} | event:{text} |"

    def write_sensor(self, sensor_data: dict[str, Any] | None = None) -> Path:
        """センサログをファイルに1行追記する。"""
        return self._write_line(self.get_log(sensor_data))

    def write_event(self, text: str) -> Path:
        """イベントログをファイルに1行追記する。"""
        return self._write_line(self.get_event_log(text))

    def elapsed_time(self) -> float:
        """ロガー起動からの経過時間を秒で返す。"""
        return monotonic() - self.start_time

    def _read_sensors(self) -> dict[str, Any]:
        """SensorManagerから全センサ値を読み取る。"""
        if self.sensor_manager is None:
            raise RuntimeError("sensor_dataを渡さない場合はsensor_managerが必要です。")
        return self.sensor_manager.read_all()

    def _write_line(self, line: str) -> Path:
        """ログファイルに1行追記する。"""
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.log_path.open("a", encoding="utf-8") as file:
            file.write(line + "\n")
        return self.log_path

    @staticmethod
    def _num(value: Any, digits: int = 2) -> str:
        """数値を指定した小数桁数の文字列に変換する。"""
        if value is None:
            return "None"
        return f"{float(value):.{digits}f}"
