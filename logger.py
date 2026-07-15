#!/usr/bin/env python3
"""CanSat2026用の簡易ロガー。"""

from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from time import monotonic, sleep
from typing import Any, Callable, TypeVar


PROJECT_DIR = Path(__file__).resolve().parent
LOG_DIR = PROJECT_DIR / "logs"
T = TypeVar("T")


class Logger:
    """センサログとイベントログをテキスト形式で保存するクラス。"""

    def __init__(
        self,
        sensor_manager: Any | None = None,
        log_dir: str | Path = LOG_DIR,
        filename: str = "log.txt",
        log_to_file: bool = True,
    ) -> None:
        self.sensor_manager = sensor_manager
        self.log_path = Path(log_dir) / filename
        self.log_to_file = log_to_file
        self.start_time = monotonic()

    def reset_timer(self) -> None:
        """経過時間の基準を現在時刻に戻す。"""
        self.start_time = monotonic()

    def sensor(self, data: str | dict[str, Any] | None = None, value: Any | None = None) -> Path:
        """センサ値を1行のログとして保存する。"""
        if data is None:
            return self._write_line_if_enabled(self._format_sensor_log(self._read_sensors()))
        if isinstance(data, dict):
            if self._is_sensor_bundle(data):
                return self._write_line_if_enabled(self._format_sensor_log(data))
            return self._write_line_if_enabled(self._format_values_log(data))
        return self._write_line_if_enabled(self._format_values_log({data: value}))

    def event(self, message: str) -> Path:
        """イベントを画面に表示し、必要ならログファイルにも保存する。"""
        print(message, flush=True)
        return self._write_line_if_enabled(self._format_event_log(message))

    def step(
        self,
        name: str,
        func: Callable[[], T],
        *,
        retries: int | None = 1,
        retry_delay: float = 0.0,
    ) -> T:
        """処理の開始・完了・失敗をログに残しながら実行する。"""
        # センサ初期化など、失敗したら再試行したい処理をまとめて記録する。
        self.event(f"{name} start")
        last_exc: Exception | None = None
        attempt = 1

        while retries is None or attempt <= max(1, retries):
            try:
                result = func()
            except Exception as exc:
                last_exc = exc
                if retries is None:
                    self.event(f"{name} attempt {attempt} failed: {type(exc).__name__}: {exc}")
                else:
                    self.event(
                        f"{name} attempt {attempt}/{max(1, retries)} failed: "
                        f"{type(exc).__name__}: {exc}"
                    )
                if (retries is None or attempt < max(1, retries)) and retry_delay > 0:
                    sleep(retry_delay)
                attempt += 1
                continue

            self.event(f"{name} complete")
            return result

        if last_exc is None:
            raise RuntimeError(f"{name} failed without an exception")
        self.event(f"{name} failed after {max(1, retries)} attempts")
        raise last_exc

    def elapsed_time(self) -> float:
        """ロガー起動からの経過時間を秒で返す。"""
        return monotonic() - self.start_time

    def _format_sensor_log(self, sensor_data: dict[str, Any]) -> str:
        """センサ値を1行のログ文字列に整形する。"""
        data = sensor_data
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

    def _format_event_log(self, message: str) -> str:
        """イベント文字列を1行のログ文字列に整形する。"""
        return f"t:{self._num(self.elapsed_time())} | event:{message} |"

    def _format_values_log(self, data: dict[str, Any]) -> str:
        """任意の値をkey:value形式のログ文字列に整形する。"""
        fields = " | ".join(f"{key}:{value}" for key, value in data.items())
        return f"t:{self._num(self.elapsed_time())} | {fields} |"

    def _read_sensors(self) -> dict[str, Any]:
        """SensorManagerから全センサ値を読み取る。"""
        if self.sensor_manager is None:
            raise RuntimeError("sensor_dataを渡さない場合はsensor_managerが必要です。")
        return self.sensor_manager.read_all()

    @staticmethod
    def _is_sensor_bundle(data: dict[str, Any]) -> bool:
        """read_all()のような複数センサをまとめた辞書か判定する。"""
        return any(key in data for key in ("environment", "imu", "gnss"))

    def _write_line(self, line: str) -> Path:
        """ログファイルに1行追記する。"""
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.log_path.open("a", encoding="utf-8") as file:
            file.write(line + "\n")
        return self.log_path

    def _write_line_if_enabled(self, line: str) -> Path:
        """ファイル保存が有効な場合だけログファイルに追記する。"""
        if not self.log_to_file:
            return self.log_path
        return self._write_line(line)

    @staticmethod
    def _num(value: Any, digits: int = 2) -> str:
        """数値を指定した小数桁数の文字列に変換する。"""
        if value is None:
            return "None"
        return f"{float(value):.{digits}f}"


class CsvLogger:
    """BME280、BNO055、TSD20の値を共通形式のCSVへ保存する。"""

    FIELDS = [
        "timestamp",
        "elapsed_s",
        "temperature_c",
        "pressure_hpa",
        "humidity_percent",
        "heading_deg",
        "roll_deg",
        "pitch_deg",
        "accel_x_mps2",
        "accel_y_mps2",
        "accel_z_mps2",
        "gyro_x_dps",
        "gyro_y_dps",
        "gyro_z_dps",
        "calibration",
        "distance_m",
        "error",
    ]

    def __init__(self, sensor_manager: Any, output_path: str | Path) -> None:
        self.sensor_manager = sensor_manager
        self.output_path = Path(output_path)
        self.start_time = monotonic()
        self._file = None
        self._writer = None

    @staticmethod
    def setup_sensors(sensor_manager: Any) -> None:
        from sensor_manager import BME280_ADDR

        sensor_manager.environment.setup()
        sensor_manager.bus.write_byte_data(BME280_ADDR, 0xF5, 0x20)
        sensor_manager.imu.setup()
        sensor_manager.distance.setup()

    def __enter__(self) -> "CsvLogger":
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.output_path.open("w", newline="", encoding="utf-8-sig")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDS)
        self._writer.writeheader()
        self.start_time = monotonic()
        return self

    def __exit__(self, *_: object) -> None:
        if self._file is not None:
            self._file.close()
        self._file = None
        self._writer = None

    def write_row(self) -> dict[str, Any]:
        if self._writer is None or self._file is None:
            raise RuntimeError("CsvLogger is not open")

        row = self._read_row()
        self._writer.writerow(row)
        self._file.flush()
        return row

    def _read_row(self) -> dict[str, Any]:
        row: dict[str, Any] = {
            field: "" for field in self.FIELDS
        }
        row["timestamp"] = datetime.now().isoformat(timespec="milliseconds")
        row["elapsed_s"] = f"{monotonic() - self.start_time:.3f}"
        errors: list[str] = []

        try:
            environment = self.sensor_manager.get_environment()
            row["temperature_c"] = environment.get("temperature_c", "")
            row["pressure_hpa"] = environment.get("pressure_hpa", "")
            row["humidity_percent"] = environment.get("humidity_percent", "")
        except Exception as exc:
            errors.append(f"BME280 {type(exc).__name__}: {exc}")

        try:
            imu = self.sensor_manager.get_imu()
            accel = imu.get("accel_mps2") or ("", "", "")
            gyro = imu.get("gyro_dps") or ("", "", "")
            row["heading_deg"] = imu.get("heading_deg", "")
            row["roll_deg"] = imu.get("roll_deg", "")
            row["pitch_deg"] = imu.get("pitch_deg", "")
            row["accel_x_mps2"], row["accel_y_mps2"], row["accel_z_mps2"] = accel[:3]
            row["gyro_x_dps"], row["gyro_y_dps"], row["gyro_z_dps"] = gyro[:3]
            row["calibration"] = imu.get("calibration", "")
        except Exception as exc:
            errors.append(f"BNO055 {type(exc).__name__}: {exc}")

        try:
            distance_m = self.sensor_manager.get_distance_m()
            row["distance_m"] = "" if distance_m is None else distance_m
        except Exception as exc:
            errors.append(f"TSD20 {type(exc).__name__}: {exc}")

        row["error"] = " | ".join(errors)
        return row
