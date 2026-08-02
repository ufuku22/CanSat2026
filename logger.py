#!/usr/bin/env python3
"""CanSat2026用の簡易ロガー。"""

from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
import threading
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
    """呼び出し側が選択したセンサ値をCSVへ保存する。"""

    COMMON_FIELDS = ("timestamp", "elapsed_s")
    GNSS_FIELDS = (
        "latitude_deg",
        "longitude_deg",
        "gnss_altitude_m",
        "ground_speed_mps",
        "gnss_has_fix",
        "satellites",
        "fix_quality",
    )
    ENVIRONMENT_FIELDS = (
        "temperature_c",
        "pressure_hpa",
        "humidity_percent",
    )
    IMU_FIELDS = (
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
    )
    DISTANCE_FIELDS = ("distance_m",)
    SENSOR_FIELDS = (
        *ENVIRONMENT_FIELDS,
        *IMU_FIELDS,
        *DISTANCE_FIELDS,
    )

    def __init__(
        self,
        sensor_manager: Any,
        output_path: str | Path,
        *,
        record_fields: list[str] | tuple[str, ...],
        start_time: float | None = None,
    ) -> None:
        self.sensor_manager = sensor_manager
        self.output_path = Path(output_path)
        self.record_fields = tuple(str(field) for field in record_fields)
        if len(set(self.record_fields)) != len(self.record_fields):
            raise ValueError("record_fields must not contain duplicates")
        reserved_fields = set(self.COMMON_FIELDS) | {"error"}
        if reserved_fields.intersection(self.record_fields):
            raise ValueError(
                "timestamp, elapsed_s, and error are added automatically"
            )
        self.fields = [*self.COMMON_FIELDS, *self.record_fields, "error"]
        self.start_time = monotonic() if start_time is None else float(start_time)
        self._reset_start_time_on_open = start_time is None
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
        self._writer = csv.DictWriter(self._file, fieldnames=self.fields)
        self._writer.writeheader()
        if self._reset_start_time_on_open:
            self.start_time = monotonic()
        return self

    def __exit__(self, *_: object) -> None:
        if self._file is not None:
            self._file.close()
        self._file = None
        self._writer = None

    def write_row(
        self,
        extra_values: (
            dict[str, Any]
            | Callable[[dict[str, Any]], dict[str, Any]]
            | None
        ) = None,
    ) -> dict[str, Any]:
        if self._writer is None or self._file is None:
            raise RuntimeError("CsvLogger is not open")

        row = self._read_row()
        if callable(extra_values):
            provided_values = extra_values(row)
        elif extra_values is not None:
            provided_values = extra_values
        else:
            provided_values = {}
        unknown_fields = set(provided_values) - set(row)
        if unknown_fields:
            names = ", ".join(sorted(unknown_fields))
            raise ValueError(f"Values were provided for unrecorded fields: {names}")
        row.update(provided_values)
        self._writer.writerow(row)
        self._file.flush()
        return row

    def _read_row(self) -> dict[str, Any]:
        row: dict[str, Any] = {
            field: "" for field in self.fields
        }
        row["timestamp"] = datetime.now().isoformat(timespec="milliseconds")
        row["elapsed_s"] = f"{monotonic() - self.start_time:.3f}"
        errors: list[str] = []

        if self._records_any(self.GNSS_FIELDS):
            try:
                gnss = self.sensor_manager.get_gnss()
                self._set_if_recorded(
                    row,
                    "latitude_deg",
                    self._blank_if_none(gnss.get("latitude_deg")),
                )
                self._set_if_recorded(
                    row,
                    "longitude_deg",
                    self._blank_if_none(gnss.get("longitude_deg")),
                )
                self._set_if_recorded(
                    row,
                    "gnss_altitude_m",
                    self._blank_if_none(gnss.get("altitude_m")),
                )
                self._set_if_recorded(
                    row,
                    "ground_speed_mps",
                    self._blank_if_none(gnss.get("ground_speed_mps")),
                )
                self._set_if_recorded(
                    row,
                    "gnss_has_fix",
                    bool(gnss.get("has_fix")),
                )
                self._set_if_recorded(
                    row,
                    "satellites",
                    self._blank_if_none(gnss.get("satellites")),
                )
                self._set_if_recorded(
                    row,
                    "fix_quality",
                    self._blank_if_none(gnss.get("fix_quality")),
                )
            except Exception as exc:
                self._set_if_recorded(row, "gnss_has_fix", False)
                errors.append(f"GNSS {type(exc).__name__}: {exc}")

        if self._records_any(self.ENVIRONMENT_FIELDS):
            try:
                environment = self.sensor_manager.get_environment()
                for field in self.ENVIRONMENT_FIELDS:
                    self._set_if_recorded(row, field, environment.get(field, ""))
            except Exception as exc:
                errors.append(f"BME280 {type(exc).__name__}: {exc}")

        if self._records_any(self.IMU_FIELDS):
            try:
                imu = self.sensor_manager.get_imu()
                accel = imu.get("accel_mps2") or ("", "", "")
                gyro = imu.get("gyro_dps") or ("", "", "")
                imu_values = {
                    "heading_deg": imu.get("heading_deg", ""),
                    "roll_deg": imu.get("roll_deg", ""),
                    "pitch_deg": imu.get("pitch_deg", ""),
                    "accel_x_mps2": accel[0],
                    "accel_y_mps2": accel[1],
                    "accel_z_mps2": accel[2],
                    "gyro_x_dps": gyro[0],
                    "gyro_y_dps": gyro[1],
                    "gyro_z_dps": gyro[2],
                    "calibration": imu.get("calibration", ""),
                }
                for field, value in imu_values.items():
                    self._set_if_recorded(row, field, value)
            except Exception as exc:
                errors.append(f"BNO055 {type(exc).__name__}: {exc}")

        if self._records_any(self.DISTANCE_FIELDS):
            try:
                distance_m = self.sensor_manager.get_distance_m()
                self._set_if_recorded(
                    row,
                    "distance_m",
                    self._blank_if_none(distance_m),
                )
            except Exception as exc:
                errors.append(f"TSD20 {type(exc).__name__}: {exc}")

        row["error"] = " | ".join(errors)
        return row

    @staticmethod
    def _blank_if_none(value: Any) -> Any:
        return "" if value is None else value

    def _records_any(self, fields: tuple[str, ...]) -> bool:
        return any(field in self.record_fields for field in fields)

    @staticmethod
    def _set_if_recorded(row: dict[str, Any], field: str, value: Any) -> None:
        if field in row:
            row[field] = value


class PeriodicCsvLogger:
    """CsvLoggerを一定周期でバックグラウンド実行する。"""

    def __init__(
        self,
        csv_logger: CsvLogger,
        *,
        interval_s: float,
        values_provider: Callable[[dict[str, Any]], dict[str, Any]] | None = None,
        status_callback: Callable[[str], Any] | None = None,
    ) -> None:
        self.csv_logger = csv_logger
        self.interval_s = float(interval_s)
        if self.interval_s <= 0.0:
            raise ValueError("interval_s must be greater than 0")
        self.values_provider = values_provider
        self.status_callback = status_callback
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._is_open = False

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return

        self.csv_logger.__enter__()
        self._is_open = True
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="control-history",
            daemon=True,
        )
        try:
            self._thread.start()
        except Exception:
            self._thread = None
            self.csv_logger.__exit__(None, None, None)
            self._is_open = False
            raise

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        if self._is_open:
            self.csv_logger.__exit__(None, None, None)
            self._is_open = False

    def _run(self) -> None:
        while not self._stop_event.is_set():
            started_at = monotonic()
            try:
                self.csv_logger.write_row(self.values_provider)
            except Exception as exc:
                if self.status_callback is not None:
                    try:
                        self.status_callback(
                            f"制御履歴記録失敗 ({type(exc).__name__}: {exc})"
                        )
                    except Exception:
                        pass

            wait_s = max(0.0, self.interval_s - (monotonic() - started_at))
            if self._stop_event.wait(wait_s):
                break


class GnssNavigationCsvLogger:
    """GNSS取得値とGPS誘導時の方位を1つのCSVへ保存する。

    SensorManagerの代理としてNavigationControllerへ渡す。
    get_gnss()ではGNSSを1回だけ取得して一時保存し、
    record_navigation()で、その取得値と誘導計算結果を1行にまとめる。
    """

    FIELDS = [
        "start_latitude_deg",
        "start_longitude_deg",
        "goal_latitude_deg",
        "goal_longitude_deg",
        "timestamp",
        "elapsed_s",
        "latitude_deg",
        "longitude_deg",
        "altitude_m",
        "distance_to_goal_m",
        "heading_deg",
    ]

    def __init__(
        self,
        sensor_manager: Any,
        output_path: str | Path,
        goal_latitude_deg: float,
        goal_longitude_deg: float,
    ) -> None:
        self.sensor_manager = sensor_manager
        self.output_path = Path(output_path)
        self.goal_latitude_deg = float(goal_latitude_deg)
        self.goal_longitude_deg = float(goal_longitude_deg)
        self.start_time = monotonic()
        self.start_latitude_deg: float | None = None
        self.start_longitude_deg: float | None = None
        self._pending_sample: dict[str, Any] | None = None
        self._file = None
        self._writer = None

    def __enter__(self) -> "GnssNavigationCsvLogger":
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.output_path.open(
            "w",
            newline="",
            encoding="utf-8-sig",
        )
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDS)
        self._writer.writeheader()
        self._file.flush()
        self.start_time = monotonic()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def __getattr__(self, name: str) -> Any:
        """GNSS記録以外の処理を元のSensorManagerへ渡す。"""
        return getattr(self.sensor_manager, name)

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
        self._file = None
        self._writer = None

    def get_gnss(self) -> dict[str, Any]:
        """GNSSを1回取得し、誘導結果と組み合わせるまで一時保存する。"""
        gnss = self.sensor_manager.get_gnss()
        acquired_at = datetime.now().isoformat(timespec="milliseconds")
        elapsed_s = monotonic() - self.start_time

        latitude = gnss.get("latitude_deg")
        longitude = gnss.get("longitude_deg")
        has_position = (
            bool(gnss.get("has_fix"))
            and latitude is not None
            and longitude is not None
        )

        if has_position:
            latitude = float(latitude)
            longitude = float(longitude)
            if self.start_latitude_deg is None:
                self.start_latitude_deg = latitude
                self.start_longitude_deg = longitude

            self._pending_sample = {
                "timestamp": acquired_at,
                "elapsed_s": elapsed_s,
                "latitude_deg": latitude,
                "longitude_deg": longitude,
                "altitude_m": gnss.get("altitude_m"),
            }
        else:
            self._pending_sample = None

        return gnss

    def record_navigation(
        self,
        *,
        distance_to_goal_m: float,
        heading_deg: float,
    ) -> dict[str, Any] | None:
        """直前の有効なGNSS取得値と誘導時の機体方位を1行保存する。"""
        if self._writer is None or self._file is None:
            raise RuntimeError("GnssNavigationCsvLogger is not open")
        if self._pending_sample is None:
            return None

        sample = self._pending_sample
        row: dict[str, Any] = {
            "start_latitude_deg": self._blank_if_none(self.start_latitude_deg),
            "start_longitude_deg": self._blank_if_none(self.start_longitude_deg),
            "goal_latitude_deg": self.goal_latitude_deg,
            "goal_longitude_deg": self.goal_longitude_deg,
            "timestamp": sample["timestamp"],
            "elapsed_s": f"{float(sample['elapsed_s']):.3f}",
            "latitude_deg": sample["latitude_deg"],
            "longitude_deg": sample["longitude_deg"],
            "altitude_m": self._blank_if_none(sample.get("altitude_m")),
            "distance_to_goal_m": float(distance_to_goal_m),
            "heading_deg": float(heading_deg),
        }
        self._writer.writerow(row)
        self._file.flush()

        # 1回のGNSS取得結果を二重記録しない。
        self._pending_sample = None
        return row

    def discard_pending_sample(self) -> None:
        """ゴール到達など、方位制御をしない取得結果を破棄する。"""
        self._pending_sample = None

    @staticmethod
    def _blank_if_none(value: Any) -> Any:
        return "" if value is None else value
