#!/usr/bin/env python3
"""CanSat2026 ローバー用センサ管理クラス。

他の制御コードからは SensorManager だけを使えば、各センサの値を
短いメソッドで取得できるようにしています。
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from shutil import which
import subprocess
import time
from typing import Any, Callable, Optional

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:
    try:
        from smbus import SMBus  # type: ignore
        i2c_msg = None  # type: ignore
    except ImportError:
        SMBus = None  # type: ignore
        i2c_msg = None  # type: ignore


I2C_BUS = 1
BME280_ADDR = 0x76
BNO055_ADDR = 0x28
BNO055_ALT_ADDR = 0x29
BNO055_CHIP_ID = 0xA0
BNO055_RETRIES = 10
BNO055_RETRY_DELAY_S = 0.1
LC76G_CMD_ADDR = 0x50
LC76G_READ_ADDR = 0x54
LC76G_WRITE_ADDR = 0x58
LC76G_MAX_READ = 1024
LC76G_RETRIES = 20
LC76G_RETRY_DELAY_S = 0.01
LC76G_READY_TIMEOUT_S = 2.0
LC76G_SETUP_COMMANDS = (
    "PAIR050,1000",
    "PAIR062,0,1",
    "PAIR062,4,1",
)
TSD20_ADDR = 0x52
CAMERA_FULL_HD_WIDTH = 1920
CAMERA_FULL_HD_HEIGHT = 1080


class BME280:
    """AE-BME280。温度・気圧・湿度を返します。"""

    def __init__(self, bus: Any, address: int = BME280_ADDR) -> None:
        self.bus = bus
        self.addr = address
        self.cal: dict[str, int] = {}
        self.t_fine = 0

    def setup(self) -> None:
        # センサの存在確認。0x60はBME280のCHIP IDです。
        if self.bus.read_byte_data(self.addr, 0xD0) != 0x60:
            raise RuntimeError(f"BME280 not found: 0x{self.addr:02X}")

        self._read_calibration()
        self.bus.write_byte_data(self.addr, 0xF2, 0x01)  # 湿度 x1
        self.bus.write_byte_data(self.addr, 0xF4, 0x27)  # 温度/気圧 x1、通常動作
        self.bus.write_byte_data(self.addr, 0xF5, 0xA0)  # 待機時間 1000ms

    def read(self) -> dict[str, float]:
        # 出力例:
        # {"temperature_c": 24.8, "pressure_hpa": 1008.6, "humidity_percent": 52.3}
        raw_p, raw_t, raw_h = self._read_raw_measurements()
        temp = self._compensate_temperature(raw_t)
        return {
            "temperature_c": temp,
            "pressure_hpa": self._compensate_pressure(raw_p) / 100.0,
            "humidity_percent": self._compensate_humidity(raw_h),
        }

    def _read_raw_measurements(self) -> tuple[int, int, int]:
        # 0xF7から、気圧3byte・温度3byte・湿度2byteをまとめて読みます。
        # ここで得られる値はまだ実単位ではなく、補正前のADC生データです。
        data = self.bus.read_i2c_block_data(self.addr, 0xF7, 8)
        raw_pressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        raw_temperature = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        raw_humidity = (data[6] << 8) | data[7]
        return raw_pressure, raw_temperature, raw_humidity

    def _read_calibration(self) -> None:
        # BME280の補正係数。データシートの計算式で使います。
        a = self.bus.read_i2c_block_data(self.addr, 0x88, 26)
        b = self.bus.read_i2c_block_data(self.addr, 0xE1, 7)
        self.cal = {
            "T1": u16(a, 0), "T2": s16(a, 2), "T3": s16(a, 4),
            "P1": u16(a, 6), "P2": s16(a, 8), "P3": s16(a, 10),
            "P4": s16(a, 12), "P5": s16(a, 14), "P6": s16(a, 16),
            "P7": s16(a, 18), "P8": s16(a, 20), "P9": s16(a, 22),
            "H1": a[25], "H2": s16(b, 0), "H3": b[2],
            "H4": signed((b[3] << 4) | (b[4] & 0x0F), 12),
            "H5": signed((b[5] << 4) | (b[4] >> 4), 12),
            "H6": signed(b[6], 8),
        }

    def _compensate_temperature(self, raw_temperature: int) -> float:
        # t_fineは温度の途中計算値ですが、気圧・湿度補正にも必要なので保存します。
        c = self.cal
        var1 = (((raw_temperature >> 3) - (c["T1"] << 1)) * c["T2"]) >> 11
        var2_base = (raw_temperature >> 4) - c["T1"]
        var2 = (((var2_base * var2_base) >> 12) * c["T3"]) >> 14
        self.t_fine = var1 + var2
        return ((self.t_fine * 5 + 128) >> 8) / 100.0

    def _compensate_pressure(self, raw_pressure: int) -> float:
        # データシートの気圧補正式です。式自体は長いので、段階ごとに変数名を残します。
        # 戻り値はPaです。read()側でhPaへ変換します。
        c = self.cal
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * c["P6"]
        var2 += (var1 * c["P5"]) << 17
        var2 += c["P4"] << 35

        var1 = ((var1 * var1 * c["P3"]) >> 8) + ((var1 * c["P2"]) << 12)
        var1 = (((1 << 47) + var1) * c["P1"]) >> 33
        if var1 == 0:
            return 0.0

        pressure = (((1048576 - raw_pressure) << 31) - var2) * 3125 // var1
        var1 = (c["P9"] * (pressure >> 13) * (pressure >> 13)) >> 25
        var2 = (c["P8"] * pressure) >> 19
        pressure = ((pressure + var1 + var2) >> 8) + (c["P7"] << 4)
        return pressure / 256.0

    def _compensate_humidity(self, raw_humidity: int) -> float:
        # データシートの湿度補正式です。最後に0〜100%の範囲へ丸めます。
        c = self.cal
        humidity = self.t_fine - 76800

        humidity_input = (raw_humidity << 14) - (c["H4"] << 20) - c["H5"] * humidity
        humidity_input = (humidity_input + 16384) >> 15

        sensitivity = (humidity * c["H6"]) >> 10
        sensitivity = (sensitivity * (((humidity * c["H3"]) >> 11) + 32768)) >> 10
        sensitivity = ((sensitivity + 2097152) * c["H2"] + 8192) >> 14

        humidity = humidity_input * sensitivity
        humidity -= (((humidity >> 15) * (humidity >> 15)) >> 7) * c["H1"] >> 4
        humidity = max(0, min(humidity, 419430400))
        return (humidity >> 12) / 1024.0


class BNO055:
    """AE-BNO055-BO。NDOF fusionモードで姿勢を返します。"""

    def __init__(self, bus: Any, address: int = BNO055_ADDR) -> None:
        self.bus = bus
        self.addr = address

    def setup(self) -> None:
        # 起動直後やI2Cが不安定な瞬間はRemote I/Oになることがあるため、短くリトライします。
        self.addr = self._detect_address()

        self._write(0x3D, 0x00)  # CONFIG_MODE
        self._write(0x07, 0x00)  # PAGE 0
        self._write(0x3E, 0x00)  # NORMAL POWER
        self._write(0x3F, 0x00)
        self._write(0x3D, 0x0C)  # NDOF_MODE = fusionモード
        time.sleep(0.1)

    def read(self) -> dict[str, Any]:
        # 出力例:
        # {
        #   "heading_deg": 135.25, "roll_deg": -1.38, "pitch_deg": 4.56,
        #   "accel_mps2": (0.02, -0.13, 9.79),
        #   "gyro_dps": (0.0, 0.06, -0.12),
        #   "calibration": 255
        # }
        # BNO055の単位: Euler角 16LSB/deg、加速度 100LSB/(m/s^2)、ジャイロ 16LSB/dps。
        heading, roll, pitch = self._vec3(0x1A, 16.0)
        accel = self._vec3(0x08, 100.0)
        gyro = self._vec3(0x14, 16.0)
        return {
            "heading_deg": heading,
            "roll_deg": roll,
            "pitch_deg": pitch,
            "accel_mps2": accel,
            "gyro_dps": gyro,
            "calibration": self._read_byte(0x35),
        }

    def read_linear_acceleration(self) -> tuple[float, float, float]:
        data = self._read_block(0x28, 6)
        return self._vec3_from_block(data, 0, 100.0)

    def heading(self) -> float:
        # 方位だけ必要な制御ループ用。3軸全部を読むよりI2C通信量を減らせます。
        return self._i16(0x1A) / 16.0

    def _write(self, reg: int, value: int) -> None:
        self._retry_i2c(lambda: self.bus.write_byte_data(self.addr, reg, value))
        time.sleep(0.02)

    def _vec3(self, reg: int, scale: float) -> tuple[float, float, float]:
        return tuple(self._i16(reg + i) / scale for i in (0, 2, 4))

    @staticmethod
    def _vec3_from_block(
        data: list[int],
        offset: int,
        scale: float,
    ) -> tuple[float, float, float]:
        return tuple(
            signed(data[offset + i] | (data[offset + i + 1] << 8), 16) / scale
            for i in (0, 2, 4)
        )

    def _i16(self, reg: int) -> int:
        d = self._read_block(reg, 2)
        return signed(d[0] | (d[1] << 8), 16)

    def _detect_address(self) -> int:
        addresses = [self.addr]
        if self.addr == BNO055_ADDR:
            addresses.append(BNO055_ALT_ADDR)

        errors: list[str] = []
        for address in addresses:
            try:
                chip_id = self._read_byte(0x00, address=address)
            except OSError as exc:
                errors.append(f"0x{address:02X}: {exc}")
                continue
            if chip_id == BNO055_CHIP_ID:
                return address
            errors.append(f"0x{address:02X}: chip id 0x{chip_id:02X}")

            # 起動直後はIDが読めないことがあるため、少し待って再確認します。
            time.sleep(0.7)
            try:
                chip_id = self._read_byte(0x00, address=address)
            except OSError as exc:
                errors.append(f"0x{address:02X} after wait: {exc}")
                continue
            if chip_id == BNO055_CHIP_ID:
                return address
            errors.append(f"0x{address:02X} after wait: chip id 0x{chip_id:02X}")

        detail = "; ".join(errors) if errors else "no response"
        raise RuntimeError(
            f"BNO055 not found: checked {', '.join(f'0x{x:02X}' for x in addresses)} ({detail})"
        )

    def _read_byte(self, reg: int, *, address: Optional[int] = None) -> int:
        target_addr = self.addr if address is None else address
        return self._retry_i2c(lambda: self.bus.read_byte_data(target_addr, reg))

    def _read_block(self, reg: int, length: int) -> list[int]:
        return self._retry_i2c(lambda: self.bus.read_i2c_block_data(self.addr, reg, length))

    @staticmethod
    def _retry_i2c(func: Any) -> Any:
        last_error: Optional[OSError] = None
        for attempt in range(BNO055_RETRIES):
            try:
                return func()
            except OSError as exc:
                last_error = exc
                if attempt < BNO055_RETRIES - 1:
                    time.sleep(BNO055_RETRY_DELAY_S)
        if last_error is not None:
            raise last_error


class LC76G:
    """LC76G GNSS。I2CからNMEAを読み、GGA/RMCを簡易パースします。"""

    def __init__(self, bus: Any) -> None:
        self.bus = bus
        self.last = empty_gnss()
        self._pending_read_length: Optional[int] = None
        self._pending_write_data: Optional[list[int]] = None

    def setup(self) -> None:
        for command in LC76G_SETUP_COMMANDS:
            self.write_nmea_command(command)
            time.sleep(0.2)
        self.last = empty_gnss()
        self.discard_pending_nmea()

    def read(self, max_length: Optional[int] = None) -> dict[str, Any]:
        # まだ測位できていない項目はNoneにします。
        # max_length=Noneの通常読みは、古い/途中のNMEAだけを拾う問題を避けるため
        # read_latest_nmea()で複数チャンクをまとめて読みます。
        # ただし、連続テレメトリ送信のように長時間・低頻度で読み続ける用途では、
        # LC76GのNMEA出力キューが空のままになることがあるため、
        # 呼び出し側でread(max_length=1024)のような指定長読みと空読み時のsetup()復旧を使います。
        try:
            if max_length is None:
                raw = self.read_latest_nmea()
            else:
                raw = self.read_nmea(max_length=max_length)
        except RuntimeError as exc:
            gnss = empty_gnss()
            gnss["connected"] = True
            gnss["error"] = str(exc)
            self.last = gnss
            return gnss
        if not raw:
            gnss = empty_gnss()
            gnss["connected"] = True
            self.last = gnss
            return gnss

        gnss = self.parse_nmea(raw)
        self.last = gnss
        return gnss

    def parse_nmea(self, raw: str) -> dict[str, Any]:
        gnss = empty_gnss()
        gnss["connected"] = True
        gnss["raw"] = raw
        has_position = False
        for line in raw.splitlines():
            parts = line.split(",")
            kind = parts[0][-3:] if parts and parts[0].startswith("$") else ""
            if kind == "GGA" and len(parts) > 9:
                latitude = nmea_latlon(parts[2], parts[3])
                longitude = nmea_latlon(parts[4], parts[5])
                fix_quality = int_or_none(parts[6])
                gnss["fix_quality"] = fix_quality
                gnss["satellites"] = int_or_none(parts[7])
                if fix_quality is not None and fix_quality > 0 and latitude is not None and longitude is not None:
                    gnss["latitude_deg"] = latitude
                    gnss["longitude_deg"] = longitude
                    gnss["altitude_m"] = float_or_none(parts[9])
                    has_position = True
            elif kind == "RMC" and len(parts) > 7 and parts[2] == "A":
                latitude = nmea_latlon(parts[3], parts[4])
                longitude = nmea_latlon(parts[5], parts[6])
                gnss["ground_speed_mps"] = (
                    float(parts[7]) * 1852.0 / 3600.0
                )
                if latitude is not None and longitude is not None:
                    gnss["latitude_deg"] = latitude
                    gnss["longitude_deg"] = longitude
                    has_position = True

        gnss["has_fix"] = has_position
        if not gnss["has_fix"]:
            gnss["latitude_deg"] = None
            gnss["longitude_deg"] = None
            gnss["altitude_m"] = None
        return gnss

    def discard_pending_nmea(self, max_reads: int = 3) -> None:
        # 起動直後にI2C側へ残っている古いNMEAを捨てます。
        for _ in range(max_reads):
            try:
                if self.available_nmea_length() <= 0:
                    break
                self.read_nmea()
            except RuntimeError:
                break

    def read_latest_nmea(self, max_reads: int = 4) -> str:
        # I2Cバッファに残った古いNMEAをまとめて読み、最後に含まれる測位文を使えるようにします。
        # これはナビゲーション中のstale/partial NMEA対策です。長時間のテレメトリ用途では
        # 読みすぎを避けるため、read(max_length=...)で1回分ずつ読む運用も残します。
        chunks = []
        for _ in range(max_reads):
            if self.available_nmea_length() <= 0:
                break
            raw = self.read_nmea()
            if raw:
                chunks.append(raw)
        return "".join(chunks)

    def read_nmea(self, max_length: Optional[int] = None) -> str:
        # QuectelのI2C仕様では、まず送信バッファ長を読み、次にその長さだけNMEAを読みます。
        length = self.available_nmea_length()
        if length <= 0:
            return ""
        limit = LC76G_MAX_READ if max_length is None else max(1, max_length)
        length = min(length, limit)  # 1回の制御周期で読みすぎないための上限です。
        data = self._read_response(0xAA512000, length, length)
        return bytes(data).decode("ascii", errors="ignore").replace("\x00", "")

    def available_nmea_length(self) -> int:
        return self._read_uint32(0xAA510008)

    def write_free_length(self) -> int:
        return self._read_uint32(0xAA510004)

    def probe_i2c_addresses(self) -> dict[str, dict[str, Any]]:
        """LC76Gの3つのI2CアドレスがACKするか診断します。0x54は1byte消費します。"""
        return {
            "cmd_0x50": self._probe_address(LC76G_CMD_ADDR, read=False),
            "read_0x54": self._probe_address(LC76G_READ_ADDR, read=True),
            "write_0x58": self._probe_address(LC76G_WRITE_ADDR, read=False),
        }

    def write_nmea_command(self, command: str) -> None:
        sentence = self._format_nmea_command(command)
        data = list(sentence.encode("ascii"))
        free_length = self.write_free_length()
        if len(data) > free_length:
            raise RuntimeError(
                f"LC76G I2C connected, but write buffer is too small: {free_length} bytes"
            )
        self._write_request(0xAA531000, data)
        time.sleep(0.05)

    def _format_nmea_command(self, command: str) -> str:
        body = command.strip()
        if body.startswith("$"):
            body = body[1:]
        if "*" not in body:
            checksum = 0
            for char in body:
                checksum ^= ord(char)
            body = f"{body}*{checksum:02X}"
        return f"${body}\r\n"

    def _read_uint32(self, command: int) -> int:
        return int.from_bytes(bytes(self._read_response(command, 4, 4)), "little")

    def _send_request_command(self, command: int, command_arg: int) -> None:
        self._finish_pending_transfer()
        self._wait_for_command_ready()
        self._raw_i2c(LC76G_CMD_ADDR, data=self._words(command, command_arg))

    def _read_response(self, command: int, command_arg: int, read_length: int) -> list[int]:
        self._send_request_command(command, command_arg)
        self._pending_read_length = read_length
        data = self._raw_i2c(LC76G_READ_ADDR, length=read_length)
        self._pending_read_length = None
        return data

    def _write_request(self, command: int, data: list[int]) -> None:
        self._send_request_command(command, len(data))
        self._pending_write_data = data
        self._raw_i2c(LC76G_WRITE_ADDR, data=data)
        self._pending_write_data = None

    def _finish_pending_transfer(self) -> None:
        if self._pending_read_length is not None:
            self._raw_i2c(LC76G_READ_ADDR, length=self._pending_read_length)
            self._pending_read_length = None
        if self._pending_write_data is not None:
            self._raw_i2c(LC76G_WRITE_ADDR, data=self._pending_write_data)
            self._pending_write_data = None

    def _wait_for_command_ready(self) -> None:
        deadline = time.monotonic() + LC76G_READY_TIMEOUT_S
        last_error = ""
        while time.monotonic() < deadline:
            if self._address_ack(LC76G_CMD_ADDR, read=False):
                return
            if self._address_ack(LC76G_READ_ADDR, read=True):
                last_error = "0x54 still had pending read data"
            else:
                last_error = "0x50 did not ACK"
            time.sleep(LC76G_RETRY_DELAY_S)
        raise RuntimeError(f"LC76G I2C command address 0x50 is not ready: {last_error}")

    @staticmethod
    def _words(word1: int, word2: int) -> list[int]:
        return list(word1.to_bytes(4, "little") + word2.to_bytes(4, "little"))

    def _has_i2c_rdwr(self) -> bool:
        return i2c_msg is not None and hasattr(self.bus, "i2c_rdwr")

    def _retry_i2c(self, address: int, action: str, operation: Callable[[], Any]) -> Any:
        last_error: Optional[OSError] = None
        for _ in range(LC76G_RETRIES):
            time.sleep(LC76G_RETRY_DELAY_S)
            try:
                return operation()
            except OSError as exc:
                last_error = exc
        if last_error is not None:
            raise RuntimeError(
                f"LC76G I2C {action} failed at 0x{address:02X}: {last_error}"
            ) from last_error

    def _raw_i2c(
        self,
        address: int,
        *,
        data: Optional[list[int]] = None,
        length: int = 0,
    ) -> Any:
        action = "write" if data is not None else "read"

        def operation() -> Any:
            if self._has_i2c_rdwr():
                if data is not None:
                    return self.bus.i2c_rdwr(i2c_msg.write(address, data))
                msg = i2c_msg.read(address, length)
                self.bus.i2c_rdwr(msg)
                return list(msg)
            if data is not None:
                return self.bus.write_i2c_block_data(address, data[0], data[1:])
            return self.bus.read_i2c_block_data(address, 0x00, length)

        return self._retry_i2c(address, action, operation)

    def _probe_address(self, address: int, *, read: bool) -> dict[str, Any]:
        try:
            if not self._address_ack(address, read=read):
                return {"open": False, "error": "no ACK"}
        except RuntimeError:
            if not read:
                return {"open": None, "error": "write_quick is not available"}
        except OSError as exc:
            return {"open": False, "error": str(exc)}
        return {"open": True, "error": None}

    def _address_ack(self, address: int, *, read: bool) -> bool:
        try:
            if read:
                if self._has_i2c_rdwr():
                    msg = i2c_msg.read(address, 1)
                    self.bus.i2c_rdwr(msg)
                else:
                    self.bus.read_byte(address)
            elif hasattr(self.bus, "write_quick"):
                self.bus.write_quick(address)
            elif self._has_i2c_rdwr():
                self.bus.i2c_rdwr(i2c_msg.write(address, []))
            else:
                raise RuntimeError("write_quick is not available")
        except OSError:
            return False
        return True


class TSD20:
    """TSD20 LiDAR。距離をm単位で返します。"""

    def __init__(self, bus: Any, address: int = TSD20_ADDR) -> None:
        self.bus = bus
        self.addr = address

    def setup(self) -> None:
        # 0x03は識別用レジスタ。標準値0x4Aで通信確認できます。
        if self.bus.read_byte_data(self.addr, 0x03) != 0x4A:
            raise RuntimeError(f"TSD20 not found: 0x{self.addr:02X}")
        self.bus.write_byte_data(self.addr, 0x02, 0x01)  # レーザーON

    def read_m(self) -> Optional[float]:
        # 出力例: 1.234
        # 測距不能値50000mmを受け取った場合はNoneを返します。
        high, low = self.bus.read_i2c_block_data(self.addr, 0x00, 2)
        mm = (high << 8) | low
        return None if mm == 50000 else mm / 1000.0


class CameraV3:
    """Raspberry Pi Camera Module V3。静止画を保存してパスを返します。"""

    def __init__(self, save_dir: Optional[Path] = None) -> None:
        self.save_dir = save_dir or (Path.home() / "cansat_camera_images")

    def capture(
        self,
        width: int = CAMERA_FULL_HD_WIDTH,
        height: int = CAMERA_FULL_HD_HEIGHT,
        hdr: bool = False,
        timeout_ms: int = 2000,
    ) -> Path:
        self.save_dir.mkdir(parents=True, exist_ok=True)
        path = self.save_dir / f"front_{datetime.now():%Y%m%d_%H%M%S}.jpg"
        cmd = which("rpicam-still") or which("libcamera-still")
        if cmd is None:
            raise RuntimeError("rpicam-still or libcamera-still was not found.")

        command = [
            cmd,
            "-o", str(path),
            "--width", str(width),
            "--height", str(height),
            "--timeout", str(timeout_ms),
            "--nopreview",
            "--rotation", "0",
        ]
        if hdr:
            command.append("--hdr")

        try:
            subprocess.run(command, check=True, text=True, capture_output=True)
        except subprocess.CalledProcessError as exc:
            details = (exc.stderr or exc.stdout or "").strip()
            message = f"Camera capture failed with exit code {exc.returncode}: {' '.join(command)}"
            if details:
                message = f"{message}\n{details}"
            raise RuntimeError(message) from exc
        return path

    def capture_frame(
        self,
        width: int = CAMERA_FULL_HD_WIDTH,
        height: int = CAMERA_FULL_HD_HEIGHT,
        hdr: bool = False,
        timeout_ms: int = 2000,
    ):
        import cv2

        path = self.capture(width=width, height=height, hdr=hdr, timeout_ms=timeout_ms)
        frame = cv2.imread(str(path))
        if frame is None:
            raise RuntimeError(f"Captured image could not be read: {path}")
        return frame


class SensorManager:
    """全センサをまとめて扱うクラス。"""

    def __init__(
        self,
        bus: Optional[Any] = None,
        camera: Optional[CameraV3] = None,
        camera_save_dir: Optional[Path] = None,
    ) -> None:
        if bus is None and SMBus is None:
            raise RuntimeError("smbus2 or smbus is required on Raspberry Pi.")
        self.bus = bus or SMBus(I2C_BUS)
        self.owns_bus = bus is None
        self.environment = BME280(self.bus)
        self.imu = BNO055(self.bus)
        self.gnss = LC76G(self.bus)
        self.distance = TSD20(self.bus)
        self.camera = camera or CameraV3(save_dir=camera_save_dir)

    def setup(self) -> None:
        self.environment.setup()
        self.imu.setup()
        self.gnss.setup()
        self.distance.setup()

    def close(self) -> None:
        if hasattr(self.camera, "close"):
            self.camera.close()
        if self.owns_bus and hasattr(self.bus, "close"):
            self.bus.close()

    def get_environment(self) -> dict[str, float]:
        # 出力例: {"temperature_c": 24.8, "pressure_hpa": 1008.6, "humidity_percent": 52.3}
        return self.environment.read()

    def get_imu(self) -> dict[str, Any]:
        # 出力例:
        # {"heading_deg": 135.25, "roll_deg": -1.38, "pitch_deg": 4.56,
        #  "accel_mps2": (0.02, -0.13, 9.79), "gyro_dps": (0.0, 0.06, -0.12), "calibration": 255}
        return self.imu.read()

    def get_linear_acceleration(self) -> tuple[float, float, float]:
        return self.imu.read_linear_acceleration()

    def get_heading_deg(self) -> float:
        # 出力例: 135.25
        return self.imu.heading()

    def get_gnss(self) -> dict[str, Any]:
        # 出力例:
        # {"latitude_deg": 35.6687, "longitude_deg": 139.7613,
        #  "altitude_m": 44.5, "ground_speed_mps": 1.2,
        #  "satellites": 8, "fix_quality": 1, "raw": "$GNGGA,..."}
        return self.gnss.read()

    def get_gnss_i2c_status(self) -> dict[str, dict[str, Any]]:
        return self.gnss.probe_i2c_addresses()

    def get_distance_m(self) -> Optional[float]:
        # 出力例: 1.234
        return self.distance.read_m()

    def capture_front_image(
        self,
        width: int = CAMERA_FULL_HD_WIDTH,
        height: int = CAMERA_FULL_HD_HEIGHT,
        hdr: bool = False,
        timeout_ms: int = 2000,
    ) -> Path:
        # 出力例: /home/pi/cansat_camera_images/front_20260525_134210.jpg
        return self.camera.capture(width=width, height=height, hdr=hdr, timeout_ms=timeout_ms)

    def capture_front_frame(
        self,
        width: int = CAMERA_FULL_HD_WIDTH,
        height: int = CAMERA_FULL_HD_HEIGHT,
        hdr: bool = False,
        timeout_ms: int = 2000,
    ):
        # OpenCVで扱いやすいBGR画像を返します。
        return self.camera.capture_frame(width=width, height=height, hdr=hdr, timeout_ms=timeout_ms)

    def read_all(self, with_camera: bool = False) -> dict[str, Any]:
        # カメラ撮影は時間がかかるため、必要なときだけ含めます。
        data = {
            "environment": self.get_environment(),
            "imu": self.get_imu(),
            "gnss": self.get_gnss(),
            "distance_m": self.get_distance_m(),
        }
        if with_camera:
            data["front_image"] = self.capture_front_image()
        return data

    def __enter__(self) -> "SensorManager":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def u16(data: list[int], i: int) -> int:
    return data[i] | (data[i + 1] << 8)


def s16(data: list[int], i: int) -> int:
    return signed(u16(data, i), 16)


def signed(value: int, bits: int) -> int:
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def empty_gnss() -> dict[str, Any]:
    return {
        "latitude_deg": None,
        "longitude_deg": None,
        "altitude_m": None,
        "satellites": None,
        "fix_quality": None,
        "connected": False,
        "has_fix": False,
        "raw": "",
    }


def has_gnss_fix(gnss: dict[str, Any]) -> bool:
    fix_quality = gnss.get("fix_quality")
    if fix_quality is not None:
        return int(fix_quality) > 0
    return gnss.get("latitude_deg") is not None and gnss.get("longitude_deg") is not None


def int_or_none(value: str) -> Optional[int]:
    return int(value) if value else None


def float_or_none(value: str) -> Optional[float]:
    return float(value) if value else None


def nmea_latlon(value: str, direction: str) -> Optional[float]:
    if not value:
        return None
    dot = value.find(".")
    deg_len = dot - 2
    degrees = int(value[:deg_len])
    minutes = float(value[deg_len:])
    result = degrees + minutes / 60.0
    return -result if direction in ("S", "W") else result


def main() -> None:
    with SensorManager() as sensors:
        sensors.setup()
        while True:
            print(sensors.read_all())
            time.sleep(1.0)


if __name__ == "__main__":
    main()
