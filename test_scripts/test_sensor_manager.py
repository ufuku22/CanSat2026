#!/usr/bin/env python3
"""sensor_manager.pyの簡易テスト。

実機I2Cの代わりにFakeBusを使うので、開発PCでも確認できます。
"""

from pathlib import Path
import unittest
from unittest.mock import patch

import sensor_manager
from sensor_manager import (
    BME280,
    BME280_ADDR,
    BNO055,
    BNO055_ADDR,
    LC76G,
    LC76G_CMD_ADDR,
    LC76G_READ_ADDR,
    LC76G_SETUP_COMMANDS,
    SensorManager,
    TSD20,
    TSD20_ADDR,
    CameraV3,
    nmea_latlon,
)


class FakeBus:
    """I2Cバスの代わり。レジスタ値を辞書で持ちます。"""

    def __init__(self) -> None:
        self.bytes: dict[tuple[int, int], int] = {}
        self.blocks: dict[tuple[int, int], list[int]] = {}
        self.writes: list[tuple[int, int, int | tuple[int, ...]]] = []
        self.closed = False

    def read_byte_data(self, addr: int, reg: int) -> int:
        return self.bytes.get((addr, reg), 0)

    def write_byte_data(self, addr: int, reg: int, value: int) -> None:
        self.writes.append((addr, reg, value))
        self.bytes[(addr, reg)] = value

    def read_i2c_block_data(self, addr: int, reg: int, length: int) -> list[int]:
        return self.blocks.get((addr, reg), [0] * length)[:length]

    def write_i2c_block_data(self, addr: int, reg: int, data: list[int]) -> None:
        self.writes.append((addr, reg, tuple(data)))

    def close(self) -> None:
        self.closed = True


class FakeCamera:
    def __init__(self) -> None:
        self.kwargs = {}

    def capture(self, **kwargs: object) -> Path:
        self.kwargs = kwargs
        return Path("/tmp/front.jpg")


class SensorManagerTest(unittest.TestCase):
    def test_bno055_reads_fusion_values(self) -> None:
        bus = FakeBus()
        bus.bytes[(BNO055_ADDR, 0x00)] = 0xA0
        bus.bytes[(BNO055_ADDR, 0x35)] = 0xFF
        for reg, value in {
            0x1A: 1440, 0x1C: -160, 0x1E: 88,
            0x08: 123, 0x0A: -456, 0x0C: 981,
            0x14: 200, 0x16: 0, 0x18: -20,
        }.items():
            bus.blocks[(BNO055_ADDR, reg)] = i16(value)

        imu = BNO055(bus)
        imu.setup()
        data = imu.read()

        self.assertIn((BNO055_ADDR, 0x3D, 0x0C), bus.writes)
        self.assertEqual(data["heading_deg"], 90.0)
        self.assertEqual(data["roll_deg"], -10.0)
        self.assertEqual(data["pitch_deg"], 5.5)
        self.assertEqual(data["accel_mps2"], (1.23, -4.56, 9.81))
        self.assertEqual(data["gyro_dps"], (12.5, 0.0, -1.25))
        self.assertEqual(data["calibration"], 0xFF)

    def test_bno055_reads_heading_only(self) -> None:
        bus = FakeBus()
        bus.blocks[(BNO055_ADDR, 0x1A)] = i16(1440)

        self.assertEqual(BNO055(bus).heading(), 90.0)

    def test_bme280_returns_reasonable_environment_values(self) -> None:
        bus = FakeBus()
        bus.bytes[(BME280_ADDR, 0xD0)] = 0x60
        bus.blocks[(BME280_ADDR, 0x88)] = bme_cal1()
        bus.blocks[(BME280_ADDR, 0xE1)] = bme_cal2()
        bus.blocks[(BME280_ADDR, 0xF7)] = bme_raw(415148, 519888, 30000)

        sensor = BME280(bus)
        sensor.setup()
        env = sensor.read()

        self.assertTrue(-40.0 < env["temperature_c"] < 85.0)
        self.assertTrue(300.0 < env["pressure_hpa"] < 1100.0)
        self.assertTrue(0.0 <= env["humidity_percent"] <= 100.0)

    def test_lc76g_reads_and_parses_nmea(self) -> None:
        bus = FakeBus()
        nmea = "$GNGGA,123519,3540.1234,N,13945.6789,E,1,08,0.9,44.5,M,0.0,M,,*00\r\n"
        raw = list(nmea.encode("ascii"))
        bus.blocks[(LC76G_READ_ADDR, 0x00)] = len(raw).to_bytes(4, "little")
        gnss = LC76G(bus)

        bus.blocks[(LC76G_READ_ADDR, 0x00)] = list(len(raw).to_bytes(4, "little"))
        original_read = bus.read_i2c_block_data
        offset = 0

        def read(addr: int, reg: int, length: int) -> list[int]:
            nonlocal offset
            if addr == LC76G_READ_ADDR and length == 4:
                return list(len(raw).to_bytes(4, "little"))
            if addr == LC76G_READ_ADDR:
                chunk = raw[offset:offset + length]
                offset += length
                return chunk
            return original_read(addr, reg, length)

        bus.read_i2c_block_data = read  # type: ignore[method-assign]
        data = gnss.read()

        self.assertEqual(data["fix_quality"], 1)
        self.assertEqual(data["satellites"], 8)
        self.assertAlmostEqual(data["latitude_deg"] or 0, 35.6687233333)
        self.assertAlmostEqual(data["longitude_deg"] or 0, 139.761315)
        self.assertEqual(data["altitude_m"], 44.5)
        self.assertTrue(any(w[0] == LC76G_CMD_ADDR for w in bus.writes))

    def test_lc76g_setup_sends_nmea_configuration(self) -> None:
        gnss = LC76G(FakeBus())
        with patch.object(gnss, "write_nmea_command") as write_command, \
             patch.object(sensor_manager.time, "sleep"):
            gnss.setup()

        self.assertEqual(
            [call.args[0] for call in write_command.call_args_list],
            list(LC76G_SETUP_COMMANDS),
        )

    def test_tsd20_reads_distance(self) -> None:
        bus = FakeBus()
        bus.bytes[(TSD20_ADDR, 0x03)] = 0x4A
        bus.blocks[(TSD20_ADDR, 0x00)] = [0x04, 0xD2]  # 1234 mm

        sensor = TSD20(bus)
        sensor.setup()

        self.assertEqual(sensor.read_m(), 1.234)
        self.assertIn((TSD20_ADDR, 0x02, 0x01), bus.writes)

    def test_sensor_manager_read_all(self) -> None:
        bus = FakeBus()
        bus.bytes[(BME280_ADDR, 0xD0)] = 0x60
        bus.bytes[(BNO055_ADDR, 0x00)] = 0xA0
        bus.bytes[(TSD20_ADDR, 0x03)] = 0x4A
        bus.blocks[(BME280_ADDR, 0x88)] = bme_cal1()
        bus.blocks[(BME280_ADDR, 0xE1)] = bme_cal2()
        bus.blocks[(BME280_ADDR, 0xF7)] = bme_raw(415148, 519888, 30000)
        bus.blocks[(TSD20_ADDR, 0x00)] = [0x00, 0x64]

        manager = SensorManager(bus=bus, camera=FakeCamera())
        with patch.object(manager.gnss, "write_nmea_command"), \
             patch.object(sensor_manager.time, "sleep"):
            manager.setup()
        data = manager.read_all(with_camera=True)

        self.assertIn("environment", data)
        self.assertIn("imu", data)
        self.assertIn("gnss", data)
        self.assertEqual(data["distance_m"], 0.1)
        self.assertEqual(data["front_image"], Path("/tmp/front.jpg"))

    def test_camera_capture_accepts_resolution_and_hdr(self) -> None:
        save_dir = Path(__file__).parent / "tmp_camera_test"
        with patch.object(sensor_manager, "which", return_value="rpicam-still"), \
             patch.object(sensor_manager.subprocess, "run") as run:
            path = CameraV3(save_dir=save_dir).capture(width=1280, height=720, hdr=True, timeout_ms=500)

        command = run.call_args.args[0]
        self.assertEqual(path.parent, save_dir)
        self.assertIn("--width", command)
        self.assertIn("1280", command)
        self.assertIn("--height", command)
        self.assertIn("720", command)
        self.assertIn("--hdr", command)
        self.assertIn("500", command)

    def test_sensor_manager_passes_camera_options(self) -> None:
        camera = FakeCamera()
        manager = SensorManager(bus=FakeBus(), camera=camera)

        manager.capture_front_image(width=640, height=480, hdr=True, timeout_ms=100)

        self.assertEqual(camera.kwargs, {"width": 640, "height": 480, "hdr": True, "timeout_ms": 100})

    def test_nmea_latlon(self) -> None:
        self.assertAlmostEqual(nmea_latlon("3540.1234", "N") or 0, 35.6687233333)
        self.assertAlmostEqual(nmea_latlon("13945.6789", "E") or 0, 139.761315)


def i16(value: int) -> list[int]:
    value &= 0xFFFF
    return [value & 0xFF, value >> 8]


def put16(block: list[int], index: int, value: int) -> None:
    value &= 0xFFFF
    block[index:index + 2] = [value & 0xFF, value >> 8]


def bme_cal1() -> list[int]:
    block = [0] * 26
    for index, value in [
        (0, 27504), (2, 26435), (4, -1000), (6, 36477), (8, -10685),
        (10, 3024), (12, 2855), (14, 140), (16, -7), (18, 15500),
        (20, -14600), (22, 6000),
    ]:
        put16(block, index, value)
    block[25] = 75
    return block


def bme_cal2() -> list[int]:
    h4, h5 = 325, 50
    return [362 & 0xFF, 362 >> 8, 0, h4 >> 4, (h4 & 0x0F) | ((h5 & 0x0F) << 4), h5 >> 4, 30]


def bme_raw(pressure: int, temperature: int, humidity: int) -> list[int]:
    return [
        pressure >> 12 & 0xFF, pressure >> 4 & 0xFF, pressure << 4 & 0xFF,
        temperature >> 12 & 0xFF, temperature >> 4 & 0xFF, temperature << 4 & 0xFF,
        humidity >> 8 & 0xFF, humidity & 0xFF,
    ]


if __name__ == "__main__":
    unittest.main()
