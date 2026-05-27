#!/usr/bin/env python3
import json
import unittest

from communication_manager import CommunicationManager


class FakeRadio:
    def __init__(self) -> None:
        self.commands: list[str] = []

    def command(self, command: str, wait: float | None = None) -> str:
        self.commands.append(command)
        return ">> Ok\r\n>> radio_tx_ok\r\n"


class CommunicationManagerTest(unittest.TestCase):
    def test_send_text_transmits_json_packet_as_hex(self) -> None:
        radio = FakeRadio()
        manager = CommunicationManager(radio=radio)

        response = manager.send_text("hello")

        self.assertIn("radio_tx_ok", response)
        self.assertEqual(len(radio.commands), 1)
        self.assertTrue(radio.commands[0].startswith("p2p tx "))

        payload_hex = radio.commands[0].removeprefix("p2p tx ")
        packet = json.loads(bytes.fromhex(payload_hex).decode("utf-8"))
        self.assertEqual(packet["type"], "text")
        self.assertEqual(packet["seq"], 1)
        self.assertEqual(packet["data"], {"message": "hello"})

    def test_send_telemetry_omits_large_raw_gnss_and_sensor_vectors(self) -> None:
        radio = FakeRadio()
        manager = CommunicationManager(radio=radio)

        manager.send_telemetry({
            "gnss": {
                "latitude_deg": 35.6687,
                "longitude_deg": 139.7613,
                "altitude_m": 44.5,
                "satellites": 8,
                "fix_quality": 1,
                "raw": "$GNGGA,...",
            },
            "imu": {
                "heading_deg": 135.25,
                "roll_deg": -1.38,
                "pitch_deg": 4.56,
                "accel_mps2": (0.02, -0.13, 9.79),
                "gyro_dps": (0.0, 0.06, -1.25),
                "calibration": 255,
            },
            "distance_m": 1.234,
        })

        payload_hex = radio.commands[0].removeprefix("p2p tx ")
        data = json.loads(bytes.fromhex(payload_hex).decode("utf-8"))["data"]
        self.assertEqual(data["gnss"]["lat"], 35.6687)
        self.assertNotIn("raw", data["gnss"])
        self.assertEqual(data["imu"]["head"], 135.25)
        self.assertNotIn("accel_mps2", data["imu"])
        self.assertEqual(data["dist"], 1.234)


if __name__ == "__main__":
    unittest.main()
