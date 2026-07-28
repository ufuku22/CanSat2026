#!/usr/bin/env python3
"""SelfieManagerのハードウェアを使わない単体テスト。"""

from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, call, patch


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from selfie_manager import ExposureCondition, SelfieManager  # noqa: E402


class ExposureConditionTest(unittest.TestCase):
    def test_capture_command(self) -> None:
        exposure = ExposureCondition(aec_value=300, gain=4)

        self.assertEqual(exposure.capture_command(), "CAPTURE 300 4")

    def test_rejects_out_of_range_values(self) -> None:
        with self.assertRaises(ValueError):
            ExposureCondition(aec_value=1201, gain=4)
        with self.assertRaises(ValueError):
            ExposureCondition(aec_value=300, gain=31)


class CaptureConnectedTest(unittest.TestCase):
    def test_sends_exposure_condition_and_saves_it_in_filename(self) -> None:
        with tempfile.TemporaryDirectory() as image_dir:
            selfie = SelfieManager(image_dir=image_dir)
            connection = Mock()
            selfie.connection = connection
            exposure = ExposureCondition(aec_value=300, gain=4)

            with (
                patch.object(selfie, "ensure_connection"),
                patch.object(
                    selfie,
                    "_receive_line",
                    side_effect=["SIZE 4", "READY"],
                ),
                patch.object(selfie, "_receive_exact", return_value=b"jpeg"),
                patch.object(selfie, "_send_line") as send_line,
            ):
                saved_path = selfie.capture_connected(exposure)

            self.assertEqual(saved_path.read_bytes(), b"jpeg")
            self.assertIn("_aec0300_gain04.jpg", saved_path.name)
            self.assertEqual(
                send_line.call_args_list,
                [
                    call(connection, "CAPTURE 300 4"),
                    call(connection, "OK"),
                    call(connection, "COMPLETE"),
                ],
            )

    def test_keeps_legacy_auto_exposure_command(self) -> None:
        with tempfile.TemporaryDirectory() as image_dir:
            selfie = SelfieManager(image_dir=image_dir)
            connection = Mock()
            selfie.connection = connection

            with (
                patch.object(selfie, "ensure_connection"),
                patch.object(
                    selfie,
                    "_receive_line",
                    side_effect=["SIZE 4", "READY"],
                ),
                patch.object(selfie, "_receive_exact", return_value=b"jpeg"),
                patch.object(selfie, "_send_line") as send_line,
            ):
                saved_path = selfie.capture_connected()

            self.assertIn("_auto.jpg", saved_path.name)
            self.assertEqual(send_line.call_args_list[0], call(connection, "CAPTURE"))


if __name__ == "__main__":
    unittest.main()
