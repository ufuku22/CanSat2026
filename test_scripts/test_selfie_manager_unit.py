#!/usr/bin/env python3
"""SelfieManagerのハードウェアを使わない単体テスト。"""

from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, call, patch


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from selfie_manager import SELFIE_EV_VALUES, SelfieManager  # noqa: E402


class CaptureConnectedTest(unittest.TestCase):
    def test_sends_five_ev_steps(self) -> None:
        expected_commands = ["CAPTURE -2", "CAPTURE -1", "CAPTURE 0", "CAPTURE 1", "CAPTURE 2"]

        for ev, expected_command in zip(
            SELFIE_EV_VALUES,
            expected_commands,
            strict=True,
        ):
            with self.subTest(ev=ev), tempfile.TemporaryDirectory() as image_dir:
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
                    saved_path = selfie.capture_connected(ev)

                self.assertEqual(saved_path.read_bytes(), b"jpeg")
                self.assertIn(f"_ev{ev:+.1f}.jpg", saved_path.name)
                self.assertEqual(
                    send_line.call_args_list,
                    [
                        call(connection, expected_command),
                        call(connection, "OK"),
                        call(connection, "COMPLETE"),
                    ],
                )

    def test_keeps_single_auto_exposure_capture(self) -> None:
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

            self.assertNotIn("_ev", saved_path.name)
            self.assertEqual(send_line.call_args_list[0], call(connection, "CAPTURE"))

    def test_captures_exposure_series_with_one_connection_check(self) -> None:
        with tempfile.TemporaryDirectory() as image_dir:
            selfie = SelfieManager(image_dir=image_dir)
            connection = Mock()
            selfie.connection = connection

            with (
                patch.object(selfie, "ensure_connection") as ensure_connection,
                patch.object(
                    selfie,
                    "_receive_line",
                    side_effect=["SIZE 4", "READY"] * len(SELFIE_EV_VALUES),
                ),
                patch.object(selfie, "_receive_exact", return_value=b"jpeg"),
                patch.object(selfie, "_send_line") as send_line,
            ):
                saved_paths = selfie.capture_exposure_series()

            ensure_connection.assert_called_once_with()
            self.assertEqual(len(saved_paths), len(SELFIE_EV_VALUES))
            self.assertEqual(
                send_line.call_args_list[::3],
                [
                    call(connection, "CAPTURE -2"),
                    call(connection, "CAPTURE -1"),
                    call(connection, "CAPTURE 0"),
                    call(connection, "CAPTURE 1"),
                    call(connection, "CAPTURE 2"),
                ],
            )

    def test_rejects_unknown_ev(self) -> None:
        selfie = SelfieManager()

        with self.assertRaises(ValueError):
            selfie.capture_connected(1.5)


if __name__ == "__main__":
    unittest.main()
