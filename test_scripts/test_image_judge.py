#!/usr/bin/env python3
"""自撮り画像の最小選定ロジックを検証する。"""

from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

try:
    import cv2  # noqa: F401
except ModuleNotFoundError:
    sys.modules["cv2"] = types.ModuleType("cv2")

from image_judge import ImageJudge  # noqa: E402
from image_processor import ImageProcessor  # noqa: E402


def evaluation(
    path: str,
    *,
    capture_ok: bool = False,
    aruco_detected: bool = False,
    sharpness: float = 100.0,
    clipping_ratio: float = 0.0,
) -> dict:
    return {
        "path": Path(path),
        "is_valid": True,
        "error": None,
        "aruco_detected": aruco_detected,
        "capture_ok": capture_ok,
        "sharpness": sharpness,
        "is_blurry": sharpness < 50.0,
        "white_clipping_ratio": clipping_ratio / 2,
        "black_crush_ratio": clipping_ratio / 2,
        "clipping_ratio": clipping_ratio,
        "is_candidate": False,
    }


class ImageJudgeTest(unittest.TestCase):
    def setUp(self) -> None:
        self.judge = ImageJudge(Mock())

    def test_capture_ok_image_has_highest_priority(self) -> None:
        normal = evaluation(
            "normal.jpg",
            capture_ok=True,
            aruco_detected=True,
            clipping_ratio=0.2,
        )
        marker_only = evaluation(
            "marker_only.jpg",
            aruco_detected=True,
            clipping_ratio=0.0,
        )

        with patch.object(
            self.judge,
            "evaluate_image",
            side_effect=[normal, marker_only],
        ):
            result = self.judge.select_best_image(["normal.jpg", "marker_only.jpg"])

        self.assertEqual(result["selected_path"], Path("normal.jpg"))
        self.assertTrue(result["capture_ok_filter_applied"])

    def test_blurry_image_is_removed_before_exposure_comparison(self) -> None:
        blurry = evaluation("blurry.jpg", sharpness=10.0, clipping_ratio=0.0)
        sharp = evaluation("sharp.jpg", sharpness=100.0, clipping_ratio=0.2)

        with patch.object(
            self.judge,
            "evaluate_image",
            side_effect=[blurry, sharp],
        ):
            result = self.judge.select_best_image(["blurry.jpg", "sharp.jpg"])

        self.assertEqual(result["selected_path"], Path("sharp.jpg"))

    def test_lowest_clipping_is_selected_from_sharp_images(self) -> None:
        normal = evaluation("normal.jpg", sharpness=100.0, clipping_ratio=0.1)
        clipped = evaluation("clipped.jpg", sharpness=1000.0, clipping_ratio=0.5)

        with patch.object(
            self.judge,
            "evaluate_image",
            side_effect=[normal, clipped],
        ):
            result = self.judge.select_best_image(["normal.jpg", "clipped.jpg"])

        self.assertEqual(result["selected_path"], Path("normal.jpg"))

    def test_sharpest_image_is_selected_when_all_are_blurry(self) -> None:
        first = evaluation("first.jpg", sharpness=10.0, clipping_ratio=0.0)
        second = evaluation("second.jpg", sharpness=20.0, clipping_ratio=0.5)

        with patch.object(
            self.judge,
            "evaluate_image",
            side_effect=[first, second],
        ):
            result = self.judge.select_best_image(["first.jpg", "second.jpg"])

        self.assertEqual(result["selected_path"], Path("second.jpg"))

    def test_image_processor_is_the_entry_point(self) -> None:
        processor = ImageProcessor()
        expected = {"selected_path": Path("best.jpg")}

        with patch("image_judge.ImageJudge") as judge_type:
            judge_type.return_value.select_best_image.return_value = expected
            result = processor.select_best_selfie_image(["first.jpg"])

        judge_type.assert_called_once_with(processor)
        self.assertEqual(result, expected)


if __name__ == "__main__":
    unittest.main()
