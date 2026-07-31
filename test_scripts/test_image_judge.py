#!/usr/bin/env python3
"""ImageJudgeの選定ロジックを検証する単体テスト。"""

from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

# 開発PCにOpenCVがなくても、画像入出力を使わない選定ロジックはテストする。
try:
    import cv2  # noqa: F401
except ModuleNotFoundError:
    sys.modules["cv2"] = types.ModuleType("cv2")

from image_judge import ImageJudge  # noqa: E402


def evaluation(
    path: str,
    *,
    aruco_detected: bool,
    clipping_ratio: float,
    sharpness: float,
) -> dict:
    """evaluate_image()が返す正常画像のテスト用評価値を作る。"""
    return {
        "path": Path(path),
        "is_valid": True,
        "error": None,
        "aruco_detected": aruco_detected,
        "white_clipping_ratio": clipping_ratio / 2,
        "black_crush_ratio": clipping_ratio / 2,
        "clipping_ratio": clipping_ratio,
        "sharpness": sharpness,
        "is_candidate": False,
        "clipping_penalty": None,
        "sharpness_normalized": None,
        "sharpness_bonus": None,
        "total_score": None,
    }


class ImageJudgeTest(unittest.TestCase):
    def setUp(self) -> None:
        self.judge = ImageJudge(image_processor=Mock())

    def test_aruco_images_are_prioritized_when_any_marker_is_detected(self) -> None:
        marker = evaluation(
            "marker.jpg",
            aruco_detected=True,
            clipping_ratio=0.20,
            sharpness=10.0,
        )
        no_marker = evaluation(
            "no_marker.jpg",
            aruco_detected=False,
            clipping_ratio=0.0,
            sharpness=1000.0,
        )

        with patch.object(
            self.judge,
            "evaluate_image",
            side_effect=[marker, no_marker],
        ):
            result = self.judge.select_best_image(
                ["marker.jpg", "no_marker.jpg"]
            )

        self.assertTrue(result["aruco_filter_applied"])
        self.assertEqual(result["candidate_count"], 1)
        self.assertEqual(result["selected_path"], Path("marker.jpg"))
        self.assertTrue(marker["is_candidate"])
        self.assertFalse(no_marker["is_candidate"])

    def test_all_images_are_candidates_when_no_marker_is_detected(self) -> None:
        blurry = evaluation(
            "blurry.jpg",
            aruco_detected=False,
            clipping_ratio=0.0,
            sharpness=1.0,
        )
        sharp = evaluation(
            "sharp.jpg",
            aruco_detected=False,
            clipping_ratio=0.0,
            sharpness=1000.0,
        )

        with patch.object(
            self.judge,
            "evaluate_image",
            side_effect=[blurry, sharp],
        ):
            result = self.judge.select_best_image(["blurry.jpg", "sharp.jpg"])

        self.assertFalse(result["aruco_filter_applied"])
        self.assertEqual(result["candidate_count"], 2)
        self.assertEqual(result["selected_path"], Path("sharp.jpg"))

    def test_white_and_black_clipping_are_penalized(self) -> None:
        normal = evaluation(
            "normal.jpg",
            aruco_detected=False,
            clipping_ratio=0.0,
            sharpness=100.0,
        )
        clipped = evaluation(
            "clipped.jpg",
            aruco_detected=False,
            clipping_ratio=1.0,
            sharpness=1000.0,
        )

        with patch.object(
            self.judge,
            "evaluate_image",
            side_effect=[normal, clipped],
        ):
            result = self.judge.select_best_image(["normal.jpg", "clipped.jpg"])

        self.assertEqual(result["selected_path"], Path("normal.jpg"))
        self.assertGreater(
            clipped["clipping_penalty"],
            normal["clipping_penalty"],
        )
        self.assertLess(clipped["total_score"], normal["total_score"])


if __name__ == "__main__":
    unittest.main()
