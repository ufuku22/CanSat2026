"""複数の自撮り画像を評価し、送信に適した1枚を選定する。"""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from image_processor import ImageProcessor


class ImageJudge:
    """ArUco、露出、鮮明度を使って自撮り画像を評価する。"""

    def __init__(
        self,
        *,
        image_processor: ImageProcessor | None = None,
        white_threshold: int = 245,
        black_threshold: int = 10,
        sharpness_max_width: int = 640,
        clipping_penalty_weight: float = 200.0,
        sharpness_bonus_weight: float = 100.0,
    ) -> None:
        if not 0 <= black_threshold < white_threshold <= 255:
            raise ValueError(
                "thresholds must satisfy 0 <= black_threshold "
                "< white_threshold <= 255"
            )
        if sharpness_max_width <= 0:
            raise ValueError("sharpness_max_width must be greater than zero")
        if clipping_penalty_weight < 0 or sharpness_bonus_weight < 0:
            raise ValueError("score weights must not be negative")

        self.image_processor = image_processor or ImageProcessor()
        self.white_threshold = white_threshold
        self.black_threshold = black_threshold
        self.sharpness_max_width = sharpness_max_width
        self.clipping_penalty_weight = clipping_penalty_weight
        self.sharpness_bonus_weight = sharpness_bonus_weight

    def evaluate_image(self, image_path: Path | str) -> dict[str, Any]:
        """画像1枚についてArUco、白飛び、黒つぶれ、鮮明度を計算する。"""
        path = Path(image_path)
        image = self.image_processor.load_image(path)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        white_clipping_ratio = float(np.mean(gray >= self.white_threshold))
        black_crush_ratio = float(np.mean(gray <= self.black_threshold))
        sharpness = self._calculate_sharpness(gray)

        aruco_result = self.image_processor.detect_single_aruco_marker_for_capture_check(
            image
        )

        return {
            "path": path,
            "is_valid": True,
            "error": None,
            "aruco_detected": bool(aruco_result["is_detected"]),
            "white_clipping_ratio": white_clipping_ratio,
            "black_crush_ratio": black_crush_ratio,
            "clipping_ratio": white_clipping_ratio + black_crush_ratio,
            "sharpness": sharpness,
            "is_candidate": False,
            "clipping_penalty": None,
            "sharpness_normalized": None,
            "sharpness_bonus": None,
            "total_score": None,
        }

    def select_best_image(
        self,
        image_paths: Iterable[Path | str],
    ) -> dict[str, Any]:
        """指定画像を評価し、基準を満たす候補の中から最高得点の1枚を返す。

        ArUcoが1枚以上から検出された場合は、検出された画像だけを候補にする。
        1枚も検出されなかった場合は、有効な全画像を候補にする。
        候補内では白飛び・黒つぶれ率を減点し、鮮明度を加点する。
        """
        paths = [Path(path) for path in image_paths]
        if not paths:
            raise ValueError("image_paths must contain at least one image")

        evaluations: list[dict[str, Any]] = []
        for path in paths:
            try:
                evaluations.append(self.evaluate_image(path))
            except (FileNotFoundError, ValueError) as exc:
                evaluations.append(self._invalid_evaluation(path, exc))

        valid_evaluations = [
            evaluation for evaluation in evaluations if evaluation["is_valid"]
        ]
        if not valid_evaluations:
            raise ValueError("no valid images could be evaluated")

        aruco_filter_applied = any(
            evaluation["aruco_detected"] for evaluation in valid_evaluations
        )
        candidates = [
            evaluation
            for evaluation in valid_evaluations
            if not aruco_filter_applied or evaluation["aruco_detected"]
        ]

        self._assign_scores(candidates)
        for evaluation in candidates:
            evaluation["is_candidate"] = True

        selected = max(
            candidates,
            key=lambda evaluation: (
                evaluation["total_score"],
                -evaluation["clipping_ratio"],
                evaluation["sharpness"],
            ),
        )

        return {
            "selected_path": selected["path"],
            "aruco_filter_applied": aruco_filter_applied,
            "candidate_count": len(candidates),
            "evaluations": evaluations,
        }

    def _calculate_sharpness(self, gray: np.ndarray) -> float:
        """縮小画像のLaplacian分散を鮮明度として返す。"""
        height, width = gray.shape[:2]
        if width > self.sharpness_max_width:
            scale = self.sharpness_max_width / width
            gray = cv2.resize(
                gray,
                (self.sharpness_max_width, max(1, round(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())

    def _assign_scores(self, candidates: list[dict[str, Any]]) -> None:
        """候補内で鮮明度を正規化し、減点と加点を割り当てる。"""
        log_sharpness = np.log1p(
            np.array(
                [evaluation["sharpness"] for evaluation in candidates],
                dtype=np.float64,
            )
        )

        if len(candidates) == 1:
            normalized_sharpness = np.array([1.0])
        else:
            minimum = float(log_sharpness.min())
            maximum = float(log_sharpness.max())
            if np.isclose(minimum, maximum):
                normalized_sharpness = np.full(len(candidates), 0.5)
            else:
                normalized_sharpness = (
                    (log_sharpness - minimum) / (maximum - minimum)
                )

        for evaluation, sharpness_normalized in zip(
            candidates,
            normalized_sharpness,
            strict=True,
        ):
            clipping_penalty = (
                evaluation["clipping_ratio"] * self.clipping_penalty_weight
            )
            sharpness_bonus = (
                float(sharpness_normalized) * self.sharpness_bonus_weight
            )
            evaluation["clipping_penalty"] = float(clipping_penalty)
            evaluation["sharpness_normalized"] = float(sharpness_normalized)
            evaluation["sharpness_bonus"] = float(sharpness_bonus)
            evaluation["total_score"] = float(sharpness_bonus - clipping_penalty)

    @staticmethod
    def _invalid_evaluation(
        path: Path,
        error: Exception,
    ) -> dict[str, Any]:
        """読み込めなかった画像の評価結果を返す。"""
        return {
            "path": path,
            "is_valid": False,
            "error": f"{type(error).__name__}: {error}",
            "aruco_detected": False,
            "white_clipping_ratio": None,
            "black_crush_ratio": None,
            "clipping_ratio": None,
            "sharpness": None,
            "is_candidate": False,
            "clipping_penalty": None,
            "sharpness_normalized": None,
            "sharpness_bonus": None,
            "total_score": None,
        }
