"""複数の自撮り画像から送信する1枚を選ぶ。"""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
from typing import Any

import cv2
import numpy as np


class ImageJudge:
    """機体の写り、ブレ、白飛び・黒つぶれの順で画像を選ぶ。"""

    def __init__(
        self,
        image_processor: Any,
        *,
        white_threshold: int = 245,
        black_threshold: int = 10,
        sharpness_threshold: float = 50.0,
    ) -> None:
        self.image_processor = image_processor
        self.white_threshold = white_threshold
        self.black_threshold = black_threshold
        self.sharpness_threshold = sharpness_threshold

    def evaluate_image(self, image_path: Path | str) -> dict[str, Any]:
        """画像1枚の機体判定、ブレ、白飛び・黒つぶれを求める。"""
        path = Path(image_path)
        image = self.image_processor.load_image(path)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco = self.image_processor.detect_single_aruco_marker_for_capture_check(
            image
        )

        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        white_ratio = float(np.mean(gray >= self.white_threshold))
        black_ratio = float(np.mean(gray <= self.black_threshold))

        return {
            "path": path,
            "is_valid": True,
            "error": None,
            "aruco_detected": bool(aruco["is_detected"]),
            "capture_ok": bool(aruco["is_capture_ok"]),
            "marker_id": aruco["marker_id"],
            "capture_reason": aruco["reason"],
            "sharpness": sharpness,
            "is_blurry": sharpness < self.sharpness_threshold,
            "white_clipping_ratio": white_ratio,
            "black_crush_ratio": black_ratio,
            "clipping_ratio": white_ratio + black_ratio,
            "is_candidate": False,
        }

    def select_best_image(
        self,
        image_paths: Iterable[Path | str],
    ) -> dict[str, Any]:
        """機体が正常に写った候補から、送信に適した1枚を選ぶ。"""
        paths = [Path(path) for path in image_paths]
        if not paths:
            raise ValueError("image_paths must contain at least one image")

        evaluations: list[dict[str, Any]] = []
        for path in paths:
            try:
                evaluations.append(self.evaluate_image(path))
            except (FileNotFoundError, ValueError) as exc:
                evaluations.append(self._invalid_evaluation(path, exc))

        valid = [item for item in evaluations if item["is_valid"]]
        if not valid:
            raise ValueError("no valid images could be evaluated")

        capture_ok = [item for item in valid if item["capture_ok"]]
        aruco_detected = [item for item in valid if item["aruco_detected"]]
        candidates = capture_ok or aruco_detected or valid

        sharp_candidates = [item for item in candidates if not item["is_blurry"]]
        if sharp_candidates:
            candidates = sharp_candidates
            selected = min(
                candidates,
                key=lambda item: (item["clipping_ratio"], -item["sharpness"]),
            )
        else:
            selected = max(
                candidates,
                key=lambda item: (item["sharpness"], -item["clipping_ratio"]),
            )

        for item in candidates:
            item["is_candidate"] = True

        return {
            "selected_path": selected["path"],
            "candidate_count": len(candidates),
            "capture_ok_filter_applied": bool(capture_ok),
            "aruco_filter_applied": bool(capture_ok or aruco_detected),
            "evaluations": evaluations,
        }

    @staticmethod
    def _invalid_evaluation(path: Path, error: Exception) -> dict[str, Any]:
        return {
            "path": path,
            "is_valid": False,
            "error": f"{type(error).__name__}: {error}",
            "aruco_detected": False,
            "capture_ok": False,
            "marker_id": None,
            "capture_reason": None,
            "sharpness": None,
            "is_blurry": None,
            "white_clipping_ratio": None,
            "black_crush_ratio": None,
            "clipping_ratio": None,
            "is_candidate": False,
        }
