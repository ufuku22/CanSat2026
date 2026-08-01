import math
from typing import Any

from config import RedBallConfig


def candidate_visible_size(candidate: dict[str, Any]) -> float | None:
    if candidate.get("radius_px") is not None:
        return float(candidate["radius_px"]) * 2.0
    if candidate.get("visible_diameter_px") is not None:
        return float(candidate["visible_diameter_px"])
    if candidate.get("score") is not None:
        return math.sqrt(max(0.0, float(candidate["score"])))
    return None


def _sized_candidates(red_result: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        candidate
        for candidate in red_result.get("red_ball_candidates", [])
        if candidate.get("center_offset_ratio") is not None
        and candidate_visible_size(candidate) is not None
    ]


def _is_duplicate_red_ball_candidate(
    candidate: dict[str, Any],
    kept_candidate: dict[str, Any],
) -> bool:
    """円候補とサイズ候補が同じ赤ボールを指すか判定する。"""
    if (
        candidate.get("x") is None
        or candidate.get("y") is None
        or kept_candidate.get("x") is None
        or kept_candidate.get("y") is None
    ):
        return False

    candidate_size = candidate_visible_size(candidate)
    kept_size = candidate_visible_size(kept_candidate)
    if candidate_size is None or kept_size is None:
        return False

    center_distance = math.hypot(
        float(candidate["x"]) - float(kept_candidate["x"]),
        float(candidate["y"]) - float(kept_candidate["y"]),
    )
    duplicate_distance = max(25.0, min(candidate_size, kept_size) * 0.425)
    return center_distance <= duplicate_distance


def merge_candidates(
    circle_candidates: list[dict[str, Any]],
    size_candidates: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """円候補があれば円候補だけを使い、なければサイズ候補を使う。"""
    if circle_candidates:
        return [{**item, "candidate_source": "circle"} for item in circle_candidates]

    merged = []
    for candidate in size_candidates:
        matching_candidate = next(
            (
                kept_candidate
                for kept_candidate in merged
                if _is_duplicate_red_ball_candidate(
                    candidate,
                    kept_candidate,
                )
            ),
            None,
        )
        if matching_candidate is not None:
            matching_candidate["color_bbox"] = (
                candidate.get("bbox") or {}
            ).copy()
            matching_candidate["color_visible_diameter_px"] = (
                candidate_visible_size(candidate)
            )
            continue
        candidate = candidate.copy()
        candidate["candidate_source"] = "size"
        merged.append(candidate)

    return merged


def select_nearest(red_result: dict[str, Any]):
    """見かけサイズから、近そうな赤ボール候補を選ぶ。"""
    return max(
        _sized_candidates(red_result),
        key=lambda candidate: (
            float(candidate_visible_size(candidate)),
            float(candidate.get("score", 0.0)),
            -abs(float(candidate["center_offset_ratio"])),
        ),
        default=None,
    )


def select_farthest(red_result: dict[str, Any]):
    """見切れや手前の球との重なりを除き、遠そうな候補を選ぶ。"""
    ball_candidates = _sized_candidates(red_result)
    if not ball_candidates:
        return None

    image_width = red_result.get("image_width")
    image_height = red_result.get("image_height")
    usable_candidates = []
    for candidate in ball_candidates:
        candidate_size = float(candidate_visible_size(candidate))
        bbox = (
            candidate.get("color_bbox")
            or candidate.get("bbox")
            or {}
        )
        clipping_reference_size = max(
            candidate_size,
            float(
                candidate.get(
                    "color_visible_diameter_px",
                    candidate_size,
                )
            ),
        )
        bbox_x = float(
            bbox.get(
                "x",
                float(candidate["x"]) - clipping_reference_size / 2.0,
            )
        )
        bbox_y = float(
            bbox.get(
                "y",
                float(candidate.get("y", 0.0))
                - clipping_reference_size / 2.0,
            )
        )
        bbox_width = float(bbox.get("width", clipping_reference_size))
        bbox_height = float(bbox.get("height", clipping_reference_size))
        excessively_clipped = False
        if image_width is not None:
            visible_width = max(
                0.0,
                min(bbox_x + bbox_width, float(image_width))
                - max(bbox_x, 0.0),
            )
            touches_horizontal_edge = (
                bbox_x <= 0.0
                or bbox_x + bbox_width >= float(image_width)
            )
            excessively_clipped = (
                touches_horizontal_edge
                and visible_width / clipping_reference_size < 0.75
            )
        if image_height is not None:
            visible_height = max(
                0.0,
                min(bbox_y + bbox_height, float(image_height))
                - max(bbox_y, 0.0),
            )
            touches_vertical_edge = (
                bbox_y <= 0.0
                or bbox_y + bbox_height >= float(image_height)
            )
            excessively_clipped = excessively_clipped or (
                touches_vertical_edge
                and visible_height / clipping_reference_size < 0.75
            )
        if excessively_clipped:
            continue

        hidden_by_larger_candidate = False
        if candidate.get("y") is not None:
            for other in ball_candidates:
                if other is candidate or other.get("y") is None:
                    continue
                other_size = float(candidate_visible_size(other))
                if other_size < candidate_size * 1.15:
                    continue
                center_distance = math.hypot(
                    float(candidate["x"]) - float(other["x"]),
                    float(candidate["y"]) - float(other["y"]),
                )
                if (
                    center_distance
                    <= other_size / 2.0 + candidate_size * 0.25
                ):
                    hidden_by_larger_candidate = True
                    break
        if not hidden_by_larger_candidate:
            usable_candidates.append(candidate)

    if usable_candidates:
        largest_size = max(
            float(candidate_visible_size(candidate))
            for candidate in usable_candidates
        )
        minimum_size = (
            largest_size
            * float(
                RedBallConfig.FARTHEST_MIN_SIZE_RATIO_TO_LARGEST
            )
        )
        usable_candidates = [
            candidate
            for candidate in usable_candidates
            if float(candidate_visible_size(candidate)) >= minimum_size
        ]

    if len(usable_candidates) < len(ball_candidates):
        print(
            "遠方候補選択: "
            f"usable={len(usable_candidates)}/"
            f"{len(ball_candidates)}"
        )
    if not usable_candidates:
        return None

    return min(
        usable_candidates,
        key=lambda candidate: (
            float(candidate_visible_size(candidate)),
            abs(float(candidate["center_offset_ratio"])),
        ),
    )


def classify_position(
    red_result: dict[str, Any],
    selected_ball: dict[str, Any],
) -> str:
    """選択したボールが検出候補列の左・中央・右のどこかを返す。"""
    candidates = sorted(
        (
            candidate
            for candidate in red_result.get("red_ball_candidates", [])
            if candidate.get("x") is not None
        ),
        key=lambda candidate: float(candidate["x"]),
    )
    if len(candidates) <= 1:
        return "center"

    selected_x = float(selected_ball["x"])
    selected_index = min(
        range(len(candidates)),
        key=lambda index: abs(
            float(candidates[index]["x"]) - selected_x
        ),
    )
    if selected_index == 0:
        return "left"
    if selected_index == len(candidates) - 1:
        return "right"
    return "center"


def _red_ball_lock_score(
    candidate: dict[str, Any],
    target_hint_x: float,
    target_hint_size_px: float,
    config: RedBallConfig,
) -> float:
    """前回位置と、前進で小さくならないサイズ変化から同じボールらしさを評価する。"""
    position_similarity = math.exp(
        -abs(float(candidate["x"]) - float(target_hint_x)) / max(
            float(config.CENTERING_TARGET_LOCK_POSITION_SCALE_PX),
            1.0,
        )
    )
    visible_size = candidate_visible_size(candidate)
    if (
        visible_size is None
        or visible_size <= 0.0
        or target_hint_size_px <= 0.0
    ):
        size_similarity = 0.0
    else:
        size_similarity = min(visible_size / target_hint_size_px, 1.0)

    position_weight = max(
        0.0, float(config.CENTERING_TARGET_LOCK_POSITION_WEIGHT)
    )
    size_weight = max(0.0, float(config.CENTERING_TARGET_LOCK_SIZE_WEIGHT))
    total_weight = max(
        position_weight + size_weight,
        1.0,
    )
    return (
        position_weight * position_similarity
        + size_weight * size_similarity
    ) / total_weight


def predict_x_after_rotation(
    ball: dict[str, Any],
    rotated_angle_deg: float,
    horizontal_fov_deg: float,
    image_width: float | None,
) -> float | None:
    """旋回後も同じ赤ボールを追うため、次フレームでの予想x座標を返す。"""
    if ball.get("x") is None or image_width is None:
        return None

    image_width = float(image_width)
    horizontal_fov_deg = float(horizontal_fov_deg)
    if image_width <= 0.0 or horizontal_fov_deg <= 0.0:
        return None

    predicted_x = (
        float(ball["x"])
        - (float(rotated_angle_deg) / horizontal_fov_deg) * image_width
    )
    return max(0.0, min(image_width - 1.0, predicted_x))


def select_near_hint(
    red_result: dict[str, Any],
    target_hint_x: float,
    config: RedBallConfig,
    *,
    target_hint_size_px: float | None = None,
):
    """全候補から前回位置と大きさに最も似た赤ボールを選ぶ。"""
    ball_candidates = [
        candidate
        for candidate in red_result.get("red_ball_candidates", [])
        if (
            candidate.get("x") is not None
            and candidate.get("center_offset_ratio") is not None
        )
    ]
    if not ball_candidates:
        return None

    if target_hint_size_px is None:
        return min(
            ball_candidates,
            key=lambda candidate: abs(
                float(candidate["x"]) - float(target_hint_x)
            ),
        )

    selected_ball = max(
        ball_candidates,
        key=lambda candidate: _red_ball_lock_score(
            candidate,
            target_hint_x,
            target_hint_size_px,
            config,
        ),
    )
    selected_ball = selected_ball.copy()
    selected_ball["target_lock_score"] = _red_ball_lock_score(
        selected_ball,
        target_hint_x,
        target_hint_size_px,
        config,
    )
    return selected_ball


def select_adjacent(
    red_result: dict[str, Any],
    horizontal_fov_deg: float,
    min_angle_deg: float | None,
):
    """中央の対象を除き、最も近そうに見える隣の赤ボールを選ぶ。"""
    adjacent_balls = []
    if min_angle_deg is not None:
        min_angle_deg = float(min_angle_deg)
    for ball in _sized_candidates(red_result):
        offset_ratio = ball.get("center_offset_ratio")
        visible_size = candidate_visible_size(ball)
        angle_deg = float(offset_ratio) * float(horizontal_fov_deg)
        if (
            min_angle_deg is not None
            and abs(angle_deg) <= min_angle_deg
        ):
            continue

        adjacent_balls.append(
            (
                float(visible_size),
                float(ball.get("score", 0.0)),
                abs(angle_deg),
                angle_deg,
                ball,
            )
        )

    if not adjacent_balls:
        return None, None

    _, _, _, angle_deg, ball = max(
        adjacent_balls,
        key=lambda item: item[:3],
    )
    return ball, angle_deg

