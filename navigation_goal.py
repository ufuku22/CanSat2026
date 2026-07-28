import math
import time
from typing import Any

from config import (
    RedBallConfig,
    RedConeConfig,
)
from image_processor import ImageProcessor
from navigation_controller import NavigationController
from sensor_manager import SensorManager


def _without_color_mask(color_result: dict[str, Any]) -> dict[str, Any]:
    """履歴用の色検出結果から大きな画像マスクを除外する。"""
    summary = color_result.copy()
    summary.pop("color_mask", None)
    return summary


def _find_red_cone_in_view(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    processor: ImageProcessor,
    red_cone_config: RedConeConfig,
):
    """カメラ画像内に赤コーンが入るまで、基礎旋回を使って探索する。"""
    scan_history = []
    for scan_index in range(red_cone_config.MAX_SCAN_STEPS):
        print(
            "赤コーン探索: "
            f"scan {scan_index + 1}/{red_cone_config.MAX_SCAN_STEPS} 撮影します"
        )
        frame = sensor_manager.capture_front_frame()
        red_result = _without_color_mask(
            processor.detect_color(
                frame,
                hsv_ranges=processor.RED_HSV_RANGES,
                color_threshold=red_cone_config.RED_THRESHOLD,
                column_threshold=red_cone_config.RED_COLUMN_THRESHOLD,
                column_average_width=red_cone_config.RED_COLUMN_AVERAGE_WIDTH,
            )
        )
        scan_history.append({
            "scan_index": scan_index,
            "red_result": red_result,
        })
        print(
            "赤コーン探索: "
            f"total={red_result['total_color_ratio'] * 100:.2f}% "
            f"column={red_result['color_peak_column_x']} "
            f"detected={red_result['is_color_detected']}"
        )

        if red_result["is_color_detected"]:
            print("赤コーン探索: 赤コーンを検出しました")
            return red_result, scan_history

        if scan_index < red_cone_config.MAX_SCAN_STEPS - 1:
            print(
                "赤コーン探索: "
                f"赤コーンなし。{red_cone_config.SCAN_ANGLE_DEG:.1f}度旋回して"
                "再探索します"
            )
            navigation_controller.rotate_by_angle(
                driver,
                sensor_manager,
                red_cone_config.SCAN_ANGLE_DEG,
                speed=red_cone_config.ROTATE_SPEED,
                tolerance_deg=red_cone_config.ROTATE_TOLERANCE_DEG,
                timeout_s=red_cone_config.ROTATE_TIMEOUT_S,
            )

    return None, scan_history


def _candidate_visible_size(candidate: dict[str, Any]) -> float | None:
    if candidate.get("radius_px") is not None:
        return float(candidate["radius_px"]) * 2.0
    if candidate.get("visible_diameter_px") is not None:
        return float(candidate["visible_diameter_px"])
    if candidate.get("score") is not None:
        return math.sqrt(max(0.0, float(candidate["score"])))
    return None


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

    candidate_size = _candidate_visible_size(candidate)
    kept_size = _candidate_visible_size(kept_candidate)
    if candidate_size is None or kept_size is None:
        return False

    center_distance = math.hypot(
        float(candidate["x"]) - float(kept_candidate["x"]),
        float(candidate["y"]) - float(kept_candidate["y"]),
    )
    duplicate_distance = max(25.0, min(candidate_size, kept_size) * 0.425)
    return center_distance <= duplicate_distance


def _merge_red_ball_candidates(
    circle_candidates: list[dict[str, Any]],
    size_candidates: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """円候補を優先し、円で拾えない端切れ候補をサイズ候補から補う。"""
    merged = []
    for candidate in circle_candidates:
        candidate = candidate.copy()
        candidate["candidate_source"] = "circle"
        merged.append(candidate)

    for candidate in size_candidates:
        if any(
            _is_duplicate_red_ball_candidate(candidate, kept_candidate)
            for kept_candidate in merged
        ):
            continue
        candidate = candidate.copy()
        candidate["candidate_source"] = "size"
        merged.append(candidate)

    return merged


def _select_nearest_red_ball(red_result: dict[str, Any]):
    """見かけサイズから、近そうな赤ボール候補を選ぶ。"""
    ball_candidates = [
        candidate
        for candidate in red_result.get("red_ball_candidates", [])
        if (
            candidate.get("center_offset_ratio") is not None
            and _candidate_visible_size(candidate) is not None
        )
    ]
    if not ball_candidates:
        return None

    return max(
        ball_candidates,
        key=lambda candidate: (
            float(_candidate_visible_size(candidate)),
            float(candidate.get("score", 0.0)),
            -abs(float(candidate["center_offset_ratio"])),
        ),
    )


def _select_farthest_red_ball(red_result: dict[str, Any]):
    """見かけサイズから、遠そうな赤ボール候補を選ぶ。"""
    ball_candidates = [
        candidate
        for candidate in red_result.get("red_ball_candidates", [])
        if (
            candidate.get("center_offset_ratio") is not None
            and _candidate_visible_size(candidate) is not None
        )
    ]
    if not ball_candidates:
        return None

    return min(
        ball_candidates,
        key=lambda candidate: (
            float(_candidate_visible_size(candidate)),
            abs(float(candidate["center_offset_ratio"])),
        ),
    )


def _classify_selected_ball_position(
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


def _candidate_delta_x(candidate: dict[str, Any], target_hint_x: float) -> float:
    return abs(float(candidate["x"]) - float(target_hint_x))


def _red_ball_lock_score(
    candidate: dict[str, Any],
    target_hint_x: float,
    target_hint_size_px: float,
    position_scale_px: float,
    position_weight: float,
    size_weight: float,
) -> float:
    """前回位置と、前進で小さくならないサイズ変化から同じボールらしさを評価する。"""
    position_similarity = math.exp(
        -_candidate_delta_x(candidate, target_hint_x) / max(
            float(position_scale_px),
            1.0,
        )
    )
    visible_size = _candidate_visible_size(candidate)
    if (
        visible_size is None
        or visible_size <= 0.0
        or target_hint_size_px <= 0.0
    ):
        size_similarity = 0.0
    else:
        size_similarity = min(visible_size / target_hint_size_px, 1.0)

    position_weight = max(0.0, float(position_weight))
    size_weight = max(0.0, float(size_weight))
    total_weight = max(
        position_weight + size_weight,
        1.0,
    )
    return (
        position_weight * position_similarity
        + size_weight * size_similarity
    ) / total_weight


def _predict_target_hint_x_after_rotation(
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


def _predict_target_hint_x_after_forward(
    ball: dict[str, Any],
    heading_before_deg: float,
    heading_after_deg: float,
    horizontal_fov_deg: float,
    image_width: float | None,
) -> tuple[float | None, float]:
    """前進前後の実方位差から、停止後のターゲットx座標を予測する。"""
    heading_change_deg = NavigationController.heading_error(
        heading_after_deg,
        heading_before_deg,
    )
    predicted_x = _predict_target_hint_x_after_rotation(
        ball,
        heading_change_deg,
        horizontal_fov_deg,
        image_width,
    )
    return predicted_x, heading_change_deg


def _select_red_ball_near_hint(
    red_result: dict[str, Any],
    target_hint_x: float,
    position_scale_px: float,
    *,
    target_hint_size_px: float | None = None,
    position_weight: float = 1.0,
    size_weight: float = 1.0,
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
            key=lambda candidate: _candidate_delta_x(candidate, target_hint_x),
        )

    selected_ball = max(
        ball_candidates,
        key=lambda candidate: _red_ball_lock_score(
            candidate,
            target_hint_x,
            target_hint_size_px,
            position_scale_px,
            position_weight,
            size_weight,
        ),
    )
    selected_ball = selected_ball.copy()
    selected_ball["target_lock_score"] = _red_ball_lock_score(
        selected_ball,
        target_hint_x,
        target_hint_size_px,
        position_scale_px,
        position_weight,
        size_weight,
    )
    return selected_ball


def _detect_red_balls(
    processor: ImageProcessor,
    frame: Any,
) -> dict[str, Any]:
    """1回の赤色解析から円候補とサイズ候補をまとめて返す。"""
    circle_candidates = processor.detect_red_ball_circle_candidates(frame)
    color_result = processor.detect_color(
        frame,
        hsv_ranges=processor.RED_BALL_HSV_RANGES,
        color_threshold=RedBallConfig.SWITCH_RED_RATIO,
        column_threshold=RedBallConfig.RED_COLUMN_THRESHOLD,
        column_average_width=RedBallConfig.RED_COLUMN_AVERAGE_WIDTH,
    )
    size_candidates = processor.detect_red_ball_candidates(
        frame,
        color_result=color_result,
    )
    merged_candidates = _merge_red_ball_candidates(
        circle_candidates,
        size_candidates,
    )
    return {
        "is_color_detected": bool(merged_candidates),
        "total_color_ratio": color_result["total_color_ratio"],
        "image_width": color_result["image_width"],
        "red_ball_candidates": merged_candidates,
    }


def align_red_ball_to_center(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    *,
    target_hint_x: float | None = None,
    target_hint_size_px: float | None = None,
    distance_m: float | None = None,
) -> dict[str, Any]:
    """カメラの横ずれを補正し、同じ赤ボールを機体正面へ合わせる。"""
    processor = ImageProcessor()
    red_ball_config = RedBallConfig()
    local_target_hint_x = target_hint_x
    local_target_hint_size_px = target_hint_size_px
    initial_selected_position = None
    target_angle_deg = 0.0
    if distance_m is not None:
        ball_center_distance_m = (
            max(0.0, float(distance_m))
            + red_ball_config.RED_BALL_RADIUS_M
        )
        target_angle_deg = math.degrees(
            math.atan2(
                -red_ball_config.CAMERA_LATERAL_OFFSET_M,
                ball_center_distance_m,
            )
        )

    for step in range(red_ball_config.MAX_CENTERING_STEPS):
        print(
            "赤ボール中央合わせ: "
            f"step {step + 1}/{red_ball_config.MAX_CENTERING_STEPS} 撮影します"
        )
        frame = sensor_manager.capture_front_frame()
        red_result = _detect_red_balls(processor, frame)
        selected_ball = None
        selected_by_hint = False
        if local_target_hint_x is not None:
            selected_ball = _select_red_ball_near_hint(
                red_result,
                local_target_hint_x,
                red_ball_config.CENTERING_TARGET_LOCK_POSITION_SCALE_PX,
                target_hint_size_px=local_target_hint_size_px,
                position_weight=(
                    red_ball_config.CENTERING_TARGET_LOCK_POSITION_WEIGHT
                ),
                size_weight=(
                    red_ball_config.CENTERING_TARGET_LOCK_SIZE_WEIGHT
                ),
            )
            selected_by_hint = selected_ball is not None
        if selected_ball is None and local_target_hint_x is not None:
            print(
                "赤ボール中央合わせ: 候補がないためロックを解除し、"
                "次の撮影で最も近く見える候補を選び直します"
            )
            local_target_hint_x = None
            local_target_hint_size_px = None
            continue
        if selected_ball is None:
            selected_ball = _select_nearest_red_ball(red_result)
        if selected_ball is None:
            reason = "赤ボールを認識できませんでした"
            print(f"赤ボール中央合わせ: {reason}")
            return {
                "centered": False,
                "red_detected": False,
                "reason": reason,
                "steps": step + 1,
                "last_red_result": red_result,
                "initial_selected_position": initial_selected_position,
            }

        if initial_selected_position is None:
            initial_selected_position = _classify_selected_ball_position(
                red_result,
                selected_ball,
            )
            position_text = {
                "left": "左寄り",
                "center": "真ん中",
                "right": "右寄り",
            }[initial_selected_position]
            print(
                "赤ボール中央合わせ: "
                f"初回選択位置={position_text}"
                f"({initial_selected_position}), "
                "candidate_count="
                f"{len(red_result.get('red_ball_candidates', []))}"
            )
        red_result["selected_red_ball"] = selected_ball
        local_target_hint_x = float(selected_ball["x"])
        selected_size_px = _candidate_visible_size(selected_ball)
        if selected_size_px is not None:
            local_target_hint_size_px = selected_size_px

        detected_angle_deg = (
            float(selected_ball["center_offset_ratio"])
            * red_ball_config.HORIZONTAL_FOV_DEG
        )
        turn_angle = detected_angle_deg - target_angle_deg

        turn_gain = red_ball_config.CENTERING_TURN_GAIN
        if abs(turn_angle) >= red_ball_config.CENTERING_FULL_GAIN_ANGLE_DEG:
            turn_gain = red_ball_config.CENTERING_LARGE_ANGLE_TURN_GAIN
        rotate_angle = turn_angle * turn_gain

        print(
            "赤ボール中央合わせ: "
            f"total={red_result['total_color_ratio'] * 100:.2f}% "
            f"ball_x={selected_ball['x']:.1f} "
            f"detected={detected_angle_deg:.2f}deg "
            f"target={target_angle_deg:.2f}deg "
            f"turn={turn_angle:.2f}deg "
            f"gain={turn_gain:.2f} "
            f"locked={selected_by_hint} "
            f"score={selected_ball.get('target_lock_score', 1.0):.3f} "
            f"rotate={rotate_angle:.2f}deg"
        )

        if abs(turn_angle) <= red_ball_config.CENTERING_TOLERANCE_DEG:
            return {
                "centered": True,
                "red_detected": True,
                "reason": "赤ボールが機体正面の補正位置に入りました",
                "steps": step + 1,
                "last_red_result": red_result,
                "initial_selected_position": initial_selected_position,
            }

        rotate_result = navigation_controller.rotate_by_angle(
            driver,
            sensor_manager,
            turn_angle,
            turn_gain=turn_gain,
            speed=red_ball_config.CENTERING_ROTATE_SPEED,
            tolerance_deg=red_ball_config.CENTERING_ROTATE_TOLERANCE_DEG,
            timeout_s=red_ball_config.ROTATE_TIMEOUT_S,
        )
        predicted_hint_x = _predict_target_hint_x_after_rotation(
            selected_ball,
            turn_angle,
            red_ball_config.HORIZONTAL_FOV_DEG,
            red_result.get("image_width"),
        )
        if predicted_hint_x is not None:
            local_target_hint_x = predicted_hint_x

    return {
        "centered": False,
        "red_detected": True,
        "reason": "最大試行回数内に中央合わせできませんでした",
        "steps": red_ball_config.MAX_CENTERING_STEPS,
        "last_red_result": red_result if "red_result" in locals() else None,
        "initial_selected_position": initial_selected_position,
    }


def _duration_from_threshold(value, default_duration_s, duration_table):
    """しきい値表から1回の前進時間を選ぶ。"""
    value = float(value)
    for threshold, duration_s in duration_table:
        if value > threshold:
            return duration_s
    return default_duration_s


def _reverse_for_duration(driver: Any, speed: float, duration_s: float) -> None:
    """短時間だけ後退する。"""
    speed = max(0.0, min(float(speed), 100.0))
    duration_s = max(0.0, float(duration_s))
    if speed == 0.0 or duration_s == 0.0:
        driver.stop()
        return

    try:
        driver.drive(-speed)
        time.sleep(duration_s)
    finally:
        driver.stop()


def _select_adjacent_red_ball(
    red_result: dict[str, Any],
    horizontal_fov_deg: float,
    min_angle_deg: float | None,
):
    """中央の対象を除き、最も近そうに見える隣の赤ボールを選ぶ。"""
    adjacent_balls = []
    if min_angle_deg is not None:
        min_angle_deg = float(min_angle_deg)
    for ball in red_result.get("red_ball_candidates", []):
        offset_ratio = ball.get("center_offset_ratio")
        visible_size = _candidate_visible_size(ball)
        if offset_ratio is None or visible_size is None:
            continue

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


def _approach_red_ball_to_distance(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    target_distance_m: float,
    *,
    target_hint_x: float | None = None,
    target_hint_size_px: float | None = None,
    initial_centering_distance_m: float | None = None,
    log_prefix: str = "赤ボール接近",
) -> dict[str, Any]:
    """赤ボールを中央に合わせ、設定された許容範囲内まで接近する。"""
    red_ball_config = RedBallConfig()
    red_cone_config = RedConeConfig()
    target_distance_m = float(target_distance_m)
    tolerance_m = float(red_ball_config.DISTANCE_TOLERANCE_M)
    stop_distance_m = target_distance_m + tolerance_m
    too_close_distance_m = target_distance_m - tolerance_m
    history = []
    last_distance_m = (
        None
        if initial_centering_distance_m is None
        else float(initial_centering_distance_m)
    )
    last_center_result = None

    for step in range(1, red_ball_config.MAX_APPROACH_STEPS + 1):
        center_result = align_red_ball_to_center(
            navigation_controller,
            driver,
            sensor_manager,
            target_hint_x=target_hint_x,
            target_hint_size_px=target_hint_size_px,
            distance_m=last_distance_m,
        )
        last_center_result = center_result
        approach_record = {
            "approach_step": step,
            "centering_result": center_result,
            "centering_distance_m": last_distance_m,
            "distance_m": None,
            "forward_duration_s": None,
        }
        history.append(approach_record)
        if not center_result["centered"]:
            return {
                "reached": False,
                "reason": center_result["reason"],
                "steps": step,
                "last_distance_m": last_distance_m,
                "centering_result": center_result,
                "last_red_result": center_result.get("last_red_result"),
                "history": history,
            }

        last_red_result = center_result.get("last_red_result") or {}
        selected_ball = last_red_result.get("selected_red_ball")
        if selected_ball is not None:
            target_hint_x = float(selected_ball["x"])
            selected_size_px = _candidate_visible_size(selected_ball)
            if selected_size_px is not None:
                target_hint_size_px = selected_size_px

        distance_m = sensor_manager.get_distance_m()
        if distance_m is None:
            driver.stop()
            return {
                "reached": False,
                "reason": "距離を測定できませんでした",
                "steps": step,
                "last_distance_m": last_distance_m,
                "centering_result": center_result,
                "last_red_result": last_red_result,
                "history": history,
            }

        last_distance_m = float(distance_m)
        distance_error_m = last_distance_m - target_distance_m
        approach_record["distance_m"] = last_distance_m
        approach_record["distance_error_m"] = distance_error_m
        print(
            f"{log_prefix}: distance={last_distance_m:.3f}m, "
            f"target={target_distance_m:.3f}m, "
            f"error={distance_error_m:.3f}m"
        )
        if last_distance_m < too_close_distance_m:
            print(
                f"{log_prefix}: 近すぎるため"
                f"{red_ball_config.REVERSE_DURATION_S:.2f}秒後退します"
            )
            _reverse_for_duration(
                driver,
                red_ball_config.REVERSE_SPEED,
                red_ball_config.REVERSE_DURATION_S,
            )
            continue

        if last_distance_m <= stop_distance_m:
            driver.stop()
            return {
                "reached": True,
                "reason": "目標距離に到達しました",
                "steps": step,
                "last_distance_m": last_distance_m,
                "centering_result": center_result,
                "last_red_result": last_red_result,
                "history": history,
            }

        forward_duration = _duration_from_threshold(
            distance_error_m,
            red_ball_config.FORWARD_DURATION_S,
            red_ball_config.FORWARD_DURATION_BY_DISTANCE_ERROR_M,
        )
        approach_record["forward_duration_s"] = forward_duration
        print(f"{log_prefix}: 前進 {forward_duration:.2f}秒")
        heading_before_deg = float(sensor_manager.get_heading_deg())
        navigation_controller.follow_forward(
            driver,
            sensor_manager,
            forward_duration,
            base_speed=red_cone_config.FORWARD_SPEED,
            loop_interval=red_cone_config.LOOP_INTERVAL_S,
        )
        heading_after_deg = float(sensor_manager.get_heading_deg())
        predicted_x, heading_change_deg = _predict_target_hint_x_after_forward(
            selected_ball,
            heading_before_deg,
            heading_after_deg,
            red_ball_config.HORIZONTAL_FOV_DEG,
            last_red_result.get("image_width"),
        )
        if predicted_x is not None:
            target_hint_x = predicted_x
        approach_record["heading_change_deg"] = heading_change_deg
        approach_record["predicted_target_x"] = target_hint_x
        target_x_text = (
            "None" if target_hint_x is None else f"{target_hint_x:.1f}"
        )
        print(
            f"{log_prefix}: 前進後方位差={heading_change_deg:+.2f}deg, "
            f"予測ball_x={target_x_text}"
        )

    return {
        "reached": False,
        "reason": "最大試行回数内に目標距離まで近づけませんでした",
        "steps": red_ball_config.MAX_APPROACH_STEPS,
        "last_distance_m": last_distance_m,
        "centering_result": last_center_result,
        "last_red_result": (
            (last_center_result or {}).get("last_red_result")
        ),
        "history": history,
    }


def _rotate_by_angle_precisely(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    target_angle_deg: float,
) -> dict[str, Any]:
    """停止後の惰性を含むIMU実回転角から残差を求めて微調整する。"""
    red_ball_config = RedBallConfig()
    target_angle_deg = float(target_angle_deg)
    settle_time_s = 0.5
    tolerance_deg = float(
        red_ball_config.CENTERING_ROTATE_TOLERANCE_DEG
    )
    rotated_angle_deg = 0.0
    history = []

    for correction_index in range(red_ball_config.MAX_CENTERING_STEPS):
        remaining_angle_deg = target_angle_deg - rotated_angle_deg
        if abs(remaining_angle_deg) <= tolerance_deg:
            break

        turn_gain = 1.0
        if correction_index > 0:
            turn_gain = red_ball_config.CENTERING_TURN_GAIN
            if (
                abs(remaining_angle_deg)
                >= red_ball_config.CENTERING_FULL_GAIN_ANGLE_DEG
            ):
                turn_gain = (
                    red_ball_config.CENTERING_LARGE_ANGLE_TURN_GAIN
                )
        print(
            "精密旋回: "
            f"step={correction_index + 1}, "
            f"remaining={remaining_angle_deg:+.2f}deg, "
            f"gain={turn_gain:.2f}",
            flush=True,
        )
        heading_before_deg = float(sensor_manager.get_heading_deg())
        rotate_result = navigation_controller.rotate_by_angle(
            driver,
            sensor_manager,
            remaining_angle_deg,
            turn_gain=turn_gain,
            speed=red_ball_config.CENTERING_ROTATE_SPEED,
            tolerance_deg=tolerance_deg,
            timeout_s=red_ball_config.ROTATE_TIMEOUT_S,
        )
        time.sleep(settle_time_s)
        settled_heading_deg = float(sensor_manager.get_heading_deg())
        reported_rotated_angle_deg = float(
            rotate_result.get("rotated_angle_deg", 0.0)
        )
        wrapped_settled_angle_deg = navigation_controller.heading_error(
            settled_heading_deg,
            heading_before_deg,
        )
        settled_rotated_angle_deg = min(
            (
                wrapped_settled_angle_deg - 360.0,
                wrapped_settled_angle_deg,
                wrapped_settled_angle_deg + 360.0,
            ),
            key=lambda angle: abs(angle - reported_rotated_angle_deg),
        )
        rotated_angle_deg += settled_rotated_angle_deg
        settled_rotate_result = {
            **rotate_result,
            "heading_before_deg": heading_before_deg,
            "settled_heading_deg": settled_heading_deg,
            "reported_rotated_angle_deg": reported_rotated_angle_deg,
            "settled_rotated_angle_deg": settled_rotated_angle_deg,
            "settle_time_s": settle_time_s,
        }
        history.append(settled_rotate_result)
        print(
            "精密旋回: 停止後確認 "
            f"wait={settle_time_s:.2f}s, "
            f"reported={reported_rotated_angle_deg:+.2f}deg, "
            f"settled={settled_rotated_angle_deg:+.2f}deg, "
            f"total={rotated_angle_deg:+.2f}deg",
            flush=True,
        )
        if not rotate_result["reached"]:
            break

    remaining_angle_deg = target_angle_deg - rotated_angle_deg
    return {
        "target_angle_deg": target_angle_deg,
        "rotated_angle_deg": rotated_angle_deg,
        "remaining_angle_deg": remaining_angle_deg,
        "reached": abs(remaining_angle_deg) <= tolerance_deg,
        "correction_history": history,
    }


def guide_to_red_cone(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    stop_red_ratio_threshold: float | None = None,
    forward_duration_by_red_ratio: tuple[tuple[float, float], ...] | None = None,
) -> dict[str, Any]:
    """NavigationControllerを使って赤コーンを探し、正面へ回頭して前進する。"""
    processor = ImageProcessor()
    red_cone_config = RedConeConfig()
    forward_duration_by_red_ratio = tuple(
        sorted(
            (
                forward_duration_by_red_ratio
                or red_cone_config.FORWARD_DURATION_BY_RED_RATIO
            ),
            reverse=True,
        )
    )

    history = []
    last_goal_result = None

    for step in range(red_cone_config.MAX_GUIDANCE_STEPS):
        print(
            "赤コーン誘導: "
            f"step {step + 1}/{red_cone_config.MAX_GUIDANCE_STEPS} 探索開始"
        )

        # 1. 赤コーンが画面に入るまで、撮影と少しの旋回を繰り返す。
        red_result, scan_history = _find_red_cone_in_view(
            navigation_controller,
            driver,
            sensor_manager,
            processor,
            red_cone_config,
        )

        if red_result is None:
            return {
                "goal_reached": False,
                "reason": "赤コーンを見つけられませんでした",
                "steps": step,
                "history": history,
                "scan_history": scan_history,
                "last_goal_result": last_goal_result,
            }

        if (
            stop_red_ratio_threshold is not None
            and red_result["total_color_ratio"] >= stop_red_ratio_threshold
        ):
            return {
                "goal_reached": False,
                "red_ratio_threshold_reached": True,
                "reason": "赤検知率が切り替えしきい値以上になりました",
                "steps": step + 1,
                "history": history,
                "last_red_result": red_result,
                "last_goal_result": last_goal_result,
            }

        # 2. 赤コーンの画面内位置から、正面へ向けるための旋回角度を決める。
        offset_ratio = red_result.get("color_peak_center_offset_ratio")
        turn_angle = (
            0.0
            if offset_ratio is None
            else float(offset_ratio) * red_cone_config.HORIZONTAL_FOV_DEG
        )
        turn_result = None
        if turn_angle != 0.0:
            turn_result = navigation_controller.rotate_by_angle(
                driver,
                sensor_manager,
                turn_angle,
                speed=red_cone_config.ROTATE_SPEED,
                tolerance_deg=red_cone_config.ROTATE_TOLERANCE_DEG,
                timeout_s=red_cone_config.ROTATE_TIMEOUT_S,
            )

        # 3. 赤色が大きく見えているほど近いとみなし、前進時間を短くする。
        forward_duration = _duration_from_threshold(
            red_result["total_color_ratio"],
            red_cone_config.FORWARD_DURATION_S,
            forward_duration_by_red_ratio,
        )

        print(
            "赤コーン誘導: "
            f"前進 {forward_duration:.2f}秒 "
            f"(total={red_result['total_color_ratio'] * 100:.2f}%, "
            f"column={red_result['color_peak_column_x']}, "
            f"turn={turn_angle:.2f}deg)"
        )
        navigation_controller.follow_forward(
            driver,
            sensor_manager,
            forward_duration,
            base_speed=red_cone_config.FORWARD_SPEED,
            loop_interval=red_cone_config.LOOP_INTERVAL_S,
        )

        # 4. 前進後にもう一度撮影し、赤コーンに十分近づいたか判定する。
        print("赤コーン誘導: ゴール判定用に撮影します")
        goal_frame = sensor_manager.capture_front_frame()
        last_goal_result = _without_color_mask(
            processor.judge_red_goal_reached(
                goal_frame,
                red_threshold=red_cone_config.RED_THRESHOLD,
                goal_angle_red_threshold=(
                    red_cone_config.GOAL_ANGLE_RED_THRESHOLD
                ),
                horizontal_fov_deg=red_cone_config.HORIZONTAL_FOV_DEG,
                goal_angle_min_deg=red_cone_config.GOAL_ANGLE_MIN_DEG,
                goal_angle_max_deg=red_cone_config.GOAL_ANGLE_MAX_DEG,
            )
        )
        print(
            "赤コーン誘導: "
            f"ゴール判定 reached={last_goal_result['goal_reached']} "
            f"total={last_goal_result['total_color_ratio'] * 100:.2f}% "
            f"angle={last_goal_result['goal_angle_color_ratio'] * 100:.2f}% "
            f"range={last_goal_result['goal_angle_min_deg']:.1f}"
            f"..{last_goal_result['goal_angle_max_deg']:.1f}deg"
        )

        history.append({
            "step": step + 1,
            "red_result": red_result,
            "turn_angle_deg": turn_angle,
            "turn_result": turn_result,
            "forward_duration_s": forward_duration,
            "goal_result": last_goal_result,
            "scan_history": scan_history,
        })

        # ゴール判定が出たら、最後に少し前進して終了する。
        if last_goal_result["goal_reached"]:
            print(
                "赤コーン誘導: "
                f"ゴール判定成功。最後に"
                f"{red_cone_config.GOAL_FINAL_FORWARD_DURATION_S:.2f}"
                "秒前進します"
            )
            navigation_controller.follow_forward(
                driver,
                sensor_manager,
                red_cone_config.GOAL_FINAL_FORWARD_DURATION_S,
                base_speed=red_cone_config.FORWARD_SPEED,
                loop_interval=red_cone_config.LOOP_INTERVAL_S,
            )
            return {
                "goal_reached": True,
                "reason": last_goal_result["goal_reason"],
                "steps": step + 1,
                "history": history,
                "last_goal_result": last_goal_result,
            }

    return {
        "goal_reached": False,
        "red_ratio_threshold_reached": False,
        "reason": "最大試行回数内にゴール判定できませんでした",
        "steps": red_cone_config.MAX_GUIDANCE_STEPS,
        "history": history,
        "last_goal_result": last_goal_result,
    }


def guide_to_red_ball(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
) -> dict[str, Any]:
    """最初の赤ボールへ誘導し、距離センサで目標距離付近まで近づく。"""
    red_ball_config = RedBallConfig()

    cone_result = guide_to_red_cone(
        navigation_controller,
        driver,
        sensor_manager,
        stop_red_ratio_threshold=red_ball_config.SWITCH_RED_RATIO,
        forward_duration_by_red_ratio=(
            red_ball_config.CONE_FORWARD_DURATION_BY_RED_RATIO
        ),
    )
    if not cone_result.get("red_ratio_threshold_reached"):
        return {
            "target_reached": False,
            "reason": cone_result["reason"],
            "cone_result": cone_result,
            "steps": 0,
            "last_distance_m": None,
            "initial_ball_position": None,
        }

    target_distance_m = float(red_ball_config.TARGET_DISTANCE_M)
    approach_result = _approach_red_ball_to_distance(
        navigation_controller,
        driver,
        sensor_manager,
        target_distance_m,
        log_prefix="赤ボール誘導",
    )
    first_centering_result = (
        approach_result["history"][0]["centering_result"]
        if approach_result["history"]
        else {}
    )
    initial_ball_position = first_centering_result.get(
        "initial_selected_position"
    )
    print(f"赤ボール誘導: 初回選択位置={initial_ball_position}")
    return {
        "target_reached": approach_result["reached"],
        "reason": (
            "目標距離範囲に入りました"
            if approach_result["reached"]
            else approach_result["reason"]
        ),
        "cone_result": cone_result,
        "centering_result": approach_result["centering_result"],
        "steps": approach_result["steps"],
        "last_distance_m": approach_result["last_distance_m"],
        "target_distance_m": target_distance_m,
        "target_tolerance_m": float(
            red_ball_config.DISTANCE_TOLERANCE_M
        ),
        "initial_ball_position": initial_ball_position,
        "approach_history": approach_result["history"],
    }


def guide_to_square_zone(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    *,
    initial_ball_position: str | None = None,
) -> dict[str, Any]:
    """最初の赤ボール到達後、隣の赤ボールへ順に近づく。"""
    processor = ImageProcessor()
    red_ball_config = RedBallConfig()

    history: list[dict[str, Any]] = []
    last_distance_m = None
    initial_turn_angle_deg = None
    initial_turn_result = None
    initial_fallback_turn_result = None
    unrestricted_first_selection = False

    try:
        if initial_ball_position in ("left", "right"):
            initial_side_turn_angle_deg = float(
                red_ball_config.INITIAL_SIDE_TURN_ANGLE_DEG
            )
            initial_turn_angle_deg = (
                initial_side_turn_angle_deg
                if initial_ball_position == "left"
                else -initial_side_turn_angle_deg
            )
            print(
                "スクエアゾーン誘導: 初回ボール位置="
                f"{initial_ball_position}, "
                f"事前旋回={initial_turn_angle_deg:+.1f}deg"
            )
            initial_turn_result = _rotate_by_angle_precisely(
                navigation_controller,
                driver,
                sensor_manager,
                initial_turn_angle_deg,
            )
            if not initial_turn_result["reached"]:
                return {
                    "square_zone_reached": False,
                    "reason": (
                        "スクエアゾーン進入前の"
                        f"{initial_side_turn_angle_deg:.1f}度旋回に"
                        "失敗しました"
                    ),
                    "approached_balls": 0,
                    "last_distance_m": None,
                    "last_red_result": None,
                    "initial_ball_position": initial_ball_position,
                    "initial_turn_result": initial_turn_result,
                    "history": history,
                }
            unrestricted_first_selection = True

        for target_index in range(1, red_ball_config.MAX_SQUARE_TARGETS + 1):
            print(
                "スクエアゾーン誘導: "
                f"target {target_index}/"
                f"{red_ball_config.MAX_SQUARE_TARGETS} 撮影します"
            )
            frame = sensor_manager.capture_front_frame()
            red_result = _detect_red_balls(processor, frame)
            visible_target_count = len(red_result["red_ball_candidates"])
            unrestricted_selection = (
                unrestricted_first_selection and target_index == 1
            )
            min_adjacent_angle_deg = (
                None
                if unrestricted_selection
                else max(
                    red_ball_config.CENTERING_TOLERANCE_DEG,
                    red_ball_config.ADJACENT_MIN_ANGLE_DEG,
                )
            )
            adjacent_ball, turn_angle = _select_adjacent_red_ball(
                red_result,
                red_ball_config.HORIZONTAL_FOV_DEG,
                min_adjacent_angle_deg,
            )
            fallback_search_performed = False
            if unrestricted_selection and adjacent_ball is None:
                fallback_turn_angle_deg = -2.0 * initial_turn_angle_deg
                print(
                    "スクエアゾーン誘導: 事前旋回後に候補がないため、"
                    "直前と逆方向へ"
                    f"{abs(fallback_turn_angle_deg):.1f}deg旋回して"
                    "再探索します"
                )
                initial_fallback_turn_result = _rotate_by_angle_precisely(
                    navigation_controller,
                    driver,
                    sensor_manager,
                    fallback_turn_angle_deg,
                )
                if not initial_fallback_turn_result["reached"]:
                    return {
                        "square_zone_reached": False,
                        "reason": "初回ボールの反対側探索旋回に失敗しました",
                        "approached_balls": 0,
                        "last_distance_m": None,
                        "last_red_result": red_result,
                        "initial_ball_position": initial_ball_position,
                        "initial_turn_result": initial_turn_result,
                        "initial_fallback_turn_result": (
                            initial_fallback_turn_result
                        ),
                        "history": history,
                    }

                fallback_search_performed = True
                frame = sensor_manager.capture_front_frame()
                red_result = _detect_red_balls(processor, frame)
                visible_target_count = len(
                    red_result["red_ball_candidates"]
                )
                adjacent_ball, turn_angle = _select_adjacent_red_ball(
                    red_result,
                    red_ball_config.HORIZONTAL_FOV_DEG,
                    min_adjacent_angle_deg,
                )
            commanded_turn_angle = (
                None
                if turn_angle is None
                else turn_angle
                * red_ball_config.CENTERING_LARGE_ANGLE_TURN_GAIN
            )
            target_history: dict[str, Any] = {
                "target_index": target_index,
                "red_result": red_result,
                "adjacent_ball": adjacent_ball,
                "detected_turn_angle_deg": turn_angle,
                "turn_angle_deg": turn_angle,
                "turn_gain": red_ball_config.CENTERING_LARGE_ANGLE_TURN_GAIN,
                "rotation_stop_angle_deg": commanded_turn_angle,
                "rotate_result": None,
                "approach_history": [],
                "initial_ball_position": initial_ball_position,
                "initial_turn_result": initial_turn_result,
                "initial_fallback_turn_result": (
                    initial_fallback_turn_result
                ),
                "fallback_search_performed": fallback_search_performed,
                "unrestricted_selection": unrestricted_selection,
            }
            history.append(target_history)

            min_angle_text = (
                "none"
                if min_adjacent_angle_deg is None
                else f"{min_adjacent_angle_deg:.1f}deg"
            )
            print(
                "スクエアゾーン誘導: "
                f"candidate_count={visible_target_count}, "
                f"min_adjacent_angle={min_angle_text}, "
                f"adjacent_ball={None if adjacent_ball is None else adjacent_ball['x']}"
            )

            if (
                adjacent_ball is None
                or (
                    visible_target_count < 2
                    and not unrestricted_selection
                )
            ):
                final_target_distance_m = float(
                    red_ball_config.FINAL_TARGET_DISTANCE_M
                )
                front_ball = _select_nearest_red_ball(red_result)
                if front_ball is None:
                    return {
                        "square_zone_reached": False,
                        "reason": (
                            "逆方向探索後も赤ボール候補を"
                            "認識できませんでした"
                            if fallback_search_performed
                            else "正面の赤ボールを認識できませんでした"
                        ),
                        "approached_balls": target_index - 1,
                        "last_distance_m": last_distance_m,
                        "last_red_result": red_result,
                        "history": history,
                    }

                print(
                    "スクエアゾーン誘導: 終了条件を確認しました。"
                    f"正面の赤ボールまで{final_target_distance_m:.3f}mに"
                    "近づきます"
                )
                final_initial_distance_m = sensor_manager.get_distance_m()
                if final_initial_distance_m is None:
                    driver.stop()
                    return {
                        "square_zone_reached": False,
                        "reason": "距離を測定できませんでした",
                        "approached_balls": target_index - 1,
                        "last_distance_m": last_distance_m,
                        "last_red_result": red_result,
                        "history": history,
                    }
                final_initial_distance_m = float(final_initial_distance_m)
                print(
                    "スクエアゾーン誘導: 最終接近の初回補正距離="
                    f"{final_initial_distance_m:.3f}m"
                )
                final_approach_result = _approach_red_ball_to_distance(
                    navigation_controller,
                    driver,
                    sensor_manager,
                    final_target_distance_m,
                    target_hint_x=float(front_ball["x"]),
                    target_hint_size_px=_candidate_visible_size(front_ball),
                    initial_centering_distance_m=final_initial_distance_m,
                    log_prefix="スクエアゾーン最終接近",
                )
                target_history["final_approach_history"] = (
                    final_approach_result["history"]
                )
                return {
                    "square_zone_reached": final_approach_result["reached"],
                    "reason": (
                        "正面の赤ボールまで"
                        f"{final_target_distance_m:.3f}mに到達しました"
                        if final_approach_result["reached"]
                        else final_approach_result["reason"]
                    ),
                    "approached_balls": target_index - 1,
                    "last_distance_m": (
                        final_approach_result["last_distance_m"]
                    ),
                    "last_red_result": (
                        final_approach_result["last_red_result"]
                    ),
                    "history": history,
                }

            print(
                "スクエアゾーン誘導: "
                f"隣の赤ボール方向={turn_angle:.2f}deg, "
                f"旋回指令={commanded_turn_angle:.2f}deg"
            )
            rotate_result = navigation_controller.rotate_by_angle(
                driver,
                sensor_manager,
                turn_angle,
                turn_gain=red_ball_config.CENTERING_LARGE_ANGLE_TURN_GAIN,
                speed=red_ball_config.CENTERING_ROTATE_SPEED,
                tolerance_deg=red_ball_config.CENTERING_ROTATE_TOLERANCE_DEG,
                timeout_s=red_ball_config.ROTATE_TIMEOUT_S,
            )
            target_history["rotate_result"] = rotate_result
            if not rotate_result["reached"]:
                return {
                    "square_zone_reached": False,
                    "reason": "隣の赤ボールへの旋回が完了しませんでした",
                    "approached_balls": target_index - 1,
                    "last_distance_m": last_distance_m,
                    "last_red_result": red_result,
                    "history": history,
                }

            target_hint_x = _predict_target_hint_x_after_rotation(
                adjacent_ball,
                turn_angle,
                red_ball_config.HORIZONTAL_FOV_DEG,
                red_result.get("image_width"),
            )
            target_hint_size_px = _candidate_visible_size(adjacent_ball)
            print(
                "スクエアゾーン誘導: "
                f"{red_ball_config.TARGET_DISTANCE_M:.3f}mまで"
                "中央合わせしながら前進します"
            )
            approach_result = _approach_red_ball_to_distance(
                navigation_controller,
                driver,
                sensor_manager,
                red_ball_config.TARGET_DISTANCE_M,
                target_hint_x=target_hint_x,
                target_hint_size_px=target_hint_size_px,
                log_prefix="スクエアゾーン誘導",
            )
            target_history["approach_history"] = approach_result["history"]
            last_distance_m = approach_result["last_distance_m"]
            if not approach_result["reached"]:
                return {
                    "square_zone_reached": False,
                    "reason": approach_result["reason"],
                    "approached_balls": target_index - 1,
                    "last_distance_m": last_distance_m,
                    "last_red_result": approach_result["last_red_result"],
                    "history": history,
                }
    finally:
        driver.stop()

    return {
        "square_zone_reached": False,
        "reason": "最大対象数まで誘導しても終了条件に到達しませんでした",
        "approached_balls": red_ball_config.MAX_SQUARE_TARGETS,
        "last_distance_m": last_distance_m,
        "last_red_result": history[-1]["red_result"] if history else None,
        "history": history,
    }


def guide_to_center_of_zone(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
) -> dict[str, Any]:
    """対角の赤ボールを基準に、スクエアゾーンの中心へ移動する。"""
    processor = ImageProcessor()
    red_ball_config = RedBallConfig()
    repeat_count = max(
        0, int(red_ball_config.CENTER_OF_ZONE_REPEAT_COUNT)
    )
    diagonal_min_distance_m = 0.55
    diagonal_max_distance_m = 1.2
    history = []
    last_approach_result = None

    try:
        for cycle_index in range(repeat_count + 1):
            cycle_history: dict[str, Any] = {
                "cycle": cycle_index + 1,
                "turn_180_result": None,
                "far_ball_result": None,
                "turn_direction": None,
                "alignment_result": None,
                "measured_distance_m": None,
                "is_diagonal_ball": False,
                "opposite_turn_result": None,
                "fallback_turn_result": None,
                "approach_result": None,
            }
            history.append(cycle_history)

            print(
                "スクエアゾーン中心誘導: "
                f"cycle {cycle_index + 1}/{repeat_count + 1} "
                "180度転回を開始します",
                flush=True,
            )
            turn_180_result = _rotate_by_angle_precisely(
                navigation_controller,
                driver,
                sensor_manager,
                180.0,
            )
            cycle_history["turn_180_result"] = turn_180_result
            print(
                "スクエアゾーン中心誘導: "
                f"180度転回 reached={turn_180_result['reached']}",
                flush=True,
            )
            if not turn_180_result["reached"]:
                return {
                    "center_reached": False,
                    "reason": "180度転回が完了しませんでした",
                    "history": history,
                    "last_distance_m": None,
                }

            frame = sensor_manager.capture_front_frame()
            red_result = _detect_red_balls(processor, frame)
            far_ball = _select_farthest_red_ball(red_result)
            cycle_history["far_ball_result"] = red_result
            if far_ball is None:
                return {
                    "center_reached": False,
                    "reason": "遠方の赤ボールを認識できませんでした",
                    "history": history,
                    "last_distance_m": None,
                }

            turn_angle_deg = (
                float(far_ball["center_offset_ratio"])
                * red_ball_config.HORIZONTAL_FOV_DEG
            )
            turn_direction = "right" if turn_angle_deg >= 0.0 else "left"
            cycle_history["turn_direction"] = turn_direction
            cycle_history["detected_turn_angle_deg"] = turn_angle_deg
            print(
                "スクエアゾーン中心誘導: 遠方候補を選択 "
                f"direction={turn_direction}, "
                f"angle={turn_angle_deg:.2f}deg",
                flush=True,
            )
            alignment_result = align_red_ball_to_center(
                navigation_controller,
                driver,
                sensor_manager,
                target_hint_x=float(far_ball["x"]),
                target_hint_size_px=_candidate_visible_size(far_ball),
                distance_m=diagonal_min_distance_m,
            )
            cycle_history["alignment_result"] = alignment_result
            if not alignment_result["centered"]:
                return {
                    "center_reached": False,
                    "reason": alignment_result["reason"],
                    "history": history,
                    "last_distance_m": None,
                }

            selected_red_result = (
                alignment_result.get("last_red_result") or red_result
            )
            selected_ball = selected_red_result.get("selected_red_ball")
            distance_m = sensor_manager.get_distance_m()
            if distance_m is None:
                driver.stop()
                return {
                    "center_reached": False,
                    "reason": "距離を測定できませんでした",
                    "history": history,
                    "last_distance_m": None,
                }

            distance_m = float(distance_m)
            cycle_history["measured_distance_m"] = distance_m
            is_diagonal_ball = (
                diagonal_min_distance_m
                <= distance_m
                <= diagonal_max_distance_m
            )
            cycle_history["is_diagonal_ball"] = is_diagonal_ball
            approach_initial_distance_m = distance_m
            print(
                "スクエアゾーン中心誘導: 対角判定 "
                f"distance={distance_m:.3f}m, "
                f"range={diagonal_min_distance_m:.3f}-"
                f"{diagonal_max_distance_m:.3f}m, "
                f"is_diagonal={is_diagonal_ball}",
                flush=True,
            )

            if not is_diagonal_ball:
                opposite_turn_angle_deg = float(
                    red_ball_config.CENTER_OF_ZONE_OPPOSITE_TURN_ANGLE_DEG
                )
                opposite_angle_deg = (
                    -opposite_turn_angle_deg
                    if turn_direction == "right"
                    else opposite_turn_angle_deg
                )
                print(
                    "スクエアゾーン中心誘導: 非対角のため逆方向へ旋回 "
                    f"angle={opposite_angle_deg:.1f}deg",
                    flush=True,
                )
                opposite_turn_result = navigation_controller.rotate_by_angle(
                    driver,
                    sensor_manager,
                    opposite_angle_deg,
                    speed=red_ball_config.CENTERING_ROTATE_SPEED,
                    tolerance_deg=(
                        red_ball_config.CENTERING_ROTATE_TOLERANCE_DEG
                    ),
                    timeout_s=red_ball_config.ROTATE_TIMEOUT_S,
                )
                cycle_history["opposite_turn_result"] = opposite_turn_result
                if not opposite_turn_result["reached"]:
                    return {
                        "center_reached": False,
                        "reason": (
                            "逆方向への"
                            f"{opposite_turn_angle_deg:.1f}度旋回が"
                            "完了しませんでした"
                        ),
                        "history": history,
                        "last_distance_m": distance_m,
                    }

                frame = sensor_manager.capture_front_frame()
                selected_red_result = _detect_red_balls(processor, frame)
                selected_ball = _select_farthest_red_ball(
                    selected_red_result
                )
                if selected_ball is None:
                    fallback_angle_deg = -2.0 * opposite_angle_deg
                    print(
                        "スクエアゾーン中心誘導: 候補がないため"
                        "反対側を探索 "
                        f"angle={fallback_angle_deg:.1f}deg",
                        flush=True,
                    )
                    fallback_turn_result = (
                        navigation_controller.rotate_by_angle(
                            driver,
                            sensor_manager,
                            fallback_angle_deg,
                            speed=red_ball_config.CENTERING_ROTATE_SPEED,
                            tolerance_deg=(
                                red_ball_config.CENTERING_ROTATE_TOLERANCE_DEG
                            ),
                            timeout_s=red_ball_config.ROTATE_TIMEOUT_S,
                        )
                    )
                    cycle_history["fallback_turn_result"] = (
                        fallback_turn_result
                    )
                    if not fallback_turn_result["reached"]:
                        return {
                            "center_reached": False,
                            "reason": (
                                "反対側を探す"
                                f"{abs(fallback_angle_deg):.1f}度旋回が"
                                "完了しませんでした"
                            ),
                            "history": history,
                            "last_distance_m": distance_m,
                        }

                    frame = sensor_manager.capture_front_frame()
                    selected_red_result = _detect_red_balls(
                        processor, frame
                    )
                    selected_ball = _select_farthest_red_ball(
                        selected_red_result
                    )
                    if selected_ball is None:
                        return {
                            "center_reached": False,
                            "reason": (
                                "両方向で対角の赤ボールを"
                                "認識できませんでした"
                            ),
                            "history": history,
                            "last_distance_m": distance_m,
                        }
                approach_initial_distance_m = diagonal_min_distance_m

            approach_target_distance_m = (
                red_ball_config.CENTER_OF_ZONE_GOAL_DISTANCE_M
                if cycle_index == repeat_count
                else red_ball_config.FINAL_TARGET_DISTANCE_M
            )
            cycle_history["approach_target_distance_m"] = (
                approach_target_distance_m
            )
            cycle_history["approach_initial_centering_distance_m"] = (
                approach_initial_distance_m
            )
            print(
                "スクエアゾーン中心誘導: 対角ボールへの接近開始 "
                f"target={approach_target_distance_m:.3f}m, "
                f"initial_correction="
                f"{approach_initial_distance_m:.3f}m",
                flush=True,
            )
            approach_result = _approach_red_ball_to_distance(
                navigation_controller,
                driver,
                sensor_manager,
                approach_target_distance_m,
                target_hint_x=(
                    None
                    if selected_ball is None
                    else float(selected_ball["x"])
                ),
                target_hint_size_px=(
                    None
                    if selected_ball is None
                    else _candidate_visible_size(selected_ball)
                ),
                initial_centering_distance_m=approach_initial_distance_m,
                log_prefix="スクエアゾーン中心誘導",
            )
            cycle_history["approach_result"] = approach_result
            last_approach_result = approach_result
            if not approach_result["reached"]:
                return {
                    "center_reached": False,
                    "reason": approach_result["reason"],
                    "history": history,
                    "last_distance_m": approach_result["last_distance_m"],
                }

        return {
            "center_reached": True,
            "reason": (
                "対角ボールとの距離"
                f"{red_ball_config.CENTER_OF_ZONE_GOAL_DISTANCE_M:.3f}m"
                "まで誘導しました"
            ),
            "history": history,
            "last_distance_m": last_approach_result["last_distance_m"],
        }
    finally:
        driver.stop()
