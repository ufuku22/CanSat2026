import math
import time
from typing import Any, Optional

from config import (
    LidarForwardConfig as LidarConfig,
    RedBallConfig,
    RedConeConfig,
    SecondRedBallConfig,
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
            return frame, red_result, scan_history

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

    return None, None, scan_history


def _red_result_to_turn_angle(red_result: dict[str, Any], horizontal_fov_deg: float):
    """列ごとの赤色ピーク位置を水平FOV内の旋回角度に変換する。"""
    offset_ratio = red_result.get("color_peak_center_offset_ratio")
    if offset_ratio is not None:
        return float(offset_ratio) * float(horizontal_fov_deg)

    return 0.0


def _select_center_nearest_red_peak(red_result: dict[str, Any]):
    """検出された赤ピークのうち、画像中央に最も近いものを選ぶ。"""
    ball_candidates = [
        candidate
        for candidate in red_result.get("red_ball_candidates", [])
        if candidate.get("center_offset_ratio") is not None
    ]
    if ball_candidates:
        return min(
            ball_candidates,
            key=lambda candidate: abs(float(candidate["center_offset_ratio"])),
        )

    peaks = red_result.get("color_peak_columns", [])
    valid_peaks = [
        peak
        for peak in peaks
        if peak.get("center_offset_ratio") is not None
    ]
    if not valid_peaks:
        return None

    return min(
        valid_peaks,
        key=lambda peak: abs(float(peak["center_offset_ratio"])),
    )


def _select_largest_red_peak(red_result: dict[str, Any]):
    """検出された赤ピークのうち、列方向の赤割合が最も大きいものを選ぶ。"""
    ball_candidates = [
        candidate
        for candidate in red_result.get("red_ball_candidates", [])
        if (
            candidate.get("center_offset_ratio") is not None
            and candidate.get("score") is not None
        )
    ]
    if ball_candidates:
        return max(
            ball_candidates,
            key=lambda candidate: float(candidate["score"]),
        )

    peaks = red_result.get("color_peak_columns", [])
    valid_peaks = [
        peak
        for peak in peaks
        if (
            peak.get("center_offset_ratio") is not None
            and peak.get("column_ratio") is not None
        )
    ]
    if not valid_peaks:
        return None

    return max(
        valid_peaks,
        key=lambda peak: float(peak["column_ratio"]),
    )


def _candidate_visible_size(candidate: dict[str, Any]) -> float | None:
    if candidate.get("radius_px") is not None:
        return float(candidate["radius_px"]) * 2.0
    if candidate.get("visible_diameter_px") is not None:
        return float(candidate["visible_diameter_px"])
    if candidate.get("score") is not None:
        return math.sqrt(max(0.0, float(candidate["score"])))
    return None


def _select_nearest_red_peak(red_result: dict[str, Any]):
    """見かけサイズから、近そうな赤ボール候補を選ぶ。"""
    ball_candidates = []
    for candidate in red_result.get("red_ball_candidates", []):
        if candidate.get("center_offset_ratio") is None:
            continue
        visible_size = _candidate_visible_size(candidate)
        if visible_size is None:
            continue
        candidate = candidate.copy()
        candidate["nearest_visible_size_px"] = visible_size
        ball_candidates.append(candidate)

    if ball_candidates:
        return max(
            ball_candidates,
            key=lambda candidate: (
                float(candidate["nearest_visible_size_px"]),
                float(candidate.get("score", 0.0)),
                -abs(float(candidate["center_offset_ratio"])),
            ),
        )

    return _select_largest_red_peak(red_result)


def _candidate_delta_x(candidate: dict[str, Any], target_hint_x: float) -> float:
    return abs(float(candidate["x"]) - float(target_hint_x))


def _image_width_from_red_result(red_result: dict[str, Any]) -> float | None:
    width = red_result.get("image_width")
    if width is not None:
        return float(width)

    for peak in red_result.get("red_ball_candidates", []):
        offset_ratio = peak.get("center_offset_ratio")
        x = peak.get("x")
        if offset_ratio is None or x is None:
            continue
        denominator = float(offset_ratio) + 0.5
        if denominator > 0.0:
            return (float(x) + 0.5) / denominator

    for peak in red_result.get("color_peak_columns", []):
        offset_ratio = peak.get("center_offset_ratio")
        x = peak.get("x")
        if offset_ratio is None or x is None:
            continue
        denominator = float(offset_ratio) + 0.5
        if denominator > 0.0:
            return (float(x) + 0.5) / denominator

    return None


def _predict_target_hint_x_after_rotation(
    peak: dict[str, Any] | None,
    rotated_angle_deg: float,
    horizontal_fov_deg: float,
    image_width: float | None,
) -> float | None:
    """旋回後も同じ赤ボールを追うため、次フレームでの予想x座標を返す。"""
    if peak is None or peak.get("x") is None or image_width is None:
        return None

    image_width = float(image_width)
    horizontal_fov_deg = float(horizontal_fov_deg)
    if image_width <= 0.0 or horizontal_fov_deg <= 0.0:
        return None

    predicted_x = (
        float(peak["x"])
        - (float(rotated_angle_deg) / horizontal_fov_deg) * image_width
    )
    return max(0.0, min(image_width - 1.0, predicted_x))


def _select_red_peak_near_hint(
    red_result: dict[str, Any],
    target_hint_x: float,
    max_delta_px: float,
):
    """前回選んだx座標に近い赤ボール候補を選ぶ。"""
    ball_candidates = [
        candidate
        for candidate in red_result.get("red_ball_candidates", [])
        if candidate.get("x") is not None
    ]
    if ball_candidates:
        nearest = min(
            ball_candidates,
            key=lambda candidate: _candidate_delta_x(candidate, target_hint_x),
        )
        if _candidate_delta_x(nearest, target_hint_x) <= max_delta_px:
            return nearest

    peaks = [
        peak
        for peak in red_result.get("color_peak_columns", [])
        if peak.get("x") is not None
    ]
    if not peaks:
        return None

    nearest = min(peaks, key=lambda peak: _candidate_delta_x(peak, target_hint_x))
    if _candidate_delta_x(nearest, target_hint_x) <= max_delta_px:
        return nearest
    return None


def _select_red_peak(red_result: dict[str, Any], peak_priority: str):
    """指定された優先条件で中央合わせに使う赤ピークを選ぶ。"""
    if peak_priority == "nearest":
        return _select_nearest_red_peak(red_result)
    if peak_priority == "largest":
        return _select_largest_red_peak(red_result)
    if peak_priority == "center_nearest":
        return _select_center_nearest_red_peak(red_result)

    raise ValueError(
        "peak_priority must be 'nearest', 'largest' or 'center_nearest'"
    )


def _use_red_peak(red_result: dict[str, Any], peak: dict[str, Any] | None):
    """中央合わせで使う赤ピークを検出結果へ反映する。"""
    if peak is None:
        return red_result

    red_result["color_peak_column_x"] = float(peak["x"])
    red_result["color_peak_center_offset_ratio"] = float(
        peak["center_offset_ratio"]
    )
    red_result["selected_color_peak"] = peak
    return red_result


def _add_red_ball_candidates(
    processor: ImageProcessor,
    frame: Any,
    red_result: dict[str, Any],
) -> dict[str, Any]:
    """赤色検出結果へ赤ボール候補を追加する。"""
    circle_candidates = processor.detect_red_ball_circle_candidates(frame)
    ball_color_result = processor.detect_color(
        frame,
        hsv_ranges=processor.RED_BALL_HSV_RANGES,
        color_threshold=0.0,
        column_threshold=0.005,
        column_average_width=31,
    )
    size_candidates = processor.detect_red_ball_candidates(
        frame,
        color_result=ball_color_result,
    )
    red_result["red_ball_circle_candidates"] = circle_candidates
    red_result["red_ball_size_candidates"] = size_candidates
    red_result["red_ball_candidates"] = circle_candidates or size_candidates
    red_result["red_ball_candidate_count"] = len(
        red_result["red_ball_candidates"]
    )
    if red_result["red_ball_candidates"]:
        red_result["is_color_detected"] = True
    return red_result


def _turn_toward_red(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    red_result: dict[str, Any],
    horizontal_fov_deg: float,
    **rotate_kwargs,
):
    """赤色の画面内位置から旋回角度を決め、必要な場合だけ旋回する。"""
    turn_angle = _red_result_to_turn_angle(red_result, horizontal_fov_deg)
    if turn_angle == 0.0:
        return turn_angle, None

    turn_result = navigation_controller.rotate_by_angle(
        driver, sensor_manager, turn_angle, **rotate_kwargs
    )
    return turn_angle, turn_result


def align_red__peak_to_center(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    *,
    peak_priority: str = "center_nearest",
    target_hint_x: float | None = None,
) -> dict[str, Any]:
    """赤検知率ピークの列が中央に来るまで撮影と旋回を繰り返す。"""
    processor = ImageProcessor()
    red_ball_config = RedBallConfig()
    local_target_hint_x = target_hint_x

    for step in range(red_ball_config.MAX_CENTERING_STEPS):
        print(
            "赤ボール中央合わせ: "
            f"step {step + 1}/{red_ball_config.MAX_CENTERING_STEPS} 撮影します"
        )
        frame = sensor_manager.capture_front_frame()
        red_result = _without_color_mask(
            processor.detect_color(
                frame,
                hsv_ranges=processor.RED_HSV_RANGES,
                color_threshold=red_ball_config.SWITCH_RED_RATIO,
                column_threshold=red_ball_config.RED_COLUMN_THRESHOLD,
                column_average_width=red_ball_config.RED_COLUMN_AVERAGE_WIDTH,
            )
        )
        red_result = _add_red_ball_candidates(processor, frame, red_result)
        selected_peak = None
        selected_by_hint = False
        if local_target_hint_x is not None:
            selected_peak = _select_red_peak_near_hint(
                red_result,
                local_target_hint_x,
                red_ball_config.CENTERING_TARGET_LOCK_MAX_DELTA_PX,
            )
            selected_by_hint = selected_peak is not None
        if selected_peak is None:
            selected_peak = _select_red_peak(red_result, peak_priority)
        red_result = _use_red_peak(red_result, selected_peak)
        red_result["selected_by_target_hint"] = selected_by_hint
        red_result["target_hint_x"] = local_target_hint_x
        if selected_peak is not None and selected_peak.get("x") is not None:
            local_target_hint_x = float(selected_peak["x"])
            red_result["updated_target_hint_x"] = local_target_hint_x

        turn_angle = _red_result_to_turn_angle(
            red_result,
            red_ball_config.HORIZONTAL_FOV_DEG,
        )

        if red_result["color_peak_column_x"] is None:
            reason = "赤を検知できませんでした"
            if red_result["is_color_detected"]:
                reason = "赤検知率ピークの列を判定できませんでした"
            print(f"赤ボール中央合わせ: {reason}")
            return {
                "centered": False,
                "red_detected": bool(red_result["is_color_detected"]),
                "reason": reason,
                "steps": step + 1,
                "last_red_result": red_result,
            }

        turn_gain = red_ball_config.CENTERING_TURN_GAIN
        if abs(turn_angle) >= red_ball_config.CENTERING_FULL_GAIN_ANGLE_DEG:
            turn_gain = 1.0
        rotate_angle = turn_angle * turn_gain

        print(
            "赤ボール中央合わせ: "
            f"total={red_result['total_color_ratio'] * 100:.2f}% "
            f"column={red_result['color_peak_column_x']} "
            f"turn={turn_angle:.2f}deg "
            f"gain={turn_gain:.2f} "
            f"locked={selected_by_hint} "
            f"rotate={rotate_angle:.2f}deg"
        )

        if abs(turn_angle) <= red_ball_config.CENTERING_TOLERANCE_DEG:
            return {
                "centered": True,
                "red_detected": True,
                "reason": "赤検知率ピークの列が中央付近に入りました",
                "steps": step + 1,
                "last_red_result": red_result,
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
            selected_peak,
            rotate_result.get("rotated_angle_deg", rotate_angle),
            red_ball_config.HORIZONTAL_FOV_DEG,
            _image_width_from_red_result(red_result),
        )
        if predicted_hint_x is not None:
            local_target_hint_x = predicted_hint_x

    return {
        "centered": False,
        "red_detected": True,
        "reason": "最大試行回数内に中央合わせできませんでした",
        "steps": red_ball_config.MAX_CENTERING_STEPS,
        "last_red_result": red_result if "red_result" in locals() else None,
    }


def _red_cone_forward_duration(red_ratio, default_duration_s, duration_table):
    """赤色の大きさに応じて前進時間を選ぶ。"""
    red_ratio = float(red_ratio)
    for threshold, duration_s in duration_table:
        if red_ratio > threshold:
            return duration_s
    return default_duration_s


def _red_ball_forward_duration(distance_m, default_duration_s, duration_table):
    """距離が遠いほど長く、近いほど短い前進時間を選ぶ。"""
    distance_m = float(distance_m)
    for threshold_m, duration_s in duration_table:
        if distance_m > threshold_m:
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


def _select_adjacent_red_peak(
    red_result: dict[str, Any],
    horizontal_fov_deg: float,
    center_exclusion_deg: float,
):
    """中央の赤ピークを除き、画面中心に最も近い隣ピークを選ぶ。"""
    adjacent_peaks = []
    source_peaks = red_result.get("red_ball_candidates") or red_result.get(
        "color_peak_columns",
        [],
    )
    for peak in source_peaks:
        offset_ratio = peak.get("center_offset_ratio")
        if offset_ratio is None:
            continue

        angle_deg = float(offset_ratio) * float(horizontal_fov_deg)
        if abs(angle_deg) <= center_exclusion_deg:
            continue

        adjacent_peaks.append((abs(angle_deg), angle_deg, peak))

    if not adjacent_peaks:
        return None, None

    _, angle_deg, peak = min(adjacent_peaks, key=lambda item: item[0])
    return peak, angle_deg


def _select_square_gate_candidates(
    red_result: dict[str, Any],
    horizontal_fov_deg: float,
    center_exclusion_deg: float,
) -> list[dict[str, Any]]:
    """Aを除き、A基準画像内でB/C候補になる赤ピークを近い角度順に返す。"""
    candidates = []
    source_peaks = red_result.get("red_ball_candidates") or red_result.get(
        "color_peak_columns",
        [],
    )
    for peak in source_peaks:
        offset_ratio = peak.get("center_offset_ratio")
        if offset_ratio is None:
            continue
        angle_deg = float(offset_ratio) * float(horizontal_fov_deg)
        if abs(angle_deg) <= center_exclusion_deg:
            continue

        candidate = peak.copy()
        candidate["angle_deg"] = angle_deg
        candidates.append(candidate)

    return sorted(candidates, key=lambda peak: abs(float(peak["angle_deg"])))


def _heading_to_xy(heading_deg: float, distance_m: float) -> tuple[float, float]:
    """方位角と距離を、北基準・時計回りのXY座標へ変換する。"""
    heading_rad = math.radians(float(heading_deg))
    distance_m = float(distance_m)
    return (
        distance_m * math.sin(heading_rad),
        distance_m * math.cos(heading_rad),
    )


def _xy_to_heading_deg(x_m: float, y_m: float) -> float:
    return (math.degrees(math.atan2(float(x_m), float(y_m))) + 360.0) % 360.0


def _distance_xy(a_xy: tuple[float, float], b_xy: tuple[float, float]) -> float:
    return math.hypot(a_xy[0] - b_xy[0], a_xy[1] - b_xy[1])


def _classify_square_ball_distance(
    distance_m: float,
    red_ball_config: RedBallConfig,
) -> str:
    """A-B間距離から隣接球・対角球・不明を判定する。"""
    side_error = abs(float(distance_m) - red_ball_config.SQUARE_SIDE_M)
    diagonal_error = abs(float(distance_m) - red_ball_config.SQUARE_DIAGONAL_M)
    tolerance_m = float(red_ball_config.SQUARE_GATE_CLASSIFICATION_TOLERANCE_M)

    if side_error <= diagonal_error and side_error <= tolerance_m:
        return "adjacent"
    if diagonal_error < side_error and diagonal_error <= tolerance_m:
        return "diagonal"
    return "unknown"


def _calculate_square_gate_geometry(
    a_heading_deg: float,
    a_surface_distance_m: float,
    b_heading_deg: float,
    b_surface_distance_m: float,
    red_ball_config: RedBallConfig,
) -> dict[str, Any] | None:
    """A/B測定値からQ、QB、ゴール中心方向を計算する。"""
    ball_radius_m = float(red_ball_config.BALL_RADIUS_M)
    lidar_forward_offset_m = float(red_ball_config.LIDAR_FORWARD_OFFSET_M)
    a_center_distance_m = (
        float(a_surface_distance_m)
        + lidar_forward_offset_m
        + ball_radius_m
    )
    b_center_distance_m = (
        float(b_surface_distance_m)
        + lidar_forward_offset_m
        + ball_radius_m
    )
    a_xy = _heading_to_xy(a_heading_deg, a_center_distance_m)
    b_xy = _heading_to_xy(b_heading_deg, b_center_distance_m)
    gate_dx = b_xy[0] - a_xy[0]
    gate_dy = b_xy[1] - a_xy[1]
    gate_length_m = math.hypot(gate_dx, gate_dy)
    if gate_length_m <= 0.0:
        return None

    midpoint_xy = (
        (a_xy[0] + b_xy[0]) / 2.0,
        (a_xy[1] + b_xy[1]) / 2.0,
    )
    denominator = b_xy[0] * gate_dx + b_xy[1] * gate_dy
    if abs(denominator) < 1e-6:
        return None

    q_scale = (midpoint_xy[0] * gate_dx + midpoint_xy[1] * gate_dy) / denominator
    q_xy = (b_xy[0] * q_scale, b_xy[1] * q_scale)
    qb_center_distance_m = _distance_xy(q_xy, b_xy)
    qb_lidar_distance_m = max(
        0.0,
        qb_center_distance_m - ball_radius_m - lidar_forward_offset_m,
    )

    normal_x = -gate_dy / gate_length_m
    normal_y = gate_dx / gate_length_m
    half_side_m = gate_length_m / 2.0
    center_candidates = (
        (
            midpoint_xy[0] + normal_x * half_side_m,
            midpoint_xy[1] + normal_y * half_side_m,
        ),
        (
            midpoint_xy[0] - normal_x * half_side_m,
            midpoint_xy[1] - normal_y * half_side_m,
        ),
    )
    q_from_mid = (q_xy[0] - midpoint_xy[0], q_xy[1] - midpoint_xy[1])

    def opposite_score(center_xy: tuple[float, float]) -> float:
        center_from_mid = (
            center_xy[0] - midpoint_xy[0],
            center_xy[1] - midpoint_xy[1],
        )
        return center_from_mid[0] * q_from_mid[0] + center_from_mid[1] * q_from_mid[1]

    square_center_xy = min(center_candidates, key=opposite_score)
    center_from_q = (
        square_center_xy[0] - q_xy[0],
        square_center_xy[1] - q_xy[1],
    )
    center_heading_deg = _xy_to_heading_deg(center_from_q[0], center_from_q[1])

    return {
        "a_xy": a_xy,
        "b_xy": b_xy,
        "midpoint_xy": midpoint_xy,
        "q_xy": q_xy,
        "q_scale": q_scale,
        "gate_length_m": gate_length_m,
        "qb_center_distance_m": qb_center_distance_m,
        "qb_lidar_distance_m": qb_lidar_distance_m,
        "lidar_forward_offset_m": lidar_forward_offset_m,
        "square_center_xy": square_center_xy,
        "center_heading_deg": center_heading_deg,
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
        _found_frame, red_result, scan_history = (
            _find_red_cone_in_view(
                navigation_controller,
                driver,
                sensor_manager,
                processor,
                red_cone_config,
            )
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
        turn_angle, turn_result = _turn_toward_red(
            navigation_controller,
            driver,
            sensor_manager,
            red_result,
            red_cone_config.HORIZONTAL_FOV_DEG,
            speed=red_cone_config.ROTATE_SPEED,
            tolerance_deg=red_cone_config.ROTATE_TOLERANCE_DEG,
            timeout_s=red_cone_config.ROTATE_TIMEOUT_S,
        )

        # 3. 赤色が大きく見えているほど近いとみなし、前進時間を短くする。
        forward_duration = _red_cone_forward_duration(
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


def _approach_first_red_ball(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
) -> dict[str, Any]:
    """最初の赤ボールへ誘導し、距離センサで目標距離付近まで近づく。"""
    red_ball_config = RedBallConfig()
    red_cone_config = RedConeConfig()

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
        }

    duration_by_distance = tuple(
        sorted(
            red_ball_config.FORWARD_DURATION_BY_DISTANCE_M,
            reverse=True,
        )
    )
    target_hint_x = None

    for step in range(red_ball_config.MAX_DISTANCE_APPROACH_STEPS):
        center_result = align_red__peak_to_center(
            navigation_controller,
            driver,
            sensor_manager,
            peak_priority="nearest",
            target_hint_x=target_hint_x,
        )
        if not center_result["centered"]:
            return {
                "target_reached": False,
                "reason": center_result["reason"],
                "cone_result": cone_result,
                "centering_result": center_result,
                "steps": step + 1,
                "last_distance_m": None,
            }

        selected_peak = (
            center_result.get("last_red_result") or {}
        ).get("selected_color_peak")
        if selected_peak is not None and selected_peak.get("x") is not None:
            target_hint_x = float(selected_peak["x"])

        distance_m = sensor_manager.get_distance_m()
        if distance_m is None:
            driver.stop()
            return {
                "target_reached": False,
                "reason": "距離を測定できませんでした",
                "cone_result": cone_result,
                "centering_result": center_result,
                "steps": step + 1,
                "last_distance_m": None,
            }

        distance_m = float(distance_m)
        target_distance_m = float(red_ball_config.APPROACH_TARGET_DISTANCE_M)
        tolerance_m = float(red_ball_config.APPROACH_DISTANCE_TOLERANCE_M)
        too_close_distance_m = target_distance_m - tolerance_m
        stop_distance_m = target_distance_m + tolerance_m
        print(
            "赤ボール誘導: "
            f"distance={distance_m:.3f}m, "
            f"target={target_distance_m:.3f}m"
        )
        if distance_m < too_close_distance_m:
            print(
                "赤ボール誘導: "
                f"近すぎるため{red_ball_config.APPROACH_REVERSE_DURATION_S:.2f}秒"
                "後退します"
            )
            _reverse_for_duration(
                driver,
                red_ball_config.APPROACH_REVERSE_SPEED,
                red_ball_config.APPROACH_REVERSE_DURATION_S,
            )
            continue

        if distance_m <= stop_distance_m:
            driver.stop()
            return {
                "target_reached": True,
                "reason": "目標距離範囲に入りました",
                "cone_result": cone_result,
                "centering_result": center_result,
                "steps": step + 1,
                "last_distance_m": distance_m,
                "target_distance_m": target_distance_m,
                "target_tolerance_m": tolerance_m,
            }

        forward_duration = _red_ball_forward_duration(
            distance_m,
            red_ball_config.FORWARD_DURATION_S,
            duration_by_distance,
        )
        print(f"赤ボール誘導: 前進 {forward_duration:.2f}秒")
        navigation_controller.follow_forward(
            driver,
            sensor_manager,
            forward_duration,
            base_speed=red_cone_config.FORWARD_SPEED,
            loop_interval=red_cone_config.LOOP_INTERVAL_S,
        )

    return {
        "target_reached": False,
        "reason": "最大試行回数内に目標距離まで近づけませんでした",
        "cone_result": cone_result,
        "steps": red_ball_config.MAX_DISTANCE_APPROACH_STEPS,
        "last_distance_m": distance_m if "distance_m" in locals() else None,
    }


def guide_to_red_ball(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
) -> dict[str, Any]:
    """赤ボールへ誘導し、距離センサで目標距離付近まで近づく。"""
    return _approach_first_red_ball(
        navigation_controller,
        driver,
        sensor_manager,
    )


def guide_to_square_zone_legacy(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    *,
    image_processor: ImageProcessor | None = None,
    first_ball_result: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """中央の赤ボール到達後、前の実装どおり隣の赤ボールへ順に近づく。"""
    processor = image_processor or ImageProcessor()
    red_ball_config = RedBallConfig()
    red_cone_config = RedConeConfig()
    duration_by_distance = tuple(
        sorted(
            red_ball_config.FORWARD_DURATION_BY_DISTANCE_M,
            reverse=True,
        )
    )

    history: list[dict[str, Any]] = []
    last_distance_m = None

    try:
        for target_index in range(1, red_ball_config.SQUARE_ZONE_MAX_TARGETS + 1):
            print(
                "スクエアゾーン誘導(旧): "
                f"target {target_index}/"
                f"{red_ball_config.SQUARE_ZONE_MAX_TARGETS} 撮影します"
            )
            frame = sensor_manager.capture_front_frame()
            red_result = _without_color_mask(
                processor.detect_color(
                    frame,
                    hsv_ranges=processor.RED_HSV_RANGES,
                    color_threshold=red_cone_config.RED_THRESHOLD,
                    column_threshold=red_ball_config.RED_COLUMN_THRESHOLD,
                    column_average_width=(
                        red_ball_config.RED_COLUMN_AVERAGE_WIDTH
                    ),
                )
            )
            red_result = _add_red_ball_candidates(processor, frame, red_result)
            peak_count = int(red_result.get("color_peak_count", 0))
            visible_target_count = int(
                red_result.get("red_ball_candidate_count") or peak_count
            )
            adjacent_peak, turn_angle = _select_adjacent_red_peak(
                red_result,
                red_ball_config.HORIZONTAL_FOV_DEG,
                red_ball_config.CENTERING_TOLERANCE_DEG,
            )
            target_history: dict[str, Any] = {
                "target_index": target_index,
                "red_result": red_result,
                "adjacent_peak": adjacent_peak,
                "turn_angle_deg": turn_angle,
                "rotate_result": None,
                "approach_history": [],
            }
            history.append(target_history)

            print(
                "スクエアゾーン誘導(旧): "
                f"peak_count={peak_count}, "
                f"candidate_count={visible_target_count}, "
                f"adjacent_peak={None if adjacent_peak is None else adjacent_peak['x']}"
            )

            if visible_target_count < 2 or adjacent_peak is None:
                return {
                    "square_zone_reached": True,
                    "reason": "画面内に隣の赤ボールが見つからないため終了します",
                    "approached_balls": target_index - 1,
                    "last_distance_m": last_distance_m,
                    "last_red_result": red_result,
                    "history": history,
                }

            print(
                "スクエアゾーン誘導(旧): "
                f"隣ピークへ{turn_angle:.2f}度旋回します"
            )
            rotate_result = navigation_controller.rotate_by_angle(
                driver,
                sensor_manager,
                turn_angle,
                speed=red_ball_config.CENTERING_ROTATE_SPEED,
                tolerance_deg=red_ball_config.CENTERING_ROTATE_TOLERANCE_DEG,
                timeout_s=red_ball_config.ROTATE_TIMEOUT_S,
            )
            target_history["rotate_result"] = rotate_result
            if not rotate_result["reached"]:
                return {
                    "square_zone_reached": False,
                    "reason": "隣ピークへの旋回が完了しませんでした",
                    "approached_balls": target_index - 1,
                    "last_distance_m": last_distance_m,
                    "last_red_result": red_result,
                    "history": history,
                }

            target_hint_x = _predict_target_hint_x_after_rotation(
                adjacent_peak,
                rotate_result.get("rotated_angle_deg", turn_angle),
                red_ball_config.HORIZONTAL_FOV_DEG,
                _image_width_from_red_result(red_result),
            )
            print(
                "スクエアゾーン誘導(旧): "
                f"{red_ball_config.SQUARE_ZONE_TARGET_DISTANCE_M:.3f}mまで"
                "中央合わせしながら前進します"
            )
            for approach_step in range(
                1, red_ball_config.MAX_DISTANCE_APPROACH_STEPS + 1
            ):
                approach_record = {
                    "approach_step": approach_step,
                    "centering_result": None,
                    "distance_m": None,
                    "forward_duration_s": None,
                }
                target_history["approach_history"].append(approach_record)

                center_result = align_red__peak_to_center(
                    navigation_controller,
                    driver,
                    sensor_manager,
                    target_hint_x=target_hint_x,
                )
                approach_record["centering_result"] = center_result
                if not center_result["centered"]:
                    return {
                        "square_zone_reached": False,
                        "reason": center_result["reason"],
                        "approached_balls": target_index - 1,
                        "last_distance_m": last_distance_m,
                        "last_red_result": red_result,
                        "history": history,
                    }

                selected_peak = (
                    center_result.get("last_red_result") or {}
                ).get("selected_color_peak")
                if selected_peak is not None and selected_peak.get("x") is not None:
                    target_hint_x = float(selected_peak["x"])

                distance_m = sensor_manager.get_distance_m()
                if distance_m is None:
                    driver.stop()
                    return {
                        "square_zone_reached": False,
                        "reason": "距離を測定できませんでした",
                        "approached_balls": target_index - 1,
                        "last_distance_m": None,
                        "last_red_result": red_result,
                        "history": history,
                    }

                distance_m = float(distance_m)
                last_distance_m = distance_m
                approach_record["distance_m"] = distance_m
                print(
                    "スクエアゾーン誘導(旧): "
                    f"distance={distance_m:.3f}m"
                )
                if distance_m < red_ball_config.SQUARE_ZONE_TARGET_DISTANCE_M:
                    driver.stop()
                    break

                forward_duration = _red_ball_forward_duration(
                    distance_m,
                    red_ball_config.FORWARD_DURATION_S,
                    duration_by_distance,
                )
                approach_record["forward_duration_s"] = forward_duration
                print(
                    "スクエアゾーン誘導(旧): "
                    f"前進 {forward_duration:.2f}秒"
                )
                navigation_controller.follow_forward(
                    driver,
                    sensor_manager,
                    forward_duration,
                    base_speed=red_cone_config.FORWARD_SPEED,
                    loop_interval=red_cone_config.LOOP_INTERVAL_S,
                )
            else:
                return {
                    "square_zone_reached": False,
                    "reason": "最大試行回数内に目標距離まで近づけませんでした",
                    "approached_balls": target_index - 1,
                    "last_distance_m": last_distance_m,
                    "last_red_result": red_result,
                    "history": history,
                }
    finally:
        driver.stop()

    return {
        "square_zone_reached": False,
        "reason": "最大対象数まで誘導しても終了条件に到達しませんでした",
        "approached_balls": red_ball_config.SQUARE_ZONE_MAX_TARGETS,
        "last_distance_m": last_distance_m,
        "last_red_result": history[-1]["red_result"] if history else None,
        "history": history,
    }


def guide_to_square_zone(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    *,
    image_processor: ImageProcessor | None = None,
    first_ball_result: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """A/Bの幾何から入口ゲートを決め、正方形領域へ進入する。"""
    processor = image_processor or ImageProcessor()
    red_ball_config = RedBallConfig()
    red_cone_config = RedConeConfig()

    history: list[dict[str, Any]] = []
    last_distance_m = None
    last_red_result = None

    try:
        print("スクエアゾーン誘導: Aを画像中央へ合わせます")
        first_ball_peak = (
            ((first_ball_result or {}).get("centering_result") or {})
            .get("last_red_result") or {}
        ).get("selected_color_peak")
        first_ball_hint_x = None
        if first_ball_peak is not None and first_ball_peak.get("x") is not None:
            first_ball_hint_x = float(first_ball_peak["x"])
        center_a_result = align_red__peak_to_center(
            navigation_controller,
            driver,
            sensor_manager,
            peak_priority="nearest",
            target_hint_x=first_ball_hint_x,
        )
        if not center_a_result["centered"]:
            return {
                "square_zone_reached": False,
                "reason": center_a_result["reason"],
                "approached_balls": 0,
                "last_distance_m": None,
                "last_red_result": center_a_result.get("last_red_result"),
                "history": history,
            }

        a_surface_distance_m = sensor_manager.get_distance_m()
        if a_surface_distance_m is None:
            driver.stop()
            return {
                "square_zone_reached": False,
                "reason": "Aまでの距離を測定できませんでした",
                "approached_balls": 0,
                "last_distance_m": None,
                "last_red_result": center_a_result.get("last_red_result"),
                "history": history,
            }

        a_surface_distance_m = float(a_surface_distance_m)
        a_heading_deg = float(sensor_manager.get_heading_deg())
        last_distance_m = a_surface_distance_m
        last_red_result = center_a_result.get("last_red_result")
        if last_red_result is None:
            frame = sensor_manager.capture_front_frame()
            last_red_result = _without_color_mask(
                processor.detect_color(
                    frame,
                    hsv_ranges=processor.RED_HSV_RANGES,
                    color_threshold=red_cone_config.RED_THRESHOLD,
                    column_threshold=red_ball_config.RED_COLUMN_THRESHOLD,
                    column_average_width=(
                        red_ball_config.RED_COLUMN_AVERAGE_WIDTH
                    ),
                )
            )
            last_red_result = _add_red_ball_candidates(
                processor,
                frame,
                last_red_result,
            )
        candidates = _select_square_gate_candidates(
            last_red_result,
            red_ball_config.HORIZONTAL_FOV_DEG,
            red_ball_config.CENTERING_TOLERANCE_DEG,
        )

        print(
            "スクエアゾーン誘導: "
            f"A距離={a_surface_distance_m:.3f}m, "
            f"A方位={a_heading_deg:.1f}deg, "
            f"B候補={len(candidates)}個"
        )
        if not candidates:
            return {
                "square_zone_reached": False,
                "reason": "A以外の赤ボール候補を同じ画像内で検出できませんでした",
                "approached_balls": 0,
                "last_distance_m": last_distance_m,
                "last_red_result": last_red_result,
                "history": history,
            }

        gate_record = None
        for candidate_index, candidate in enumerate(candidates, start=1):
            candidate_heading_deg = (
                a_heading_deg + float(candidate["angle_deg"])
            ) % 360.0
            current_heading_deg = float(sensor_manager.get_heading_deg())
            rotate_angle_deg = NavigationController.heading_error(
                candidate_heading_deg,
                current_heading_deg,
            )
            candidate_record: dict[str, Any] = {
                "candidate_index": candidate_index,
                "candidate_peak": candidate,
                "candidate_heading_deg": candidate_heading_deg,
                "initial_rotate_angle_deg": rotate_angle_deg,
                "initial_rotate_result": None,
                "centering_result": None,
                "b_surface_distance_m": None,
                "b_heading_deg": None,
                "ab_distance_m": None,
                "classification": None,
                "geometry": None,
            }
            history.append(candidate_record)

            print(
                "スクエアゾーン誘導: "
                f"B候補{candidate_index}/{len(candidates)}へ"
                f"{rotate_angle_deg:.2f}度旋回します"
            )
            rotate_result = navigation_controller.rotate_by_angle(
                driver,
                sensor_manager,
                rotate_angle_deg,
                speed=red_ball_config.CENTERING_ROTATE_SPEED,
                tolerance_deg=red_ball_config.CENTERING_ROTATE_TOLERANCE_DEG,
                timeout_s=red_ball_config.ROTATE_TIMEOUT_S,
            )
            candidate_record["initial_rotate_result"] = rotate_result
            if not rotate_result["reached"]:
                return {
                    "square_zone_reached": False,
                    "reason": "B候補への旋回が完了しませんでした",
                    "approached_balls": 0,
                    "last_distance_m": last_distance_m,
                    "last_red_result": last_red_result,
                    "history": history,
                }

            target_hint_x = _predict_target_hint_x_after_rotation(
                candidate,
                rotate_result.get("rotated_angle_deg", rotate_angle_deg),
                red_ball_config.HORIZONTAL_FOV_DEG,
                _image_width_from_red_result(last_red_result),
            )
            center_b_result = align_red__peak_to_center(
                navigation_controller,
                driver,
                sensor_manager,
                peak_priority="center_nearest",
                target_hint_x=target_hint_x,
            )
            candidate_record["centering_result"] = center_b_result
            last_red_result = center_b_result.get("last_red_result")
            if not center_b_result["centered"]:
                return {
                    "square_zone_reached": False,
                    "reason": center_b_result["reason"],
                    "approached_balls": 0,
                    "last_distance_m": last_distance_m,
                    "last_red_result": last_red_result,
                    "history": history,
                }

            b_surface_distance_m = sensor_manager.get_distance_m()
            if b_surface_distance_m is None:
                driver.stop()
                return {
                    "square_zone_reached": False,
                    "reason": "B候補までの距離を測定できませんでした",
                    "approached_balls": 0,
                    "last_distance_m": None,
                    "last_red_result": last_red_result,
                    "history": history,
                }

            b_surface_distance_m = float(b_surface_distance_m)
            b_heading_deg = float(sensor_manager.get_heading_deg())
            last_distance_m = b_surface_distance_m
            geometry = _calculate_square_gate_geometry(
                a_heading_deg,
                a_surface_distance_m,
                b_heading_deg,
                b_surface_distance_m,
                red_ball_config,
            )
            if geometry is None:
                candidate_record["classification"] = "unknown"
                print("スクエアゾーン誘導: 幾何計算に失敗したため次候補へ進みます")
                continue
            if not 0.0 < float(geometry["q_scale"]) < 1.0:
                candidate_record["classification"] = "unknown"
                candidate_record["geometry"] = geometry
                print("スクエアゾーン誘導: QがB方向の前進線上にないため次候補へ進みます")
                continue

            ab_distance_m = float(geometry["gate_length_m"])
            classification = _classify_square_ball_distance(
                ab_distance_m,
                red_ball_config,
            )
            candidate_record["b_surface_distance_m"] = b_surface_distance_m
            candidate_record["b_heading_deg"] = b_heading_deg
            candidate_record["ab_distance_m"] = ab_distance_m
            candidate_record["classification"] = classification
            candidate_record["geometry"] = geometry

            print(
                "スクエアゾーン誘導: "
                f"AB距離={ab_distance_m:.3f}m, 判定={classification}"
            )
            if classification == "adjacent":
                gate_record = candidate_record
                break
            if classification == "diagonal":
                print("スクエアゾーン誘導: 対角球のため次候補へ進みます")
                continue

            print("スクエアゾーン誘導: 隣接/対角を判定できないため次候補へ進みます")

        if gate_record is None:
            return {
                "square_zone_reached": False,
                "reason": "隣接球として使えるB候補を検出できませんでした",
                "approached_balls": 0,
                "last_distance_m": last_distance_m,
                "last_red_result": last_red_result,
                "history": history,
            }

        geometry = gate_record["geometry"]
        target_qb_lidar_m = float(geometry["qb_lidar_distance_m"])
        tolerance_m = float(red_ball_config.SQUARE_GATE_DISTANCE_TOLERANCE_M)
        gate_record["q_advance_history"] = []
        print(
            "スクエアゾーン誘導: "
            f"Q到達目標 LiDAR={target_qb_lidar_m:.3f}m "
            f"(許容±{tolerance_m:.3f}m)"
        )

        for advance_step in range(
            1,
            red_ball_config.SQUARE_GATE_MAX_ADVANCE_STEPS + 1,
        ):
            distance_m = sensor_manager.get_distance_m()
            if distance_m is None:
                driver.stop()
                return {
                    "square_zone_reached": False,
                    "reason": "Qへの微前進中に距離を測定できませんでした",
                    "approached_balls": 1,
                    "last_distance_m": None,
                    "last_red_result": last_red_result,
                    "history": history,
                }

            distance_m = float(distance_m)
            last_distance_m = distance_m
            advance_record = {
                "step": advance_step,
                "distance_m": distance_m,
                "target_qb_lidar_m": target_qb_lidar_m,
                "forward_duration_s": None,
                "reverse_duration_s": None,
            }
            gate_record["q_advance_history"].append(advance_record)
            print(
                "スクエアゾーン誘導: "
                f"Q微前進 step {advance_step}, "
                f"LiDAR={distance_m:.3f}m"
            )
            if abs(distance_m - target_qb_lidar_m) <= tolerance_m:
                driver.stop()
                break

            if distance_m < target_qb_lidar_m - tolerance_m:
                print(
                    "スクエアゾーン誘導: "
                    f"Qを行き過ぎたため"
                    f"{red_ball_config.SQUARE_GATE_REVERSE_DURATION_S:.2f}秒"
                    "後退します"
                )
                advance_record["reverse_duration_s"] = (
                    red_ball_config.SQUARE_GATE_REVERSE_DURATION_S
                )
                _reverse_for_duration(
                    driver,
                    red_ball_config.SQUARE_GATE_REVERSE_SPEED,
                    red_ball_config.SQUARE_GATE_REVERSE_DURATION_S,
                )
                continue

            advance_record["forward_duration_s"] = (
                red_ball_config.SQUARE_GATE_ADVANCE_DURATION_S
            )
            navigation_controller.follow_forward(
                driver,
                sensor_manager,
                red_ball_config.SQUARE_GATE_ADVANCE_DURATION_S,
                base_speed=red_cone_config.FORWARD_SPEED,
                loop_interval=red_cone_config.LOOP_INTERVAL_S,
            )
        else:
            return {
                "square_zone_reached": False,
                "reason": "最大試行回数内にQ付近まで前進できませんでした",
                "approached_balls": 1,
                "last_distance_m": last_distance_m,
                "last_red_result": last_red_result,
                "history": history,
            }

        current_heading_deg = float(sensor_manager.get_heading_deg())
        center_heading_deg = float(geometry["center_heading_deg"])
        center_rotate_angle_deg = NavigationController.heading_error(
            center_heading_deg,
            current_heading_deg,
        )
        print(
            "スクエアゾーン誘導: "
            f"ゴール中央方向へ{center_rotate_angle_deg:.2f}度旋回します"
        )
        center_rotate_result = navigation_controller.rotate_by_angle(
            driver,
            sensor_manager,
            center_rotate_angle_deg,
            speed=red_ball_config.CENTERING_ROTATE_SPEED,
            tolerance_deg=red_ball_config.CENTERING_ROTATE_TOLERANCE_DEG,
            timeout_s=red_ball_config.ROTATE_TIMEOUT_S,
        )
        gate_record["center_rotate_result"] = center_rotate_result
        if not center_rotate_result["reached"]:
            return {
                "square_zone_reached": False,
                "reason": "ゴール中央方向への旋回が完了しませんでした",
                "approached_balls": 1,
                "last_distance_m": last_distance_m,
                "last_red_result": last_red_result,
                "history": history,
            }

        print(
            "スクエアゾーン誘導: "
            f"{red_ball_config.SQUARE_ZONE_ENTRY_FORWARD_DURATION_S:.2f}秒直進します"
        )
        navigation_controller.follow_forward(
            driver,
            sensor_manager,
            red_ball_config.SQUARE_ZONE_ENTRY_FORWARD_DURATION_S,
            base_speed=red_cone_config.FORWARD_SPEED,
            loop_interval=red_cone_config.LOOP_INTERVAL_S,
        )
    finally:
        driver.stop()

    return {
        "square_zone_reached": True,
        "reason": "入口ゲートから正方形領域へ進入しました",
        "approached_balls": 1,
        "last_distance_m": last_distance_m,
        "last_red_result": last_red_result,
        "history": history,
    }


def search_second_red_ball_and_advance(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    *,
    image_processor: ImageProcessor | None = None,
    distance_min_m: float | None = None,
    distance_max_m: float | None = None,
    center_red_ratio_threshold: float | None = None,
    lidar_distance_threshold_m: float | None = None,
) -> dict[str, Any]:
    """距離と画面中央の赤色割合から2つ目の赤ボールを探して前進する。

    距離を測定し、設定範囲内に物体がある場合だけ前方画像を撮影する。
    画像中央の赤色割合がしきい値以上なら、現在方位を維持して前進する。
    条件を満たさない場合は小角度ずつ右旋回して探索を続ける。

    各しきい値にNoneを指定した場合は設定クラスの値を使用する。
    条件成立後は、距離しきい値までlidar_forward()で直進する。
    """
    config = SecondRedBallConfig()
    processor = image_processor or ImageProcessor()

    distance_min_m = float(
        config.DISTANCE_MIN_M
        if distance_min_m is None
        else distance_min_m
    )
    distance_max_m = float(
        config.DISTANCE_MAX_M
        if distance_max_m is None
        else distance_max_m
    )
    center_red_ratio_threshold = float(
        config.CENTER_RED_RATIO_THRESHOLD
        if center_red_ratio_threshold is None
        else center_red_ratio_threshold
    )
    lidar_distance_threshold_m = float(
        RedBallConfig.TARGET_DISTANCE_M
        if lidar_distance_threshold_m is None
        else lidar_distance_threshold_m
    )
    if distance_min_m < 0.0:
        raise ValueError("DISTANCE_MIN_M must be greater than or equal to 0")
    if distance_min_m > distance_max_m:
        raise ValueError("DISTANCE_MIN_M must not exceed DISTANCE_MAX_M")
    if not 0.0 <= center_red_ratio_threshold <= 1.0:
        raise ValueError(
            "center_red_ratio_threshold must be between 0 and 1"
        )
    if lidar_distance_threshold_m < 0.0:
        raise ValueError(
            "lidar_distance_threshold_m must be greater than or equal to 0"
        )

    history: list[dict[str, Any]] = []
    last_distance_m = None
    last_red_result = None
    thresholds = {
        "distance_min_m": distance_min_m,
        "distance_max_m": distance_max_m,
        "center_red_ratio_threshold": center_red_ratio_threshold,
    }

    try:
        for step in range(1, config.MAX_SCAN_STEPS + 1):
            measured_distance = sensor_manager.get_distance_m()
            distance_m = (
                None
                if measured_distance is None
                else float(measured_distance)
            )
            last_distance_m = distance_m
            distance_in_range = (
                distance_m is not None
                and distance_min_m <= distance_m <= distance_max_m
            )

            scan_result: dict[str, Any] = {
                "step": step,
                "distance_m": distance_m,
                "distance_in_range": distance_in_range,
                "red_result": None,
                "rotate_result": None,
            }
            history.append(scan_result)

            distance_text = (
                "測定失敗"
                if distance_m is None
                else f"{distance_m:.3f} m"
            )
            print(
                "2つ目の赤ボール探索: "
                f"step {step}/{config.MAX_SCAN_STEPS}, "
                f"distance={distance_text}, "
                f"range={distance_min_m:.3f}..{distance_max_m:.3f} m"
            )

            if distance_in_range:
                frame = sensor_manager.capture_front_frame()
                last_red_result = _without_color_mask(
                    processor.judge_red_goal_reached(
                        frame,
                        red_threshold=config.RED_THRESHOLD,
                        goal_angle_red_threshold=(
                            center_red_ratio_threshold
                        ),
                        horizontal_fov_deg=config.HORIZONTAL_FOV_DEG,
                        goal_angle_min_deg=config.CENTER_ANGLE_MIN_DEG,
                        goal_angle_max_deg=config.CENTER_ANGLE_MAX_DEG,
                    )
                )
                scan_result["red_result"] = last_red_result
                center_red_ratio = float(
                    last_red_result["goal_angle_color_ratio"]
                )

                print(
                    "2つ目の赤ボール探索: "
                    f"center_red_ratio={center_red_ratio * 100:.2f}%, "
                    "threshold="
                    f"{center_red_ratio_threshold * 100:.2f}%"
                )

                if center_red_ratio >= center_red_ratio_threshold:
                    scan_result["forward_mode"] = "lidar"
                    print(
                        "2つ目の赤ボール探索: "
                        "条件成立。lidar_forward()で"
                        f"{lidar_distance_threshold_m:.3f} mまで"
                        "前進します"
                    )
                    lidar_final_distance_m = lidar_forward(
                        driver,
                        sensor_manager,
                        lidar_distance_threshold_m,
                    )
                    approach_completed = lidar_final_distance_m is not None
                    return {
                        "target_found": True,
                        "moved_forward": approach_completed,
                        "reason": (
                            "LiDARの停止距離まで前進しました"
                            if approach_completed
                            else "LiDARによる前進を完了できませんでした"
                        ),
                        "steps": step,
                        "last_distance_m": (
                            lidar_final_distance_m
                            if approach_completed
                            else distance_m
                        ),
                        "detection_distance_m": distance_m,
                        "last_red_result": last_red_result,
                        "forward_mode": "lidar",
                        "forward_duration_s": None,
                        "lidar_distance_threshold_m": (
                            lidar_distance_threshold_m
                        ),
                        "lidar_final_distance_m": lidar_final_distance_m,
                        "thresholds": thresholds,
                        "history": history,
                    }

            if step == config.MAX_SCAN_STEPS:
                break

            print(
                "2つ目の赤ボール探索: "
                f"右へ{config.SCAN_ANGLE_DEG:.1f}度旋回します"
            )
            rotate_result = navigation_controller.rotate_by_angle(
                driver,
                sensor_manager,
                config.SCAN_ANGLE_DEG,
                speed=config.ROTATE_SPEED,
                tolerance_deg=config.ROTATE_TOLERANCE_DEG,
                timeout_s=config.ROTATE_TIMEOUT_S,
            )
            scan_result["rotate_result"] = rotate_result

            if not rotate_result["reached"]:
                return {
                    "target_found": False,
                    "moved_forward": False,
                    "reason": "探索中の旋回が完了しませんでした",
                    "steps": step,
                    "last_distance_m": last_distance_m,
                    "last_red_result": last_red_result,
                    "forward_mode": None,
                    "forward_duration_s": None,
                    "lidar_distance_threshold_m": None,
                    "lidar_final_distance_m": None,
                    "thresholds": thresholds,
                    "history": history,
                }

            if config.INTER_ROTATION_INTERVAL_S > 0.0:
                print(
                    "2つ目の赤ボール探索: "
                    f"次の旋回まで"
                    f"{config.INTER_ROTATION_INTERVAL_S:.2f}秒待機します"
                )
                time.sleep(config.INTER_ROTATION_INTERVAL_S)
    finally:
        driver.stop()

    return {
        "target_found": False,
        "moved_forward": False,
        "reason": "最大探索回数内に条件を満たす赤ボールを検出できませんでした",
        "steps": len(history),
        "last_distance_m": last_distance_m,
        "last_red_result": last_red_result,
        "forward_mode": None,
        "forward_duration_s": None,
        "lidar_distance_threshold_m": None,
        "lidar_final_distance_m": None,
        "thresholds": thresholds,
        "history": history,
    }


def lidar_forward(
    driver: Any,
    sensor_manager: SensorManager,
    distance_threshold_m: float,
    *,
    base_speed: float = LidarConfig.FORWARD_SPEED,
    loop_interval_s: float = LidarConfig.LOOP_INTERVAL_S,
    timeout_s: float = LidarConfig.FORWARD_TIMEOUT_S,
) -> Optional[float]:
    """開始時の方位を保ち、制限時間内に距離閾値まで直進する。"""
    distance_threshold_m = float(distance_threshold_m)
    base_speed = float(base_speed)
    loop_interval_s = float(loop_interval_s)
    timeout_s = float(timeout_s)

    if timeout_s <= 0.0:
        raise ValueError("timeout_s must be greater than 0")

    distance = sensor_manager.get_distance_m()
    if distance is None:
        driver.stop()
        print("距離を測定できませんでした")
        return None

    distance = float(distance)
    target_heading = float(sensor_manager.get_heading_deg())
    print(
        f"直進開始: 距離={distance:.3f} m, "
        f"保持方位={target_heading:.1f} deg"
    )

    navigation_controller = NavigationController()
    previous_error = 0.0
    deadline = time.monotonic() + timeout_s

    try:
        while distance > distance_threshold_m:
            if time.monotonic() >= deadline:
                print(f"直進が{timeout_s:g}秒でタイムアウトしました")
                break

            _, _, previous_error = (
                navigation_controller.drive_toward_heading(
                    driver,
                    sensor_manager,
                    target_heading=target_heading,
                    base_speed=base_speed,
                    prev_error=previous_error,
                    loop_interval=loop_interval_s,
                )
            )
            time.sleep(loop_interval_s)

            measured_distance = sensor_manager.get_distance_m()
            if measured_distance is None:
                print("直進中に距離を測定できなくなりました")
                break
            distance = float(measured_distance)
    finally:
        driver.stop()

    if distance > distance_threshold_m:
        print(
            f"停止距離が閾値を超えています: "
            f"{distance:.3f} m > {distance_threshold_m:.3f} m"
        )
        return None

    print(f"停止距離={distance:.3f} m")
    return distance
