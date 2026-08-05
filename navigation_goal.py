import math
import random

import cv2
import red_ball_selector as selector
import time
from copy import copy
from typing import Any

from config import (
    RedBallConfig,
    RedConeConfig,
)
from image_processor import ImageProcessor
from logger import Logger
from navigation_controller import NavigationController
from sensor_manager import SensorManager


IMU_SETTLE_TIME_S = 0.5
ROTATION_HISTORY_FIELDS = (
    "target_angle_deg", "rotated_angle_deg", "remaining_angle_deg", "reached"
)
CENTERING_HISTORY_FIELDS = (
    "centered", "red_detected", "reason", "steps", "initial_selected_position"
)
APPROACH_HISTORY_FIELDS = ("reached", "reason", "steps", "last_distance_m")
CANDIDATE_HISTORY_FIELDS = (
    "x", "y", "center_offset_ratio", "candidate_source"
)


def _log(stage: str, *, logger: Logger | None = None, **fields: Any) -> None:
    """実機誘導ログを1イベント1行で出力する。"""
    details = " ".join(f"{key}={value}" for key, value in fields.items())
    message = f"{stage}: {details}"
    if logger is not None:
        logger.console(message)
    else:
        print(message, flush=True)


def _without_color_mask(color_result: dict[str, Any]) -> dict[str, Any]:
    """履歴用の色検出結果から大きな画像マスクを除外する。"""
    summary = color_result.copy()
    summary.pop("color_mask", None)
    return summary


def _result_summary(
    result: dict[str, Any] | None,
    fields: tuple[str, ...],
) -> dict[str, Any] | None:
    """履歴には指定した診断項目だけを残す。"""
    if result is None:
        return None
    return {key: result[key] for key in fields if key in result}


def _detection_summary(result: dict[str, Any]) -> dict[str, Any]:
    summary = _result_summary(
        result,
        (
            "is_color_detected", "total_color_ratio", "color_peak_column_x",
            "color_peak_center_offset_ratio",
            "largest_color_component_area_ratio",
            "largest_color_component_center_x", "goal_reached", "goal_reason",
            "goal_angle_color_ratio", "goal_angle_min_deg", "goal_angle_max_deg",
        ),
    ) or {}
    candidates = result.get("red_ball_candidates")
    if candidates is not None:
        summary["candidate_count"] = len(candidates)
    selected_ball = result.get("selected_red_ball")
    if selected_ball is not None:
        summary["selected_red_ball"] = _result_summary(
            selected_ball, CANDIDATE_HISTORY_FIELDS
        )
    return summary


def _is_red_cone_detected(
    result: dict[str, Any],
    min_component_area_ratio: float,
) -> bool:
    """赤の総量、方向ピーク、最大連結領域がそろった候補だけを返す。"""
    mask = result["color_mask"]
    label_count, _, stats, centroids = cv2.connectedComponentsWithStats(
        mask,
        connectivity=8,
    )
    largest_label = (
        int(stats[1:, cv2.CC_STAT_AREA].argmax()) + 1
        if label_count > 1 else None
    )
    largest_area = (
        int(stats[largest_label, cv2.CC_STAT_AREA])
        if largest_label is not None else 0
    )
    largest_center_x = (
        float(centroids[largest_label][0])
        if largest_label is not None else None
    )
    image_area = result["image_width"] * result["image_height"]
    largest_area_ratio = largest_area / image_area if image_area else 0.0
    result["largest_color_component_area_ratio"] = largest_area_ratio
    result["largest_color_component_center_x"] = largest_center_x
    detected = bool(
        result["is_color_detected"]
        and result["color_peak_column_x"] is not None
        and largest_area_ratio >= min_component_area_ratio
    )
    if detected:
        result["color_peak_column_x"] = largest_center_x
        result["color_peak_center_offset_ratio"] = (
            ((largest_center_x + 0.5) / result["image_width"]) - 0.5
        )
    return detected


def _find_red_cone_in_view(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    processor: ImageProcessor,
    red_cone_config: RedConeConfig,
    logger: Logger | None = None,
):
    """カメラ画像内に赤コーンが入るまで、基礎旋回を使って探索する。"""
    scan_history = []
    for scan_index in range(red_cone_config.MAX_SCAN_STEPS):
        navigation_controller.restore_posture(driver, sensor_manager)
        frame = sensor_manager.capture_front_frame()
        red_result = processor.detect_color(
            frame,
            hsv_ranges=processor.RED_HSV_RANGES,
            color_threshold=red_cone_config.RED_THRESHOLD,
            column_threshold=red_cone_config.RED_COLUMN_THRESHOLD,
            column_average_width=red_cone_config.RED_COLUMN_AVERAGE_WIDTH,
        )
        red_result["is_color_detected"] = _is_red_cone_detected(
            red_result,
            red_cone_config.MIN_RED_COMPONENT_AREA_RATIO,
        )
        red_result = _without_color_mask(red_result)
        scan_history.append({
            "scan_index": scan_index,
            "red_result": _detection_summary(red_result),
        })
        _log("赤コーン探索", logger=logger,
             scan=f"{scan_index + 1}/{red_cone_config.MAX_SCAN_STEPS}",
             total=f"{red_result['total_color_ratio'] * 100:.2f}%",
             component=f"{red_result['largest_color_component_area_ratio'] * 100:.2f}%",
             column=red_result["color_peak_column_x"], detected=red_result["is_color_detected"])

        if red_result["is_color_detected"]:
            return red_result, scan_history

        if scan_index < red_cone_config.MAX_SCAN_STEPS - 1:
            navigation_controller.rotate_by_angle(
                driver,
                sensor_manager,
                red_cone_config.SCAN_ANGLE_DEG,
                speed=red_cone_config.ROTATE_SPEED,
                tolerance_deg=red_cone_config.ROTATE_TOLERANCE_DEG,
                timeout_s=red_cone_config.ROTATE_TIMEOUT_S,
            )

    return None, scan_history


def _detect_red_balls(
    processor: ImageProcessor,
    frame: Any,
) -> dict[str, Any]:
    """円候補を優先し、円候補がない場合だけサイズ候補を返す。"""
    circle_candidates = processor.detect_red_ball_circle_candidates(frame)
    color_result = processor.detect_color(
        frame,
        hsv_ranges=processor.RED_HSV_RANGES,
        color_threshold=RedBallConfig.SWITCH_RED_RATIO,
        column_threshold=RedBallConfig.RED_COLUMN_THRESHOLD,
        column_average_width=RedBallConfig.RED_COLUMN_AVERAGE_WIDTH,
    )
    size_candidates = []
    if not circle_candidates:
        size_candidates = processor.detect_red_ball_candidates(
            frame,
            color_result=color_result,
        )
    merged_candidates = selector.merge_candidates(circle_candidates, size_candidates)
    return {
        "is_color_detected": bool(merged_candidates),
        "total_color_ratio": color_result["total_color_ratio"],
        "image_width": color_result["image_width"],
        "image_height": color_result["image_height"],
        "red_ball_candidates": merged_candidates,
    }


class RedBallGuidance:
    """赤ボール固有の認識・位置合わせ・接近動作を担当する。"""

    def __init__(
        self,
        navigation_controller: NavigationController,
        driver: Any,
        sensor_manager: SensorManager,
        *,
        logger: Logger | None = None,
    ) -> None:
        self.navigation = navigation_controller
        self.driver = driver
        self.sensors = sensor_manager
        self.logger = logger
        self.processor = ImageProcessor(logger=logger)
        self.config = RedBallConfig()

    def detect(self, frame: Any) -> dict[str, Any]:
        return _detect_red_balls(self.processor, frame)

    def capture(self) -> dict[str, Any]:
        self.navigation.restore_posture(self.driver, self.sensors)
        return self.detect(self.sensors.capture_front_frame())

    def rotate(
        self,
        angle_deg: float,
        *,
        turn_gain: float = 1.0,
    ) -> dict[str, Any]:
        return self.navigation.rotate_by_angle(
            self.driver,
            self.sensors,
            angle_deg,
            turn_gain=turn_gain,
            speed=self.config.CENTERING_ROTATE_SPEED,
            tolerance_deg=self.config.CENTERING_ROTATE_TOLERANCE_DEG,
            timeout_s=self.config.ROTATE_TIMEOUT_S,
        )


    def align(
        self,
        *,
        target_hint_x: float | None = None,
        target_hint_size_px: float | None = None,
        distance_m: float | None = None,
    ) -> dict[str, Any]:
        """カメラの横ずれを補正し、同じ赤ボールを機体正面へ合わせる。"""
        red_ball_config = self.config
        local_target_hint_x = target_hint_x
        local_target_hint_size_px = target_hint_size_px
        initial_selected_position = None
        red_result = None
        step = -1

        def finish(
            reason: str,
            *,
            centered: bool = False,
            red_detected: bool = True,
        ) -> dict[str, Any]:
            return {
                "centered": centered,
                "red_detected": red_detected,
                "reason": reason,
                "steps": step + 1,
                "last_red_result": red_result,
                "initial_selected_position": initial_selected_position,
            }

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
            red_result = self.capture()
            selected_ball = None
            selected_by_hint = False
            if local_target_hint_x is not None:
                selected_ball = selector.select_near_hint(
                    red_result,
                    local_target_hint_x,
                    red_ball_config,
                    target_hint_size_px=local_target_hint_size_px,
                )
                selected_by_hint = selected_ball is not None
            if selected_ball is None and local_target_hint_x is not None:
                _log("赤ボール中央合わせ", logger=self.logger,
                     step=f"{step + 1}/{red_ball_config.MAX_CENTERING_STEPS}", result="reacquire")
                local_target_hint_x = None
                local_target_hint_size_px = None
                continue
            if selected_ball is None:
                selected_ball = selector.select_nearest(red_result)
            if selected_ball is None:
                reason = "赤ボールを認識できませんでした"
                _log("赤ボール中央合わせ", logger=self.logger,
                     result="failed", reason=reason)
                return finish(reason, red_detected=False)

            if initial_selected_position is None:
                initial_selected_position = selector.classify_position(
                    red_result,
                    selected_ball,
                )
            red_result["selected_red_ball"] = selected_ball
            local_target_hint_x = float(selected_ball["x"])
            selected_size_px = selector.candidate_visible_size(selected_ball)
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

            _log("赤ボール中央合わせ", logger=self.logger,
                 step=f"{step + 1}/{red_ball_config.MAX_CENTERING_STEPS}",
                 total=f"{red_result['total_color_ratio'] * 100:.2f}%",
                 candidates=len(red_result.get("red_ball_candidates", [])),
                 initial=initial_selected_position, ball_x=f"{selected_ball['x']:.1f}",
                 detected=f"{detected_angle_deg:.2f}deg", target=f"{target_angle_deg:.2f}deg",
                 turn=f"{turn_angle:.2f}deg", gain=f"{turn_gain:.2f}", locked=selected_by_hint,
                 score=f"{selected_ball.get('target_lock_score', 1.0):.3f}",
                 rotate=f"{rotate_angle:.2f}deg")

            if abs(turn_angle) <= red_ball_config.CENTERING_TOLERANCE_DEG:
                return finish(
                    "赤ボールが機体正面の補正位置に入りました",
                    centered=True,
                )

            self.rotate(
                turn_angle,
                turn_gain=turn_gain,
            )
            predicted_hint_x = selector.predict_x_after_rotation(
                selected_ball,
                turn_angle,
                red_ball_config.HORIZONTAL_FOV_DEG,
                red_result.get("image_width"),
            )
            if predicted_hint_x is not None:
                local_target_hint_x = predicted_hint_x

        return finish("最大試行回数内に中央合わせできませんでした")


    def approach(
        self,
        target_distance_m: float,
        *,
        target_hint_x: float | None = None,
        target_hint_size_px: float | None = None,
        initial_centering_distance_m: float | None = None,
        log_prefix: str = "赤ボール接近",
    ) -> dict[str, Any]:
        """赤ボールを中央に合わせ、設定された許容範囲内まで接近する。"""
        red_ball_config = self.config
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
        last_red_result = None
        step = 0

        def finish(reason: str, *, reached: bool = False) -> dict[str, Any]:
            return {
                "reached": reached,
                "reason": reason,
                "steps": step,
                "last_distance_m": last_distance_m,
                "centering_result": last_center_result,
                "last_red_result": last_red_result,
                "history": history,
            }

        for step in range(1, red_ball_config.MAX_APPROACH_STEPS + 1):
            center_result = self.align(
                target_hint_x=target_hint_x,
                target_hint_size_px=target_hint_size_px,
                distance_m=last_distance_m,
            )
            last_center_result = center_result
            approach_record = {
                "approach_step": step,
                "centering_result": _result_summary(
                    center_result, CENTERING_HISTORY_FIELDS
                ),
                "centering_distance_m": last_distance_m,
            }
            history.append(approach_record)
            last_red_result = center_result.get("last_red_result")
            if not center_result["centered"]:
                return finish(center_result["reason"])

            last_red_result = last_red_result or {}
            selected_ball = last_red_result.get("selected_red_ball")
            if selected_ball is not None:
                target_hint_x = float(selected_ball["x"])
                selected_size_px = selector.candidate_visible_size(selected_ball)
                if selected_size_px is not None:
                    target_hint_size_px = selected_size_px

            distance_m = self.sensors.get_distance_m()
            if distance_m is None:
                self.driver.stop()
                return finish("距離を測定できませんでした")

            last_distance_m = float(distance_m)
            distance_error_m = last_distance_m - target_distance_m
            approach_record["distance_m"] = last_distance_m
            approach_record["distance_error_m"] = distance_error_m
            if last_distance_m < too_close_distance_m:
                _log(log_prefix, logger=self.logger,
                     step=step, distance=f"{last_distance_m:.3f}m",
                     target=f"{target_distance_m:.3f}m",
                     action=f"reverse:{red_ball_config.REVERSE_DURATION_S:.2f}s")
                _reverse_for_duration(
                    self.driver,
                    red_ball_config.REVERSE_SPEED,
                    red_ball_config.REVERSE_DURATION_S,
                )
                continue

            if last_distance_m <= stop_distance_m:
                self.driver.stop()
                _log(log_prefix, logger=self.logger,
                     step=step, distance=f"{last_distance_m:.3f}m",
                     target=f"{target_distance_m:.3f}m", result="reached")
                return finish("目標距離に到達しました", reached=True)

            forward_duration = _duration_from_threshold(
                distance_error_m,
                red_ball_config.FORWARD_DURATION_S,
                red_ball_config.FORWARD_DURATION_BY_DISTANCE_ERROR_M,
            )
            approach_record["forward_duration_s"] = forward_duration
            heading_before_deg = float(self.sensors.get_heading_deg())
            self.navigation.pd_forward(
                self.driver,
                self.sensors,
                forward_duration,
                base_speed=red_cone_config.FORWARD_SPEED,
                loop_interval=red_cone_config.LOOP_INTERVAL_S,
                stop_ramp_steps=red_cone_config.STOP_RAMP_STEPS,
                stop_ramp_interval=red_cone_config.STOP_RAMP_INTERVAL_S,
            )
            time.sleep(IMU_SETTLE_TIME_S)
            heading_after_deg = float(self.sensors.get_heading_deg())
            heading_change_deg = self.navigation.heading_error(
                heading_after_deg,
                heading_before_deg,
            )
            heading_restore_result = self.rotate_precisely(
                -heading_change_deg,
            )
            approach_record["heading_change_deg"] = heading_change_deg
            approach_record["heading_restore_result"] = _result_summary(
                heading_restore_result, ROTATION_HISTORY_FIELDS
            )
            approach_record["target_hint_x"] = target_hint_x
            _log(log_prefix, logger=self.logger,
                 step=step, distance=f"{last_distance_m:.3f}m",
                 target=f"{target_distance_m:.3f}m", forward=f"{forward_duration:.2f}s",
                 heading_change=f"{heading_change_deg:+.2f}deg",
                 restore_remaining=f"{heading_restore_result['remaining_angle_deg']:+.2f}deg")
            if not heading_restore_result["reached"]:
                return finish("前進後の方位を元に戻せませんでした")

        last_red_result = (last_center_result or {}).get("last_red_result")
        return finish("最大試行回数内に目標距離まで近づけませんでした")


    def rotate_precisely(
        self,
        target_angle_deg: float,
    ) -> dict[str, Any]:
        """停止後の惰性を含むIMU実回転角から残差を求めて微調整する。"""
        red_ball_config = self.config
        target_angle_deg = float(target_angle_deg)
        settle_time_s = IMU_SETTLE_TIME_S
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
            heading_before_deg = float(self.sensors.get_heading_deg())
            rotate_result = self.rotate(
                remaining_angle_deg,
                turn_gain=turn_gain,
            )
            time.sleep(settle_time_s)
            settled_heading_deg = float(self.sensors.get_heading_deg())
            reported_rotated_angle_deg = float(
                rotate_result.get("rotated_angle_deg", 0.0)
            )
            wrapped_settled_angle_deg = self.navigation.heading_error(
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
                "step": correction_index + 1,
                "requested_angle_deg": remaining_angle_deg,
                "turn_gain": turn_gain,
                "reached": rotate_result["reached"],
                "reported_rotated_angle_deg": reported_rotated_angle_deg,
                "settled_rotated_angle_deg": settled_rotated_angle_deg,
                "total_rotated_angle_deg": rotated_angle_deg,
            }
            history.append(settled_rotate_result)
            _log("精密旋回", logger=self.logger,
                 step=correction_index + 1,
                 requested=f"{remaining_angle_deg:+.2f}deg", gain=f"{turn_gain:.2f}",
                 reported=f"{reported_rotated_angle_deg:+.2f}deg",
                 settled=f"{settled_rotated_angle_deg:+.2f}deg",
                 total=f"{rotated_angle_deg:+.2f}deg", reached=rotate_result["reached"])
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


def align_red_ball_to_center(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    *,
    target_hint_x: float | None = None,
    target_hint_size_px: float | None = None,
    distance_m: float | None = None,
    logger: Logger | None = None,
) -> dict[str, Any]:
    """既存API互換の赤ボール中央合わせ入口。"""
    return RedBallGuidance(
        navigation_controller, driver, sensor_manager, logger=logger
    ).align(
        target_hint_x=target_hint_x,
        target_hint_size_px=target_hint_size_px,
        distance_m=distance_m,
    )


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


def _scan_red_target_360(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    processor: ImageProcessor,
    red_cone_config: RedConeConfig,
    red_ratio_threshold: float,
    scan_angle_deg: float,
    logger: Logger | None = None,
) -> dict[str, Any]:
    """現在地点で360度旋回しながら赤いゴール目標を探索する。"""
    scan_history = []
    rotation_completed_deg = 0.0
    scan_index = 0
    last_red_result = None

    def finish(reason: str, *, red_detected: bool = False) -> dict[str, Any]:
        return {
            "red_detected": red_detected,
            "reason": reason,
            "rotation_completed_deg": rotation_completed_deg,
            "last_red_result": last_red_result,
            "scan_history": scan_history,
        }

    while rotation_completed_deg < 360.0:
        frame = sensor_manager.capture_front_frame()
        last_red_result = processor.detect_color(
            frame,
            hsv_ranges=processor.RED_HSV_RANGES,
            color_threshold=red_ratio_threshold,
            column_threshold=red_cone_config.RED_COLUMN_THRESHOLD,
            column_average_width=red_cone_config.RED_COLUMN_AVERAGE_WIDTH,
        )
        last_red_result["is_color_detected"] = _is_red_cone_detected(
            last_red_result,
            red_cone_config.MIN_RED_COMPONENT_AREA_RATIO,
        )
        last_red_result = _without_color_mask(last_red_result)
        scan_record = {
            "scan_index": scan_index,
            "rotation_completed_deg": rotation_completed_deg,
            "red_result": _detection_summary(last_red_result),
            "rotation_result": None,
        }
        scan_history.append(scan_record)
        _log(
            "GNSSゴール周辺探索",
            logger=logger,
            scan=scan_index + 1,
            rotated=f"{rotation_completed_deg:.1f}deg",
            red=f"{last_red_result['total_color_ratio'] * 100:.2f}%",
            component=(
                f"{last_red_result['largest_color_component_area_ratio'] * 100:.2f}%"
            ),
            detected=last_red_result["is_color_detected"],
        )

        if last_red_result["is_color_detected"]:
            return finish(
                "赤ターゲットの検知条件を満たしました",
                red_detected=True,
            )

        rotation_angle_deg = min(
            scan_angle_deg,
            360.0 - rotation_completed_deg,
        )
        rotation_result = navigation_controller.rotate_by_angle(
            driver,
            sensor_manager,
            rotation_angle_deg,
            speed=red_cone_config.ROTATE_SPEED,
            tolerance_deg=red_cone_config.ROTATE_TOLERANCE_DEG,
            timeout_s=red_cone_config.ROTATE_TIMEOUT_S,
        )
        scan_record["rotation_angle_deg"] = rotation_angle_deg
        scan_record["rotation_result"] = _result_summary(
            rotation_result,
            ROTATION_HISTORY_FIELDS,
        )

        if not rotation_result["reached"]:
            return finish("360度探索中の旋回が完了しませんでした")

        rotation_completed_deg += rotation_angle_deg
        scan_index += 1

    return finish("360度探索しましたが赤を検知できませんでした")


def search_around_gnss_goal(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    search_distance_m: float,
    red_ratio_threshold: float,
    *,
    status_callback=None,
    scan_angle_deg: float | None = None,
    processor: ImageProcessor | None = None,
    relocate_before_scan: bool = False,
    logger: Logger | None = None,
) -> dict[str, Any]:
    """必要なら現在地を探索後、ランダム地点へ移動して再探索する。"""
    search_distance_m = float(search_distance_m)
    if search_distance_m <= 0.0:
        raise ValueError("search_distance_m must be greater than 0")
    red_ratio_threshold = float(red_ratio_threshold)
    if not 0.0 <= red_ratio_threshold <= 1.0:
        raise ValueError("red_ratio_threshold must be between 0 and 1")

    red_cone_config = RedConeConfig()
    if scan_angle_deg is None:
        scan_angle_deg = red_cone_config.SCAN_ANGLE_DEG
    scan_angle_deg = float(scan_angle_deg)
    if not 0.0 < scan_angle_deg <= 360.0:
        raise ValueError("scan_angle_deg must be greater than 0 and at most 360")

    base_gnss = None
    target_gnss = None
    random_bearing_deg = None
    random_distance_m = None
    initial_scan_result = None
    scan_result = None

    def finish(
        reason: str,
        *,
        red_detected: bool = False,
        target_reached: bool = False,
    ) -> dict[str, Any]:
        return {
            "red_detected": red_detected,
            "target_reached": target_reached,
            "reason": reason,
            "base_gnss": base_gnss,
            "target_gnss": target_gnss,
            "random_bearing_deg": random_bearing_deg,
            "random_distance_m": random_distance_m,
            "search_distance_m": search_distance_m,
            "initial_scan_result": initial_scan_result,
            "scan_result": scan_result,
        }

    gnss = sensor_manager.get_gnss()
    latitude_deg = gnss.get("latitude_deg")
    longitude_deg = gnss.get("longitude_deg")
    if (
        not gnss.get("has_fix")
        or latitude_deg is None
        or longitude_deg is None
    ):
        driver.stop()
        return finish("探索開始地点のGNSS座標を取得できませんでした")

    latitude_deg = float(latitude_deg)
    longitude_deg = float(longitude_deg)
    base_gnss = {
        "latitude_deg": latitude_deg,
        "longitude_deg": longitude_deg,
    }

    try:
        if processor is None:
            processor = ImageProcessor(logger=logger)

        if not relocate_before_scan:
            initial_scan_result = _scan_red_target_360(
                navigation_controller,
                driver,
                sensor_manager,
                processor,
                red_cone_config,
                red_ratio_threshold,
                scan_angle_deg,
                logger,
            )
            scan_result = initial_scan_result
            if initial_scan_result["red_detected"]:
                return finish(
                    initial_scan_result["reason"],
                    red_detected=True,
                    target_reached=True,
                )

        random_bearing_deg = random.uniform(0.0, 360.0)
        random_distance_m = random.uniform(0.0, search_distance_m)
        goal_latitude_deg = float(navigation_controller.target_latitude_deg)
        goal_longitude_deg = float(navigation_controller.target_longitude_deg)

        earth_radius_m = 6371000.0
        angular_distance = random_distance_m / earth_radius_m
        latitude_rad = math.radians(goal_latitude_deg)
        longitude_rad = math.radians(goal_longitude_deg)
        bearing_rad = math.radians(random_bearing_deg)
        target_latitude_rad = math.asin(
            math.sin(latitude_rad) * math.cos(angular_distance)
            + math.cos(latitude_rad)
            * math.sin(angular_distance)
            * math.cos(bearing_rad)
        )
        target_longitude_rad = longitude_rad + math.atan2(
            math.sin(bearing_rad)
            * math.sin(angular_distance)
            * math.cos(latitude_rad),
            math.cos(angular_distance)
            - math.sin(latitude_rad) * math.sin(target_latitude_rad),
        )
        target_latitude_deg = math.degrees(target_latitude_rad)
        target_longitude_deg = (
            math.degrees(target_longitude_rad) + 540.0
        ) % 360.0 - 180.0
        target_gnss = {
            "latitude_deg": target_latitude_deg,
            "longitude_deg": target_longitude_deg,
        }

        _log(
            "GNSSゴール周辺探索",
            logger=logger,
            origin=f"{goal_latitude_deg:.7f},{goal_longitude_deg:.7f}",
            bearing=f"{random_bearing_deg:.1f}deg",
            distance=f"{random_distance_m:.1f}m",
            target=f"{target_latitude_deg:.7f},{target_longitude_deg:.7f}",
        )

        target_navigation_controller = NavigationController(
            target_latitude_deg=target_latitude_deg,
            target_longitude_deg=target_longitude_deg,
            logger=logger,
        )
        for config_name in (
            "pd_config",
            "posture_restore_config",
            "follow_target_config",
            "parachute_avoidance_config",
        ):
            setattr(
                target_navigation_controller,
                config_name,
                copy(getattr(navigation_controller, config_name)),
            )

        target_reached = target_navigation_controller.follow_target(
            driver,
            sensor_manager,
            status_callback=status_callback,
        )
        if not target_reached:
            return finish("ランダムに設定した探索地点へ到着できませんでした")

        scan_result = _scan_red_target_360(
            target_navigation_controller,
            driver,
            sensor_manager,
            processor,
            red_cone_config,
            red_ratio_threshold,
            scan_angle_deg,
            logger,
        )

        return finish(
            scan_result["reason"],
            red_detected=scan_result["red_detected"],
            target_reached=True,
        )
    finally:
        driver.stop()


def guide_to_red_cone(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    stop_red_ratio_threshold: float | None = None,
    forward_duration_by_red_ratio: tuple[tuple[float, float], ...] | None = None,
    *,
    logger: Logger | None = None,
) -> dict[str, Any]:
    """NavigationControllerを使って赤コーンを探し、正面へ回頭して前進する。"""
    processor = ImageProcessor(logger=logger)
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
    step = -1

    def finish(
        reason: str,
        *,
        reached: bool = False,
        steps: int | None = None,
        **details: Any,
    ) -> dict[str, Any]:
        return {
            "goal_reached": reached,
            "reason": reason,
            "steps": step + 1 if steps is None else steps,
            "history": history,
            **details,
            "last_goal_result": last_goal_result,
        }

    for step in range(red_cone_config.MAX_GUIDANCE_STEPS):
        # 1. 赤コーンが画面に入るまで、撮影と少しの旋回を繰り返す。
        red_result, scan_history = _find_red_cone_in_view(
            navigation_controller,
            driver,
            sensor_manager,
            processor,
            red_cone_config,
            logger,
        )

        if red_result is None:
            return finish(
                "赤コーンを見つけられませんでした",
                steps=step,
                scan_history=scan_history,
            )

        if (
            stop_red_ratio_threshold is not None
            and red_result["total_color_ratio"] >= stop_red_ratio_threshold
        ):
            return finish(
                "赤検知率が切り替えしきい値以上になりました",
                red_ratio_threshold_reached=True,
                last_red_result=red_result,
            )

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

        _log("赤コーン誘導", logger=logger,
             step=f"{step + 1}/{red_cone_config.MAX_GUIDANCE_STEPS}",
             red=f"{red_result['total_color_ratio'] * 100:.2f}%",
             column=red_result["color_peak_column_x"], turn=f"{turn_angle:.2f}deg",
             forward=f"{forward_duration:.2f}s")
        navigation_controller.pd_forward(
            driver,
            sensor_manager,
            forward_duration,
            base_speed=red_cone_config.FORWARD_SPEED,
            loop_interval=red_cone_config.LOOP_INTERVAL_S,
            stop_ramp_steps=red_cone_config.STOP_RAMP_STEPS,
            stop_ramp_interval=red_cone_config.STOP_RAMP_INTERVAL_S,
            enable_head_swing=True,
        )

        # 4. 前進後にもう一度撮影し、赤コーンに十分近づいたか判定する。
        navigation_controller.restore_posture(driver, sensor_manager)
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
        _log("赤コーン誘導", logger=logger,
             step=step + 1, goal=last_goal_result["goal_reached"],
             total=f"{last_goal_result['total_color_ratio'] * 100:.2f}%",
             angle_ratio=f"{last_goal_result['goal_angle_color_ratio'] * 100:.2f}%",
             angle_range=f"{last_goal_result['goal_angle_min_deg']:.1f}"
                         f"..{last_goal_result['goal_angle_max_deg']:.1f}deg")

        history.append({
            "step": step + 1,
            "red_result": _detection_summary(red_result),
            "turn_angle_deg": turn_angle,
            "turn_result": _result_summary(turn_result, ROTATION_HISTORY_FIELDS),
            "forward_duration_s": forward_duration,
            "goal_result": _detection_summary(last_goal_result),
            "scan_steps": len(scan_history),
        })

        # ゴール判定が出たら、最後に少し前進して終了する。
        if last_goal_result["goal_reached"]:
            _log("赤コーン誘導", logger=logger, result="reached",
                 final_forward=f"{red_cone_config.GOAL_FINAL_FORWARD_DURATION_S:.2f}s")
            navigation_controller.pd_forward(
                driver,
                sensor_manager,
                red_cone_config.GOAL_FINAL_FORWARD_DURATION_S,
                base_speed=red_cone_config.FORWARD_SPEED,
                loop_interval=red_cone_config.LOOP_INTERVAL_S,
                stop_ramp_steps=red_cone_config.STOP_RAMP_STEPS,
                stop_ramp_interval=red_cone_config.STOP_RAMP_INTERVAL_S,
            )
            return finish(last_goal_result["goal_reason"], reached=True)

    return finish(
        "最大試行回数内にゴール判定できませんでした",
        red_ratio_threshold_reached=False,
    )


def guide_to_red_ball(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    *,
    logger: Logger | None = None,
) -> dict[str, Any]:
    """最初の赤ボールへ誘導し、距離センサで目標距離付近まで近づく。"""
    guidance = RedBallGuidance(
        navigation_controller,
        driver,
        sensor_manager,
        logger=logger,
    )
    red_ball_config = guidance.config

    cone_result = guide_to_red_cone(
        navigation_controller,
        driver,
        sensor_manager,
        stop_red_ratio_threshold=red_ball_config.SWITCH_RED_RATIO,
        forward_duration_by_red_ratio=(
            red_ball_config.CONE_FORWARD_DURATION_BY_RED_RATIO
        ),
        logger=logger,
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
    approach_result = guidance.approach(
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
    _log("赤ボール誘導", logger=logger,
         initial_position=initial_ball_position)
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
    logger: Logger | None = None,
) -> dict[str, Any]:
    """最初の赤ボール到達後、隣の赤ボールへ順に近づく。"""
    guidance = RedBallGuidance(
        navigation_controller,
        driver,
        sensor_manager,
        logger=logger,
    )
    red_ball_config = guidance.config

    history: list[dict[str, Any]] = []
    last_distance_m = None
    last_red_result = None
    initial_turn_angle_deg = None
    initial_turn_result = None
    initial_fallback_turn_result = None
    unrestricted_first_selection = False

    def finish(
        reason: str,
        *,
        reached: bool = False,
        approached_balls: int = 0,
        last_red_result: dict[str, Any] | None = None,
        **details: Any,
    ) -> dict[str, Any]:
        return {
            "square_zone_reached": reached,
            "reason": reason,
            "approached_balls": approached_balls,
            "last_distance_m": last_distance_m,
            "last_red_result": last_red_result,
            **details,
            "history": history,
        }

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
            _log("スクエアゾーン誘導", logger=logger,
                 initial_position=initial_ball_position,
                 initial_turn=f"{initial_turn_angle_deg:+.1f}deg")
            initial_turn_result = guidance.rotate_precisely(
                initial_turn_angle_deg
            )
            if not initial_turn_result["reached"]:
                return finish(
                    (
                        "スクエアゾーン進入前の"
                        f"{initial_side_turn_angle_deg:.1f}度旋回に"
                        "失敗しました"
                    ),
                    initial_ball_position=initial_ball_position,
                    initial_turn_result=initial_turn_result,
                )
            unrestricted_first_selection = True

        for target_index in range(1, red_ball_config.MAX_SQUARE_TARGETS + 1):
            red_result = guidance.capture()
            last_red_result = red_result
            visible_target_count = len(red_result["red_ball_candidates"])
            unrestricted_selection = (
                unrestricted_first_selection and target_index == 1
            )
            prefer_farthest = (
                unrestricted_selection or target_index >= 2
            )
            min_adjacent_angle_deg = (
                None
                if prefer_farthest
                else max(
                    red_ball_config.CENTERING_TOLERANCE_DEG,
                    red_ball_config.ADJACENT_MIN_ANGLE_DEG,
                )
            )
            if prefer_farthest:
                adjacent_ball = selector.select_farthest(red_result, logger=logger)
                turn_angle = (
                    None
                    if adjacent_ball is None
                    else float(adjacent_ball["center_offset_ratio"])
                    * red_ball_config.HORIZONTAL_FOV_DEG
                )
            else:
                adjacent_ball, turn_angle = selector.select_adjacent(
                    red_result,
                    red_ball_config.HORIZONTAL_FOV_DEG,
                    min_adjacent_angle_deg,
                )
            fallback_search_performed = False
            if unrestricted_selection and adjacent_ball is None:
                fallback_turn_angle_deg = -2.0 * initial_turn_angle_deg
                _log("スクエアゾーン誘導", logger=logger,
                     target=target_index, result="reacquire",
                     turn=f"{fallback_turn_angle_deg:+.1f}deg")
                initial_fallback_turn_result = guidance.rotate_precisely(
                    fallback_turn_angle_deg,
                )
                if not initial_fallback_turn_result["reached"]:
                    return finish(
                        "初回ボールの反対側探索旋回に失敗しました",
                        last_red_result=red_result,
                        initial_ball_position=initial_ball_position,
                        initial_turn_result=initial_turn_result,
                        initial_fallback_turn_result=initial_fallback_turn_result,
                    )

                fallback_search_performed = True
                red_result = guidance.capture()
                last_red_result = red_result
                visible_target_count = len(
                    red_result["red_ball_candidates"]
                )
                adjacent_ball = selector.select_farthest(red_result, logger=logger)
                turn_angle = (
                    None
                    if adjacent_ball is None
                    else float(adjacent_ball["center_offset_ratio"])
                    * red_ball_config.HORIZONTAL_FOV_DEG
                )
            commanded_turn_angle = (
                None
                if turn_angle is None
                else turn_angle
                * red_ball_config.CENTERING_LARGE_ANGLE_TURN_GAIN
            )
            target_history: dict[str, Any] = {
                "target_index": target_index,
                "red_result": _detection_summary(red_result),
                "adjacent_ball": _result_summary(
                    adjacent_ball, CANDIDATE_HISTORY_FIELDS
                ),
                "turn_angle_deg": turn_angle,
                "rotate_result": None,
                "approach_history": [],
                "fallback_search_performed": fallback_search_performed,
                "unrestricted_selection": unrestricted_selection,
                "selection_strategy": (
                    "farthest"
                    if prefer_farthest
                    else "adjacent_nearest"
                ),
            }
            history.append(target_history)

            min_angle_text = (
                "none"
                if min_adjacent_angle_deg is None
                else f"{min_adjacent_angle_deg:.1f}deg"
            )
            _log("スクエアゾーン誘導", logger=logger,
                 target=f"{target_index}/{red_ball_config.MAX_SQUARE_TARGETS}",
                 candidates=visible_target_count, min_angle=min_angle_text,
                 selection=target_history["selection_strategy"],
                 selected_x=None if adjacent_ball is None else adjacent_ball["x"])

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
                front_ball = selector.select_nearest(red_result)
                if front_ball is None:
                    return finish(
                        (
                            "逆方向探索後も赤ボール候補を"
                            "認識できませんでした"
                            if fallback_search_performed
                            else "正面の赤ボールを認識できませんでした"
                        ),
                        approached_balls=target_index - 1,
                        last_red_result=red_result,
                    )

                final_initial_distance_m = sensor_manager.get_distance_m()
                if final_initial_distance_m is None:
                    driver.stop()
                    return finish(
                        "距離を測定できませんでした",
                        approached_balls=target_index - 1,
                        last_red_result=red_result,
                    )
                final_initial_distance_m = float(final_initial_distance_m)
                _log("スクエアゾーン誘導", logger=logger,
                     action="final_approach",
                     target=f"{final_target_distance_m:.3f}m",
                     initial=f"{final_initial_distance_m:.3f}m")
                final_approach_result = guidance.approach(
                    final_target_distance_m,
                    target_hint_x=float(front_ball["x"]),
                    target_hint_size_px=selector.candidate_visible_size(front_ball),
                    initial_centering_distance_m=final_initial_distance_m,
                    log_prefix="スクエアゾーン最終接近",
                )
                target_history["final_approach_history"] = (
                    final_approach_result["history"]
                )
                last_distance_m = final_approach_result["last_distance_m"]
                return finish(
                    (
                        "正面の赤ボールまで"
                        f"{final_target_distance_m:.3f}mに到達しました"
                        if final_approach_result["reached"]
                        else final_approach_result["reason"]
                    ),
                    reached=final_approach_result["reached"],
                    approached_balls=target_index - 1,
                    last_red_result=final_approach_result["last_red_result"],
                )

            _log("スクエアゾーン誘導", logger=logger,
                 target=target_index,
                 turn=f"{turn_angle:.2f}deg", command=f"{commanded_turn_angle:.2f}deg")
            rotate_result = guidance.rotate(
                turn_angle,
                turn_gain=red_ball_config.CENTERING_LARGE_ANGLE_TURN_GAIN,
            )
            target_history["rotate_result"] = _result_summary(
                rotate_result, ROTATION_HISTORY_FIELDS
            )
            if not rotate_result["reached"]:
                return finish(
                    "隣の赤ボールへの旋回が完了しませんでした",
                    approached_balls=target_index - 1,
                    last_red_result=red_result,
                )

            target_hint_x = selector.predict_x_after_rotation(
                adjacent_ball,
                turn_angle,
                red_ball_config.HORIZONTAL_FOV_DEG,
                red_result.get("image_width"),
            )
            target_hint_size_px = selector.candidate_visible_size(adjacent_ball)
            approach_result = guidance.approach(
                red_ball_config.TARGET_DISTANCE_M,
                target_hint_x=target_hint_x,
                target_hint_size_px=target_hint_size_px,
                log_prefix="スクエアゾーン誘導",
            )
            target_history["approach_history"] = approach_result["history"]
            last_distance_m = approach_result["last_distance_m"]
            if not approach_result["reached"]:
                return finish(
                    approach_result["reason"],
                    approached_balls=target_index - 1,
                    last_red_result=approach_result["last_red_result"],
                )
    finally:
        driver.stop()

    return finish(
        "最大対象数まで誘導しても終了条件に到達しませんでした",
        approached_balls=red_ball_config.MAX_SQUARE_TARGETS,
        last_red_result=last_red_result,
    )


def guide_to_center_of_zone(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    *,
    logger: Logger | None = None,
) -> dict[str, Any]:
    """対角の赤ボールを基準に、スクエアゾーンの中心へ移動する。"""
    guidance = RedBallGuidance(
        navigation_controller,
        driver,
        sensor_manager,
        logger=logger,
    )
    red_ball_config = guidance.config
    repeat_count = max(
        0, int(red_ball_config.CENTER_OF_ZONE_REPEAT_COUNT)
    )
    diagonal_min_distance_m = 0.55
    diagonal_max_distance_m = 1.2
    history = []
    last_approach_result = None

    def finish(
        reason: str,
        *,
        reached: bool = False,
        last_distance_m: float | None = None,
    ) -> dict[str, Any]:
        return {
            "center_reached": reached,
            "reason": reason,
            "history": history,
            "last_distance_m": last_distance_m,
        }

    try:
        for cycle_index in range(repeat_count + 1):
            cycle_history: dict[str, Any] = {"cycle": cycle_index + 1}
            history.append(cycle_history)

            turn_180_result = guidance.rotate_precisely(180.0)
            cycle_history["turn_180_result"] = _result_summary(
                turn_180_result, ROTATION_HISTORY_FIELDS
            )
            _log("スクエアゾーン中心誘導", logger=logger,
                 cycle=f"{cycle_index + 1}/{repeat_count + 1}",
                 turn_180=turn_180_result["reached"])
            if not turn_180_result["reached"]:
                return finish("180度転回が完了しませんでした")

            red_result = guidance.capture()
            far_ball = selector.select_farthest(red_result, logger=logger)
            cycle_history["far_ball_result"] = _detection_summary(red_result)
            if far_ball is None:
                return finish("遠方の赤ボールを認識できませんでした")

            turn_angle_deg = (
                float(far_ball["center_offset_ratio"])
                * red_ball_config.HORIZONTAL_FOV_DEG
            )
            turn_direction = "right" if turn_angle_deg >= 0.0 else "left"
            cycle_history["turn_direction"] = turn_direction
            cycle_history["detected_turn_angle_deg"] = turn_angle_deg
            _log("スクエアゾーン中心誘導", logger=logger,
                 cycle=cycle_index + 1,
                 selection=turn_direction, angle=f"{turn_angle_deg:.2f}deg")
            alignment_result = guidance.align(
                target_hint_x=float(far_ball["x"]),
                target_hint_size_px=selector.candidate_visible_size(far_ball),
                distance_m=diagonal_min_distance_m,
            )
            cycle_history["alignment_result"] = _result_summary(
                alignment_result, CENTERING_HISTORY_FIELDS
            )
            if not alignment_result["centered"]:
                return finish(alignment_result["reason"])

            selected_red_result = (
                alignment_result.get("last_red_result") or red_result
            )
            selected_ball = selected_red_result.get("selected_red_ball")
            distance_m = sensor_manager.get_distance_m()
            if distance_m is None:
                driver.stop()
                return finish("距離を測定できませんでした")

            distance_m = float(distance_m)
            cycle_history["measured_distance_m"] = distance_m
            is_diagonal_ball = (
                diagonal_min_distance_m
                <= distance_m
                <= diagonal_max_distance_m
            )
            cycle_history["is_diagonal_ball"] = is_diagonal_ball
            approach_initial_distance_m = distance_m
            _log("スクエアゾーン中心誘導", logger=logger,
                 cycle=cycle_index + 1,
                 distance=f"{distance_m:.3f}m",
                 diagonal_range=f"{diagonal_min_distance_m:.3f}-{diagonal_max_distance_m:.3f}m",
                 is_diagonal=is_diagonal_ball)

            if not is_diagonal_ball:
                opposite_turn_angle_deg = float(
                    red_ball_config.CENTER_OF_ZONE_OPPOSITE_TURN_ANGLE_DEG
                )
                opposite_angle_deg = (
                    -opposite_turn_angle_deg
                    if turn_direction == "right"
                    else opposite_turn_angle_deg
                )
                _log("スクエアゾーン中心誘導", logger=logger,
                     action="opposite_search",
                     turn=f"{opposite_angle_deg:+.1f}deg")
                opposite_turn_result = guidance.rotate(opposite_angle_deg)
                cycle_history["opposite_turn_result"] = _result_summary(
                    opposite_turn_result, ROTATION_HISTORY_FIELDS
                )
                if not opposite_turn_result["reached"]:
                    return finish(
                        (
                            "逆方向への"
                            f"{opposite_turn_angle_deg:.1f}度旋回が"
                            "完了しませんでした"
                        ),
                        last_distance_m=distance_m,
                    )

                selected_red_result = guidance.capture()
                selected_ball = selector.select_farthest(
                    selected_red_result, logger=logger
                )
                if selected_ball is None:
                    fallback_angle_deg = -2.0 * opposite_angle_deg
                    _log("スクエアゾーン中心誘導", logger=logger,
                         action="fallback_search",
                         turn=f"{fallback_angle_deg:+.1f}deg")
                    fallback_turn_result = guidance.rotate(
                        fallback_angle_deg
                    )
                    cycle_history["fallback_turn_result"] = _result_summary(
                        fallback_turn_result, ROTATION_HISTORY_FIELDS
                    )
                    if not fallback_turn_result["reached"]:
                        return finish(
                            (
                                "反対側を探す"
                                f"{abs(fallback_angle_deg):.1f}度旋回が"
                                "完了しませんでした"
                            ),
                            last_distance_m=distance_m,
                        )

                    selected_red_result = guidance.capture()
                    selected_ball = selector.select_farthest(
                        selected_red_result, logger=logger
                    )
                    if selected_ball is None:
                        return finish(
                            (
                                "両方向で対角の赤ボールを"
                                "認識できませんでした"
                            ),
                            last_distance_m=distance_m,
                        )
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
            _log("スクエアゾーン中心誘導", logger=logger,
                 cycle=cycle_index + 1,
                 action="approach", target=f"{approach_target_distance_m:.3f}m",
                 initial=f"{approach_initial_distance_m:.3f}m")
            approach_result = guidance.approach(
                approach_target_distance_m,
                target_hint_x=(
                    None
                    if selected_ball is None
                    else float(selected_ball["x"])
                ),
                target_hint_size_px=(
                    None
                    if selected_ball is None
                    else selector.candidate_visible_size(selected_ball)
                ),
                initial_centering_distance_m=approach_initial_distance_m,
                log_prefix="スクエアゾーン中心誘導",
            )
            cycle_history["approach_result"] = _result_summary(
                approach_result, APPROACH_HISTORY_FIELDS
            )
            last_approach_result = approach_result
            if not approach_result["reached"]:
                return finish(
                    approach_result["reason"],
                    last_distance_m=approach_result["last_distance_m"],
                )

        return finish(
            (
                "対角ボールとの距離"
                f"{red_ball_config.CENTER_OF_ZONE_GOAL_DISTANCE_M:.3f}m"
                "まで誘導しました"
            ),
            reached=True,
            last_distance_m=last_approach_result["last_distance_m"],
        )
    finally:
        driver.stop()
