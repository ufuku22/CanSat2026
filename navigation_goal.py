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
) -> dict[str, Any]:
    """赤検知率ピークの列が中央に来るまで撮影と旋回を繰り返す。"""
    processor = ImageProcessor()
    red_cone_config = RedConeConfig()

    for step in range(red_cone_config.MAX_CENTERING_STEPS):
        print(
            "赤コーン中央合わせ: "
            f"step {step + 1}/{red_cone_config.MAX_CENTERING_STEPS} 撮影します"
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

        turn_angle = _red_result_to_turn_angle(
            red_result,
            red_cone_config.HORIZONTAL_FOV_DEG,
        )

        if (
            not red_result["is_color_detected"]
            or red_result["color_peak_column_x"] is None
        ):
            reason = "赤を検知できませんでした"
            if red_result["is_color_detected"]:
                reason = "赤検知率ピークの列を判定できませんでした"
            print(f"赤コーン中央合わせ: {reason}")
            return {
                "centered": False,
                "red_detected": bool(red_result["is_color_detected"]),
                "reason": reason,
                "steps": step + 1,
                "last_red_result": red_result,
            }

        rotate_angle = turn_angle * red_cone_config.CENTERING_TURN_GAIN

        print(
            "赤コーン中央合わせ: "
            f"total={red_result['total_color_ratio'] * 100:.2f}% "
            f"column={red_result['color_peak_column_x']} "
            f"turn={turn_angle:.2f}deg "
            f"rotate={rotate_angle:.2f}deg"
        )

        if abs(turn_angle) <= red_cone_config.CENTERING_TOLERANCE_DEG:
            return {
                "centered": True,
                "red_detected": True,
                "reason": "赤検知率ピークの列が中央付近に入りました",
                "steps": step + 1,
                "last_red_result": red_result,
            }

        navigation_controller.rotate_by_angle(
            driver,
            sensor_manager,
            turn_angle,
            turn_gain=red_cone_config.CENTERING_TURN_GAIN,
            speed=red_cone_config.CENTERING_ROTATE_SPEED,
            tolerance_deg=red_cone_config.CENTERING_ROTATE_TOLERANCE_DEG,
            timeout_s=red_cone_config.ROTATE_TIMEOUT_S,
        )

    return {
        "centered": False,
        "red_detected": True,
        "reason": "最大試行回数内に中央合わせできませんでした",
        "steps": red_cone_config.MAX_CENTERING_STEPS,
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


def guide_to_red_ball(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
) -> dict[str, Any]:
    """赤ボールへ誘導し、距離センサで0.8m未満まで近づく。"""
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

    for step in range(red_ball_config.MAX_DISTANCE_APPROACH_STEPS):
        center_result = align_red__peak_to_center(
            navigation_controller,
            driver,
            sensor_manager,
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
        print(f"赤ボール誘導: distance={distance_m:.3f}m")
        if distance_m < red_ball_config.TARGET_DISTANCE_M:
            driver.stop()
            return {
                "target_reached": True,
                "reason": "目標距離未満まで近づきました",
                "cone_result": cone_result,
                "centering_result": center_result,
                "steps": step + 1,
                "last_distance_m": distance_m,
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


def search_second_red_ball_and_advance(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    *,
    image_processor: ImageProcessor | None = None,
    distance_min_m: float | None = None,
    distance_max_m: float | None = None,
    center_red_ratio_threshold: float | None = None,
) -> dict[str, Any]:
    """距離と画面中央の赤色割合から2つ目の赤ボールを探して前進する。

    距離を測定し、設定範囲内に物体がある場合だけ前方画像を撮影する。
    画像中央の赤色割合がしきい値以上なら、現在方位を維持して前進する。
    条件を満たさない場合は小角度ずつ右旋回して探索を続ける。

    各しきい値にNoneを指定した場合はSecondRedBallConfigの値を使用する。
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
    if distance_min_m < 0.0:
        raise ValueError("DISTANCE_MIN_M must be greater than or equal to 0")
    if distance_min_m > distance_max_m:
        raise ValueError("DISTANCE_MIN_M must not exceed DISTANCE_MAX_M")
    if not 0.0 <= center_red_ratio_threshold <= 1.0:
        raise ValueError(
            "center_red_ratio_threshold must be between 0 and 1"
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
                    print(
                        "2つ目の赤ボール探索: "
                        f"条件成立。{config.FORWARD_DURATION_S:.2f}秒前進します"
                    )
                    navigation_controller.follow_forward(
                        driver,
                        sensor_manager,
                        config.FORWARD_DURATION_S,
                        base_speed=config.FORWARD_SPEED,
                        loop_interval=config.LOOP_INTERVAL_S,
                    )
                    return {
                        "target_found": True,
                        "moved_forward": True,
                        "reason": (
                            "距離と画面中央の赤色割合が条件を満たしました"
                        ),
                        "steps": step,
                        "last_distance_m": distance_m,
                        "last_red_result": last_red_result,
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
                    "thresholds": thresholds,
                    "history": history,
                }

            if config.POST_ROTATION_PAUSE_S > 0.0:
                time.sleep(config.POST_ROTATION_PAUSE_S)
    finally:
        driver.stop()

    return {
        "target_found": False,
        "moved_forward": False,
        "reason": "最大探索回数内に条件を満たす赤ボールを検出できませんでした",
        "steps": len(history),
        "last_distance_m": last_distance_m,
        "last_red_result": last_red_result,
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
