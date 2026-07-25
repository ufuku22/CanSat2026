import time
from typing import Any, Optional

from config import LidarForwardConfig as LidarConfig, RedConeConfig
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


def _red_cone_forward_duration(red_ratio, default_duration_s, duration_table):
    """赤色の大きさに応じて前進時間を選ぶ。"""
    red_ratio = float(red_ratio)
    for threshold, duration_s in duration_table:
        if red_ratio > threshold:
            return duration_s
    return default_duration_s


def guide_to_red_cone(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
) -> dict[str, Any]:
    """NavigationControllerを使って赤コーンを探し、正面へ回頭して前進する。"""
    processor = ImageProcessor()
    red_cone_config = RedConeConfig()
    forward_duration_by_red_ratio = tuple(
        sorted(
            red_cone_config.FORWARD_DURATION_BY_RED_RATIO,
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
        "reason": "最大試行回数内にゴール判定できませんでした",
        "steps": red_cone_config.MAX_GUIDANCE_STEPS,
        "history": history,
        "last_goal_result": last_goal_result,
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
