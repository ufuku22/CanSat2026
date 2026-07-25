import time
from typing import Any, Optional

from config import CameraCaptureConfig, GoalNavigatorConfig as GoalConfig, RedConeConfig
from image_processor import ImageProcessor
from navigation_controller import NavigationController
from sensor_manager import SensorManager


def _find_red_cone_in_view(
    navigation_controller: NavigationController,
    driver: Any,
    sensor_manager: SensorManager,
    processor: ImageProcessor,
    red_cone_config: RedConeConfig,
    camera_config: CameraCaptureConfig,
):
    """カメラ画像内に赤コーンが入るまで、基礎旋回を使って探索する。"""
    scan_history = []
    for scan_index in range(red_cone_config.MAX_SCAN_STEPS):
        print(
            "赤コーン探索: "
            f"scan {scan_index + 1}/{red_cone_config.MAX_SCAN_STEPS} 撮影します"
        )
        frame = sensor_manager.capture_front_frame(
            width=camera_config.WIDTH,
            height=camera_config.HEIGHT,
            hdr=camera_config.HDR,
            timeout_ms=camera_config.TIMEOUT_MS,
        )
        red_result = processor.detect_color(
            frame,
            hsv_ranges=processor.RED_HSV_RANGES,
            color_threshold=red_cone_config.RED_THRESHOLD,
            block_threshold=red_cone_config.RED_BLOCK_THRESHOLD,
        )
        scan_history.append({
            "scan_index": scan_index,
            "red_result": red_result,
        })
        print(
            "赤コーン探索: "
            f"total={red_result['total_color_ratio'] * 100:.2f}% "
            f"direction={red_result['color_direction']} "
            f"detected={red_result['is_color_detected']}"
        )

        if float(red_result["total_color_ratio"]) > red_cone_config.RED_THRESHOLD:
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


def _red_direction_to_turn_angle(red_direction, camera_fov_deg):
    """赤コーンの画面内位置を旋回角度に変換する。"""
    direction_offsets = {
        "left_far": -2,
        "left": -1,
        "center": 0,
        "right": 1,
        "right_far": 2,
    }
    if red_direction not in direction_offsets:
        return 0.0
    block_angle_deg = float(camera_fov_deg) / 5.0
    return direction_offsets[red_direction] * block_angle_deg


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
    image_processor: Optional[ImageProcessor] = None,
    *,
    red_cone_config: Optional[RedConeConfig] = None,
    camera_config: Optional[CameraCaptureConfig] = None,
) -> dict[str, Any]:
    """NavigationControllerを使って赤コーンを探し、正面へ回頭して前進する。"""
    processor = ImageProcessor() if image_processor is None else image_processor
    red_cone_config = RedConeConfig() if red_cone_config is None else red_cone_config
    camera_config = CameraCaptureConfig() if camera_config is None else camera_config
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
                camera_config,
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
        turn_angle = _red_direction_to_turn_angle(
            red_result["color_direction"],
            red_cone_config.CAMERA_FOV_DEG,
        )
        turn_result = None
        if turn_angle != 0.0:
            print(f"赤コーン誘導: {turn_angle:.1f}度旋回します")
            turn_result = navigation_controller.rotate_by_angle(
                driver,
                sensor_manager,
                turn_angle,
                speed=red_cone_config.ROTATE_SPEED,
                tolerance_deg=red_cone_config.ROTATE_TOLERANCE_DEG,
                timeout_s=red_cone_config.ROTATE_TIMEOUT_S,
            )
        else:
            print("赤コーン誘導: 旋回なし")

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
            f"direction={red_result['color_direction']})"
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
        goal_frame = sensor_manager.capture_front_frame(
            width=camera_config.WIDTH,
            height=camera_config.HEIGHT,
            hdr=camera_config.HDR,
            timeout_ms=camera_config.TIMEOUT_MS,
        )
        last_goal_result = processor.judge_red_goal_reached(
            goal_frame,
            red_threshold=red_cone_config.RED_THRESHOLD,
            goal_center_threshold=red_cone_config.GOAL_CENTER_THRESHOLD,
        )
        print(
            "赤コーン誘導: "
            f"ゴール判定 reached={last_goal_result['goal_reached']} "
            f"total={last_goal_result['total_color_ratio'] * 100:.2f}% "
            f"center={last_goal_result['center_block_color_ratio'] * 100:.2f}%"
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


class GoalNavigator:
    """赤色画像と距離センサを使用してボールへ接近する。"""

    def detect_ball(
        self,
        driver: Any,
        sensor_manager: SensorManager,
        *,
        red_ratio_threshold: float = GoalConfig.RED_RATIO_THRESHOLD,
        red_block_threshold: float = GoalConfig.RED_BLOCK_THRESHOLD,
        red_scan_angle_deg: float = GoalConfig.RED_SCAN_ANGLE_DEG,
        red_scan_steps: int = GoalConfig.RED_SCAN_STEPS,
        camera_fov_deg: float = GoalConfig.CAMERA_FOV_DEG,
        center_red_ratio_threshold: float = (
            GoalConfig.CENTER_RED_RATIO_THRESHOLD
        ),
        distance_scan_angle_deg: float = (
            GoalConfig.DISTANCE_SCAN_ANGLE_DEG
        ),
        distance_scan_steps: int = GoalConfig.DISTANCE_SCAN_STEPS,
        target_distance_m: float = GoalConfig.TARGET_DISTANCE_M,
        clockwise: bool = GoalConfig.CLOCKWISE,
        rotation_speed: float = GoalConfig.ROTATION_SPEED,
        rotation_tolerance_deg: float = GoalConfig.TURN_TOLERANCE_DEG,
        timeout_s: Optional[float] = GoalConfig.ROTATION_TIMEOUT_S,
        measurement_pause_s: float = GoalConfig.MEASUREMENT_PAUSE_S,
        loop_interval_s: float = GoalConfig.LOOP_INTERVAL_S,
        forward_stop_distance_m: float = (
            GoalConfig.FORWARD_STOP_DISTANCE_M
        ),
        forward_speed: float = GoalConfig.FORWARD_SPEED,
        follow_forward_duration_s: float = (
            GoalConfig.FOLLOW_FORWARD_DURATION_S
        ),
        image_processor: Optional[ImageProcessor] = None,
    ) -> dict[str, Any]:
        """赤色を探し、2m以内の物体がある方向からボールへ接近する。

        赤色探索にはこのモジュールの ``_find_red_cone_in_view()``、
        中央割合判定には ``ImageProcessor.judge_red_goal_reached()`` を使い、
        赤コーン誘導と同じ撮影・赤検知処理を使用する。赤色方向へ回頭後に
        画像を撮り直し、中央ブロックの赤色割合が1%を超えた場合は、
        10度ずつ旋回しながら距離を測定する。
        1%以下の場合は ``follow_forward()`` で1秒間直進し、赤色探索から
        繰り返す。2m以内の値を取得した
        時点で旋回を止め、``rider_forward()`` で指定停止距離まで直進する。
        """
        red_ratio_threshold = float(red_ratio_threshold)
        red_block_threshold = float(red_block_threshold)
        red_scan_angle_deg = float(red_scan_angle_deg)
        red_scan_steps = int(red_scan_steps)
        camera_fov_deg = float(camera_fov_deg)
        center_red_ratio_threshold = float(center_red_ratio_threshold)
        distance_scan_angle_deg = float(distance_scan_angle_deg)
        distance_scan_steps = int(distance_scan_steps)
        target_distance_m = float(target_distance_m)
        rotation_speed = float(rotation_speed)
        rotation_tolerance_deg = float(rotation_tolerance_deg)
        if timeout_s is not None:
            timeout_s = float(timeout_s)
        measurement_pause_s = float(measurement_pause_s)
        loop_interval_s = float(loop_interval_s)
        forward_stop_distance_m = float(forward_stop_distance_m)
        forward_speed = float(forward_speed)
        follow_forward_duration_s = float(follow_forward_duration_s)

        if not 0.0 <= red_ratio_threshold <= 1.0:
            raise ValueError("red_ratio_threshold must be in the range 0 to 1")
        if not 0.0 <= red_block_threshold <= 1.0:
            raise ValueError("red_block_threshold must be in the range 0 to 1")
        if red_scan_angle_deg <= 0.0:
            raise ValueError("red_scan_angle_deg must be greater than 0")
        if red_scan_steps < 1:
            raise ValueError("red_scan_steps must be 1 or greater")
        if camera_fov_deg <= 0.0:
            raise ValueError("camera_fov_deg must be greater than 0")
        if not 0.0 <= center_red_ratio_threshold <= 1.0:
            raise ValueError(
                "center_red_ratio_threshold must be in the range 0 to 1"
            )
        if distance_scan_angle_deg <= 0.0:
            raise ValueError("distance_scan_angle_deg must be greater than 0")
        if distance_scan_steps < 1:
            raise ValueError("distance_scan_steps must be 1 or greater")
        if target_distance_m <= 0.0:
            raise ValueError("target_distance_m must be greater than 0")
        if not 0.0 < rotation_speed <= 100.0:
            raise ValueError("rotation_speed must be in the range 0 to 100")
        if rotation_tolerance_deg < 0.0:
            raise ValueError("rotation_tolerance_deg must be 0 or greater")
        if timeout_s is not None and timeout_s <= 0.0:
            raise ValueError("timeout_s must be greater than 0")
        if measurement_pause_s < 0.0:
            raise ValueError("measurement_pause_s must be 0 or greater")
        if loop_interval_s <= 0.0:
            raise ValueError("loop_interval_s must be greater than 0")
        if forward_stop_distance_m < 0.0:
            raise ValueError("forward_stop_distance_m must be 0 or greater")
        if not 0.0 < forward_speed <= 100.0:
            raise ValueError("forward_speed must be in the range 0 to 100")
        if follow_forward_duration_s <= 0.0:
            raise ValueError("follow_forward_duration_s must be greater than 0")

        if image_processor is None:
            image_processor = ImageProcessor()
        navigation_controller = NavigationController()
        red_cone_config = RedConeConfig()
        camera_config = CameraCaptureConfig()
        red_cone_config.RED_THRESHOLD = red_ratio_threshold
        red_cone_config.RED_BLOCK_THRESHOLD = red_block_threshold
        red_cone_config.SCAN_ANGLE_DEG = red_scan_angle_deg
        red_cone_config.MAX_SCAN_STEPS = red_scan_steps
        red_cone_config.ROTATE_SPEED = rotation_speed
        red_cone_config.ROTATE_TOLERANCE_DEG = rotation_tolerance_deg
        red_cone_config.ROTATE_TIMEOUT_S = timeout_s

        direction_sign = 1.0 if clockwise else -1.0
        distance_history: list[dict[str, Optional[float]]] = []

        try:
            red_guidance_history = []
            for red_step in range(red_cone_config.MAX_GUIDANCE_STEPS):
                _, red_result, red_scan_history = (
                    _find_red_cone_in_view(
                        navigation_controller,
                        driver,
                        sensor_manager,
                        image_processor,
                        red_cone_config,
                        camera_config,
                    )
                )

                if red_result is None:
                    driver.stop()
                    return {
                        "ball_detected": False,
                        "reason": "赤色を検知できませんでした",
                        "red_ratio": 0.0,
                        "red_result": None,
                        "red_scan_history": red_scan_history,
                        "red_guidance_history": red_guidance_history,
                        "distance_history": distance_history,
                        "detected_distance_m": None,
                        "stop_distance_m": None,
                    }

                red_ratio = float(red_result["total_color_ratio"])
                red_direction = str(red_result["color_direction"])
                print(
                    f"赤色検知: 割合={red_ratio * 100:.2f}%, "
                    f"方向={red_direction}"
                )

                turn_angle = _red_direction_to_turn_angle(
                    red_direction,
                    camera_fov_deg,
                )
                turn_result = navigation_controller.rotate_by_angle(
                    driver,
                    sensor_manager,
                    turn_angle,
                    speed=rotation_speed,
                    tolerance_deg=rotation_tolerance_deg,
                    timeout_s=timeout_s,
                    loop_interval=loop_interval_s,
                )
                if not turn_result["reached"]:
                    return {
                        "ball_detected": False,
                        "reason": "赤色方向へ旋回できませんでした",
                        "red_ratio": red_ratio,
                        "red_direction": red_direction,
                        "red_result": red_result,
                        "red_scan_history": red_scan_history,
                        "red_guidance_history": red_guidance_history,
                        "turn_result": turn_result,
                        "distance_history": distance_history,
                        "detected_distance_m": None,
                        "stop_distance_m": None,
                    }

                time.sleep(measurement_pause_s)
                print("赤色方向へ回頭後、中央の赤色割合を確認します")
                center_frame = sensor_manager.capture_front_frame(
                    width=camera_config.WIDTH,
                    height=camera_config.HEIGHT,
                    hdr=camera_config.HDR,
                    timeout_ms=camera_config.TIMEOUT_MS,
                )
                center_red_result = image_processor.judge_red_goal_reached(
                    center_frame,
                    red_threshold=red_cone_config.RED_THRESHOLD,
                    goal_center_threshold=center_red_ratio_threshold,
                )
                center_red_ratio = float(
                    center_red_result["center_block_color_ratio"]
                )
                print(
                    f"画面中央の赤色割合={center_red_ratio * 100:.2f}% "
                    f"(距離探索条件: "
                    f"{center_red_ratio_threshold * 100:.2f}%超)"
                )

                if center_red_ratio > center_red_ratio_threshold:
                    red_guidance_history.append(
                        {
                            "step": red_step + 1,
                            "red_result": red_result,
                            "center_red_result": center_red_result,
                            "turn_result": turn_result,
                            "forward_duration_s": None,
                        }
                    )
                    break

                print(
                    "中央の赤色割合が1%以下のため、"
                    f"follow_forwardで{follow_forward_duration_s:.2f}秒直進します"
                )
                navigation_controller.follow_forward(
                    driver,
                    sensor_manager,
                    follow_forward_duration_s,
                    base_speed=red_cone_config.FORWARD_SPEED,
                    loop_interval=red_cone_config.LOOP_INTERVAL_S,
                )
                red_guidance_history.append(
                    {
                        "step": red_step + 1,
                        "red_result": red_result,
                        "center_red_result": center_red_result,
                        "turn_result": turn_result,
                        "forward_duration_s": follow_forward_duration_s,
                    }
                )
            else:
                return {
                    "ball_detected": False,
                    "reason": "中央の赤色割合が1%を超えませんでした",
                    "red_ratio": red_ratio,
                    "center_red_ratio": center_red_ratio,
                    "red_direction": red_direction,
                    "red_result": red_result,
                    "center_red_result": center_red_result,
                    "red_scan_history": red_scan_history,
                    "red_guidance_history": red_guidance_history,
                    "turn_result": turn_result,
                    "distance_history": distance_history,
                    "detected_distance_m": None,
                    "stop_distance_m": None,
                }

            detected_distance = None
            detected_heading = None
            for scan_index in range(distance_scan_steps + 1):
                if scan_index > 0:
                    scan_turn_result = navigation_controller.rotate_by_angle(
                        driver,
                        sensor_manager,
                        direction_sign * distance_scan_angle_deg,
                        speed=rotation_speed,
                        tolerance_deg=min(
                            rotation_tolerance_deg,
                            distance_scan_angle_deg / 4.0,
                        ),
                        timeout_s=timeout_s,
                        loop_interval=loop_interval_s,
                    )
                    if not scan_turn_result["reached"]:
                        return {
                            "ball_detected": False,
                            "reason": "距離探索中に旋回できませんでした",
                            "red_ratio": red_ratio,
                            "center_red_ratio": center_red_ratio,
                            "red_direction": red_direction,
                            "red_result": red_result,
                            "center_red_result": center_red_result,
                            "red_scan_history": red_scan_history,
                            "red_guidance_history": red_guidance_history,
                            "turn_result": scan_turn_result,
                            "distance_history": distance_history,
                            "detected_distance_m": None,
                            "stop_distance_m": None,
                        }

                time.sleep(measurement_pause_s)
                heading = float(sensor_manager.get_heading_deg())
                measured_distance = sensor_manager.get_distance_m()
                distance = (
                    None
                    if measured_distance is None
                    else float(measured_distance)
                )
                distance_history.append(
                    {
                        "scan_index": float(scan_index),
                        "heading_deg": heading,
                        "distance_m": distance,
                    }
                )
                distance_text = (
                    "測定不能" if distance is None else f"{distance:.3f} m"
                )
                print(
                    f"距離探索{scan_index + 1:02d}: "
                    f"方位={heading:.1f} deg, 距離={distance_text}"
                )

                if distance is not None and 0.0 < distance <= target_distance_m:
                    detected_distance = distance
                    detected_heading = heading
                    print(
                        f"{target_distance_m:.1f} m以内を検知したため"
                        "旋回を停止します"
                    )
                    break

            if detected_distance is None:
                return {
                    "ball_detected": False,
                    "reason": (
                        f"{target_distance_m:.1f} m以内の距離を"
                        "検知できませんでした"
                    ),
                    "red_ratio": red_ratio,
                    "center_red_ratio": center_red_ratio,
                    "red_direction": red_direction,
                    "red_result": red_result,
                    "center_red_result": center_red_result,
                    "red_scan_history": red_scan_history,
                    "red_guidance_history": red_guidance_history,
                    "turn_result": turn_result,
                    "distance_history": distance_history,
                    "detected_distance_m": None,
                    "stop_distance_m": None,
                }

            print(
                f"検知方位={detected_heading:.1f} degから、"
                f"{forward_stop_distance_m:.3f} m以下まで直進します"
            )
            stop_distance = self.rider_forward(
                driver,
                sensor_manager,
                forward_stop_distance_m,
                base_speed=forward_speed,
                loop_interval_s=loop_interval_s,
            )
            forward_completed = stop_distance is not None

            return {
                "ball_detected": forward_completed,
                "reason": (
                    "ボール検知成功"
                    if forward_completed
                    else "rider_forwardで停止距離を取得できませんでした"
                ),
                "red_ratio": red_ratio,
                "center_red_ratio": center_red_ratio,
                "red_direction": red_direction,
                "red_result": red_result,
                "center_red_result": center_red_result,
                "red_scan_history": red_scan_history,
                "red_guidance_history": red_guidance_history,
                "turn_result": turn_result,
                "distance_history": distance_history,
                "detected_distance_m": detected_distance,
                "detected_heading_deg": detected_heading,
                "forward_completed": forward_completed,
                "stop_distance_m": stop_distance,
            }
        finally:
            driver.stop()

    def rider_forward(
        self,
        driver: Any,
        sensor_manager: SensorManager,
        distance_threshold_m: float,
        *,
        base_speed: float = GoalConfig.FORWARD_SPEED,
        loop_interval_s: float = GoalConfig.LOOP_INTERVAL_S,
    ) -> Optional[float]:
        """開始時の方位を保ち、距離が閾値以下になるまでPD制御で直進する。"""
        distance_threshold_m = float(distance_threshold_m)
        base_speed = float(base_speed)
        loop_interval_s = float(loop_interval_s)

        distance = sensor_manager.get_distance_m()
        target_heading = float(sensor_manager.get_heading_deg())

        if distance is None:
            driver.stop()
            print("距離を測定できませんでした")
            return None

        distance = float(distance)
        print(
            f"直進開始: 距離={distance:.3f} m, "
            f"保持方位={target_heading:.1f} deg"
        )

        navigation_controller = NavigationController()
        previous_error = 0.0

        try:
            while distance > distance_threshold_m:
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

        stop_distance = sensor_manager.get_distance_m()
        if stop_distance is None:
            print("停止後の距離を測定できませんでした")
            return None

        stop_distance = float(stop_distance)
        print(f"停止距離={stop_distance:.3f} m")
        return stop_distance
