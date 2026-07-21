import time
from typing import Any, Optional

from image_processor import ImageProcessor
from navigation_controller import NavigationController
from sensor_manager import SensorManager


class GoalNavigator:
    """赤色画像と距離センサを使用してボールへ接近する。"""

    DEFAULT_RED_RATIO_THRESHOLD = NavigationController.RED_CONE_RED_THRESHOLD
    DEFAULT_RED_BLOCK_THRESHOLD = (
        NavigationController.RED_CONE_RED_BLOCK_THRESHOLD
    )
    DEFAULT_RED_SCAN_ANGLE_DEG = NavigationController.RED_CONE_SCAN_ANGLE_DEG
    DEFAULT_RED_SCAN_STEPS = NavigationController.RED_CONE_MAX_SCAN_STEPS
    DEFAULT_CAMERA_FOV_DEG = NavigationController.RED_CONE_CAMERA_FOV_DEG
    DEFAULT_CENTER_RED_RATIO_THRESHOLD = 0.01
    DEFAULT_ROTATION_SPEED = NavigationController.RED_CONE_ROTATE_SPEED
    DEFAULT_TURN_TOLERANCE_DEG = (
        NavigationController.RED_CONE_ROTATE_TOLERANCE_DEG
    )
    DEFAULT_DISTANCE_SCAN_ANGLE_DEG = 10.0
    DEFAULT_DISTANCE_SCAN_STEPS = 36
    DEFAULT_TARGET_DISTANCE_M = 2.0
    DEFAULT_FORWARD_STOP_DISTANCE_M = 0.5
    DEFAULT_FORWARD_SPEED = 60.0
    DEFAULT_LOOP_INTERVAL_S = 0.01
    DEFAULT_MEASUREMENT_PAUSE_S = 0.3

    def detect_ball(
        self,
        driver: Any,
        sensor_manager: SensorManager,
        *,
        red_ratio_threshold: float = DEFAULT_RED_RATIO_THRESHOLD,
        red_block_threshold: float = DEFAULT_RED_BLOCK_THRESHOLD,
        red_scan_angle_deg: float = DEFAULT_RED_SCAN_ANGLE_DEG,
        red_scan_steps: int = DEFAULT_RED_SCAN_STEPS,
        camera_fov_deg: float = DEFAULT_CAMERA_FOV_DEG,
        center_red_ratio_threshold: float = DEFAULT_CENTER_RED_RATIO_THRESHOLD,
        distance_scan_angle_deg: float = DEFAULT_DISTANCE_SCAN_ANGLE_DEG,
        distance_scan_steps: int = DEFAULT_DISTANCE_SCAN_STEPS,
        target_distance_m: float = DEFAULT_TARGET_DISTANCE_M,
        clockwise: bool = True,
        rotation_speed: float = DEFAULT_ROTATION_SPEED,
        rotation_tolerance_deg: float = DEFAULT_TURN_TOLERANCE_DEG,
        timeout_s: Optional[float] = None,
        measurement_pause_s: float = DEFAULT_MEASUREMENT_PAUSE_S,
        loop_interval_s: float = DEFAULT_LOOP_INTERVAL_S,
        forward_stop_distance_m: float = DEFAULT_FORWARD_STOP_DISTANCE_M,
        forward_speed: float = DEFAULT_FORWARD_SPEED,
        image_processor: Optional[ImageProcessor] = None,
    ) -> dict[str, Any]:
        """赤色を探し、2m以内の物体がある方向からボールへ接近する。

        赤色探索は ``NavigationController._find_red_cone_in_view()`` と同じ
        流れを使用する。赤色方向へ回頭後に画像を撮り直し、中央ブロックの
        赤色割合が1%を超えた場合は、10度ずつ旋回しながら距離を測定する。
        1%以下の場合は元の赤コーン誘導と同じ ``follow_forward()`` で
        時間指定の直進を行い、赤色探索から繰り返す。2m以内の値を取得した
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

        processor = image_processor or ImageProcessor()
        navigation_controller = NavigationController()
        navigation_controller.RED_CONE_RED_THRESHOLD = red_ratio_threshold
        navigation_controller.RED_CONE_RED_BLOCK_THRESHOLD = red_block_threshold
        navigation_controller.RED_CONE_SCAN_ANGLE_DEG = red_scan_angle_deg
        navigation_controller.RED_CONE_MAX_SCAN_STEPS = red_scan_steps
        navigation_controller.RED_CONE_ROTATE_SPEED = rotation_speed
        navigation_controller.RED_CONE_ROTATE_TOLERANCE_DEG = (
            rotation_tolerance_deg
        )
        navigation_controller.RED_CONE_ROTATE_TIMEOUT_S = timeout_s

        direction_sign = 1.0 if clockwise else -1.0
        distance_history: list[dict[str, Optional[float]]] = []

        try:
            red_guidance_history = []
            forward_duration_by_red_ratio = tuple(
                sorted(
                    navigation_controller.RED_CONE_FORWARD_DURATION_BY_RED_RATIO,
                    reverse=True,
                )
            )

            for red_step in range(navigation_controller.RED_CONE_MAX_STEPS):
                _, red_result, red_scan_history = (
                    navigation_controller._find_red_cone_in_view(
                        driver,
                        sensor_manager,
                        processor,
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

                turn_angle = navigation_controller._red_direction_to_turn_angle(
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
                    width=navigation_controller.CAPTURE_WIDTH,
                    height=navigation_controller.CAPTURE_HEIGHT,
                    hdr=navigation_controller.CAPTURE_HDR,
                    timeout_ms=navigation_controller.CAPTURE_TIMEOUT_MS,
                )
                center_red_result = processor.detect_color(
                    center_frame,
                    hsv_ranges=processor.RED_HSV_RANGES,
                    color_threshold=red_ratio_threshold,
                    block_threshold=red_block_threshold,
                )
                center_red_ratio = float(
                    center_red_result["color_block_ratios"][2]
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

                forward_duration = (
                    navigation_controller._red_cone_forward_duration(
                        red_ratio,
                        navigation_controller.RED_CONE_FORWARD_DURATION_S,
                        forward_duration_by_red_ratio,
                    )
                )
                print(
                    "中央の赤色割合が1%以下のため、"
                    f"follow_forwardで{forward_duration:.2f}秒直進します"
                )
                navigation_controller.follow_forward(
                    driver,
                    sensor_manager,
                    forward_duration,
                    base_speed=navigation_controller.RED_CONE_FORWARD_SPEED,
                    loop_interval=navigation_controller.RED_CONE_LOOP_INTERVAL,
                    stop_ramp_steps=(
                        navigation_controller.RED_CONE_STOP_RAMP_STEPS
                    ),
                    stop_ramp_interval=(
                        navigation_controller.RED_CONE_STOP_RAMP_INTERVAL
                    ),
                )
                red_guidance_history.append(
                    {
                        "step": red_step + 1,
                        "red_result": red_result,
                        "center_red_result": center_red_result,
                        "turn_result": turn_result,
                        "forward_duration_s": forward_duration,
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
        base_speed: float = DEFAULT_FORWARD_SPEED,
        loop_interval_s: float = DEFAULT_LOOP_INTERVAL_S,
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
                    navigation_controller._drive_pd_toward_heading(
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
