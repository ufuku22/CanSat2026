import time
from typing import Any, Optional

from image_processor import ImageProcessor
from navigation_controller import NavigationController
from sensor_manager import SensorManager


class GoalNavigator:
    """正方形状に配置されたゴールマーカーを探索するための制御クラス。"""

    DEFAULT_ROTATION_ANGLE_DEG = 60.0
    DEFAULT_ROTATION_SPEED = 30.0
    DEFAULT_LOOP_INTERVAL_S = 0.01
    DEFAULT_MEASUREMENT_PAUSE_S = 0.3
    DEFAULT_BALL_DISTANCE_LOWER_THRESHOLD_M = 0.20
    DEFAULT_BALL_DISTANCE_UPPER_THRESHOLD_M = 5.00
    DEFAULT_TURN_TOLERANCE_DEG = 1.0
    DEFAULT_FORWARD_SPEED = 60.0
    DEFAULT_FORWARD_STOP_DISTANCE_M = 0.5
    DEFAULT_RED_RATIO_THRESHOLD = 0.001
    DEFAULT_RED_ALIGN_NEAR_ANGLE_DEG = 5.0
    DEFAULT_RED_ALIGN_FAR_ANGLE_DEG = 10.0
    DEFAULT_DISTANCE_SWEEP_STEP_DEG = 3.0
    DEFAULT_DISTANCE_SWEEP_STEPS = 2
    DEFAULT_SENSOR_ALIGNMENT_OFFSET_DEG = 0.0

    def detect_ball(
        self,
        driver: Any,
        sensor_manager: SensorManager,
        lower_threshold_m: float = DEFAULT_BALL_DISTANCE_LOWER_THRESHOLD_M,
        upper_threshold_m: float = DEFAULT_BALL_DISTANCE_UPPER_THRESHOLD_M,
        *,
        rotation_angle_deg: float = DEFAULT_ROTATION_ANGLE_DEG,
        rotation_speed: float = DEFAULT_ROTATION_SPEED,
        rotation_tolerance_deg: float = DEFAULT_TURN_TOLERANCE_DEG,
        clockwise: bool = True,
        timeout_s: Optional[float] = None,
        loop_interval_s: float = DEFAULT_LOOP_INTERVAL_S,
        measurement_pause_s: float = DEFAULT_MEASUREMENT_PAUSE_S,
        forward_speed: float = DEFAULT_FORWARD_SPEED,
        forward_stop_distance_m: float = DEFAULT_FORWARD_STOP_DISTANCE_M,
        red_ratio_threshold: float = DEFAULT_RED_RATIO_THRESHOLD,
        red_align_near_angle_deg: float = DEFAULT_RED_ALIGN_NEAR_ANGLE_DEG,
        red_align_far_angle_deg: float = DEFAULT_RED_ALIGN_FAR_ANGLE_DEG,
        distance_sweep_step_deg: float = DEFAULT_DISTANCE_SWEEP_STEP_DEG,
        distance_sweep_steps: int = DEFAULT_DISTANCE_SWEEP_STEPS,
        sensor_alignment_offset_deg: float = DEFAULT_SENSOR_ALIGNMENT_OFFSET_DEG,
        image_processor: Optional[ImageProcessor] = None,
    ) -> dict[str, Any]:
        """5分割画像で赤色方向を探し、距離測定と接近を繰り返す。

        最初に正面画像を撮影し、``ImageProcessor.detect_color()`` が返す
        5区画の赤色割合を使用する。赤色が中央区画に来るまで5度または
        10度ずつ旋回と再撮影を繰り返してから距離を測定する。中央でも
        距離を取得できない場合は、中央を基準に左右3度、6度の順で距離を
        再測定する。距離取得後は専用の前進停止距離以下まで
        ``rider_forward()`` で直進し、停止距離が判定下限以上かつ
        判定上限以下なら成功とする。
        """
        lower_threshold_m = float(lower_threshold_m)
        upper_threshold_m = float(upper_threshold_m)
        rotation_angle_deg = float(rotation_angle_deg)
        rotation_speed = float(rotation_speed)
        rotation_tolerance_deg = float(rotation_tolerance_deg)
        if timeout_s is not None:
            timeout_s = float(timeout_s)
        loop_interval_s = float(loop_interval_s)
        measurement_pause_s = float(measurement_pause_s)
        forward_speed = float(forward_speed)
        forward_stop_distance_m = float(forward_stop_distance_m)
        red_ratio_threshold = float(red_ratio_threshold)
        red_align_near_angle_deg = float(red_align_near_angle_deg)
        red_align_far_angle_deg = float(red_align_far_angle_deg)
        distance_sweep_step_deg = float(distance_sweep_step_deg)
        distance_sweep_steps = int(distance_sweep_steps)
        sensor_alignment_offset_deg = float(sensor_alignment_offset_deg)

        if lower_threshold_m < 0.0:
            raise ValueError("lower_threshold_m must be 0 or greater")
        if upper_threshold_m < lower_threshold_m:
            raise ValueError(
                "upper_threshold_m must be greater than or equal to "
                "lower_threshold_m"
            )
        if rotation_angle_deg <= 0.0:
            raise ValueError("rotation_angle_deg must be greater than 0")
        if not 0.0 < rotation_speed <= 100.0:
            raise ValueError("rotation_speed must be in the range 0 to 100")
        if rotation_tolerance_deg < 0.0:
            raise ValueError("rotation_tolerance_deg must be 0 or greater")
        if timeout_s is not None and timeout_s <= 0.0:
            raise ValueError("timeout_s must be greater than 0")
        if loop_interval_s <= 0.0:
            raise ValueError("loop_interval_s must be greater than 0")
        if measurement_pause_s < 0.0:
            raise ValueError("measurement_pause_s must be 0 or greater")
        if not 0.0 < forward_speed <= 100.0:
            raise ValueError("forward_speed must be in the range 0 to 100")
        if forward_stop_distance_m < 0.0:
            raise ValueError("forward_stop_distance_m must be 0 or greater")
        if not 0.0 <= red_ratio_threshold <= 1.0:
            raise ValueError("red_ratio_threshold must be in the range 0 to 1")
        if red_align_near_angle_deg <= 0.0:
            raise ValueError("red_align_near_angle_deg must be greater than 0")
        if red_align_far_angle_deg <= 0.0:
            raise ValueError("red_align_far_angle_deg must be greater than 0")
        if distance_sweep_step_deg <= 0.0:
            raise ValueError("distance_sweep_step_deg must be greater than 0")
        if distance_sweep_steps < 1:
            raise ValueError("distance_sweep_steps must be 1 or greater")

        processor = image_processor or ImageProcessor()
        navigation_controller = NavigationController()
        direction_sign = 1.0 if clockwise else -1.0
        attempt = 0
        rotation_count = 0

        try:
            while True:
                attempt += 1
                distance = None
                stop_distance = None
                distance_sweep_offset = 0.0
                alignment_steps = 0

                red_result = self._capture_red_result(
                    sensor_manager,
                    processor,
                    red_ratio_threshold,
                    label=f"探索{attempt:02d}",
                )
                red_detected = bool(red_result["is_color_detected"])
                red_direction = str(red_result["color_direction"])

                alignment_angles = {
                    "left_far": -red_align_far_angle_deg,
                    "left": -red_align_near_angle_deg,
                    "right": red_align_near_angle_deg,
                    "right_far": red_align_far_angle_deg,
                }

                while red_detected and red_direction != "center":
                    alignment_turn_angle = alignment_angles.get(red_direction)
                    if alignment_turn_angle is None:
                        red_detected = False
                        break

                    print(
                        f"赤色方向={red_direction}: "
                        f"{alignment_turn_angle:.1f} deg旋回します"
                    )
                    alignment_result = navigation_controller.rotate_by_angle(
                        driver,
                        sensor_manager,
                        alignment_turn_angle,
                        speed=rotation_speed,
                        tolerance_deg=rotation_tolerance_deg,
                        timeout_s=timeout_s,
                        loop_interval=loop_interval_s,
                    )
                    if not alignment_result["reached"]:
                        print("赤いボールを中央へ合わせられませんでした")
                        return {
                            "ball_detected": False,
                            "attempts": attempt,
                            "rotation_count": rotation_count,
                            "distance_m": None,
                            "stop_distance_m": None,
                            "red_ratio": float(red_result["total_color_ratio"]),
                            "red_direction": red_direction,
                            "red_result": red_result,
                            "turn_result": alignment_result,
                        }

                    alignment_steps += 1
                    time.sleep(measurement_pause_s)
                    red_result = self._capture_red_result(
                        sensor_manager,
                        processor,
                        red_ratio_threshold,
                        label=f"中央合わせ{alignment_steps:02d}",
                    )
                    red_detected = bool(red_result["is_color_detected"])
                    red_direction = str(red_result["color_direction"])

                red_ratio = float(red_result["total_color_ratio"])

                if red_detected and red_direction == "center":
                    print("赤いボールが画像中央に入りました")

                    if abs(sensor_alignment_offset_deg) > rotation_tolerance_deg:
                        print(
                            "カメラ・距離センサ間の取付角を補正します: "
                            f"{sensor_alignment_offset_deg:.1f} deg"
                        )
                        offset_result = navigation_controller.rotate_by_angle(
                            driver,
                            sensor_manager,
                            sensor_alignment_offset_deg,
                            speed=rotation_speed,
                            tolerance_deg=rotation_tolerance_deg,
                            timeout_s=timeout_s,
                            loop_interval=loop_interval_s,
                        )
                        if not offset_result["reached"]:
                            print("距離センサの取付角を補正できませんでした")
                            return {
                                "ball_detected": False,
                                "attempts": attempt,
                                "rotation_count": rotation_count,
                                "distance_m": None,
                                "stop_distance_m": None,
                                "red_ratio": red_ratio,
                                "red_direction": red_direction,
                                "red_result": red_result,
                                "turn_result": offset_result,
                            }

                    time.sleep(measurement_pause_s)
                    measured_distance = sensor_manager.get_distance_m()

                    if measured_distance is None:
                        print(
                            "中央で距離を取得できないため、"
                            "距離センサを左右に微調整します"
                        )
                        current_sweep_offset = 0.0
                        sweep_offsets = []
                        for step in range(1, distance_sweep_steps + 1):
                            sweep_offsets.extend(
                                (
                                    distance_sweep_step_deg * step,
                                    -distance_sweep_step_deg * step,
                                )
                            )

                        for target_sweep_offset in sweep_offsets:
                            sweep_turn_angle = (
                                target_sweep_offset - current_sweep_offset
                            )
                            print(
                                "距離探索: 中央基準"
                                f"{target_sweep_offset:+.1f} degを測定します"
                            )
                            sweep_result = navigation_controller.rotate_by_angle(
                                driver,
                                sensor_manager,
                                sweep_turn_angle,
                                speed=rotation_speed,
                                tolerance_deg=rotation_tolerance_deg,
                                timeout_s=timeout_s,
                                loop_interval=loop_interval_s,
                            )
                            if not sweep_result["reached"]:
                                break

                            current_sweep_offset = target_sweep_offset
                            time.sleep(measurement_pause_s)
                            measured_distance = sensor_manager.get_distance_m()
                            if measured_distance is not None:
                                distance_sweep_offset = current_sweep_offset
                                break

                        if measured_distance is None and current_sweep_offset != 0.0:
                            navigation_controller.rotate_by_angle(
                                driver,
                                sensor_manager,
                                -current_sweep_offset,
                                speed=rotation_speed,
                                tolerance_deg=rotation_tolerance_deg,
                                timeout_s=timeout_s,
                                loop_interval=loop_interval_s,
                            )

                    if measured_distance is None:
                        print("微調整後も距離を測定できなかったため旋回します")
                    else:
                        distance = float(measured_distance)
                        print(
                            f"赤色方向の距離={distance:.3f} m "
                            f"(微調整={distance_sweep_offset:+.1f} deg)"
                        )
                        print(
                            f"距離が{forward_stop_distance_m:.3f} m以下になるまで"
                            "直進します"
                        )
                        stop_distance = self.rider_forward(
                            driver,
                            sensor_manager,
                            forward_stop_distance_m,
                            base_speed=forward_speed,
                            loop_interval_s=loop_interval_s,
                        )

                        if (
                            stop_distance is not None
                            and lower_threshold_m
                            <= stop_distance
                            <= upper_threshold_m
                        ):
                            print("ボール検知成功")
                            return {
                                "ball_detected": True,
                                "attempts": attempt,
                                "rotation_count": rotation_count,
                                "alignment_steps": alignment_steps,
                                "distance_m": distance,
                                "distance_sweep_offset_deg": (
                                    distance_sweep_offset
                                ),
                                "stop_distance_m": stop_distance,
                                "red_ratio": red_ratio,
                                "red_direction": red_direction,
                                "red_result": red_result,
                            }

                        if stop_distance is None:
                            print("停止距離を取得できなかったため旋回します")
                        else:
                            print(
                                "停止距離が閾値の範囲外です: "
                                f"{stop_distance:.3f} m "
                                f"(閾値={lower_threshold_m:.3f}～"
                                f"{upper_threshold_m:.3f} m)"
                            )
                else:
                    print("赤色を中央で検知できなかったため旋回します")

                time.sleep(measurement_pause_s)
                step_tolerance = min(
                    rotation_tolerance_deg,
                    rotation_angle_deg / 4.0,
                )
                turn_result = navigation_controller.rotate_by_angle(
                    driver,
                    sensor_manager,
                    direction_sign * rotation_angle_deg,
                    speed=rotation_speed,
                    tolerance_deg=step_tolerance,
                    timeout_s=timeout_s,
                    loop_interval=loop_interval_s,
                )
                rotation_count += 1

                if not turn_result["reached"]:
                    print("既定角度まで旋回できませんでした")
                    return {
                        "ball_detected": False,
                        "attempts": attempt,
                        "rotation_count": rotation_count,
                        "distance_m": distance,
                        "stop_distance_m": stop_distance,
                        "red_ratio": red_ratio,
                        "red_result": red_result,
                        "turn_result": turn_result,
                    }
        finally:
            driver.stop()

    @staticmethod
    def _capture_red_result(
        sensor_manager: SensorManager,
        processor: ImageProcessor,
        red_ratio_threshold: float,
        *,
        label: str,
    ) -> dict[str, Any]:
        """正面画像を撮影し、既存の5分割赤色検知結果を返す。"""
        print(f"{label}: 正面画像を撮影します")
        frame = sensor_manager.capture_front_frame()
        red_result = processor.detect_color(
            frame,
            hsv_ranges=processor.RED_HSV_RANGES,
            color_threshold=red_ratio_threshold,
            block_threshold=red_ratio_threshold,
        )
        block_text = ", ".join(
            f"{float(ratio) * 100:.2f}%"
            for ratio in red_result["color_block_ratios"]
        )
        print(
            f"赤色占有率={float(red_result['total_color_ratio']) * 100:.2f}%, "
            f"5分割=[{block_text}], "
            f"最大方向={red_result['color_direction']}"
        )
        return red_result

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
