import time
from typing import Any, Optional

from image_processor import ImageProcessor
from navigation_controller import NavigationController
from sensor_manager import SensorManager


class GoalNavigator:
    """正方形状に配置されたゴールマーカーを探索するための制御クラス。"""

    DEFAULT_SCAN_ANGLE_DEG = 360.0
    DEFAULT_SAMPLE_INTERVAL_DEG = 30.0
    DEFAULT_ROTATION_SPEED = 30.0
    DEFAULT_LOOP_INTERVAL_S = 0.01
    DEFAULT_MEASUREMENT_PAUSE_S = 0.2
    DEFAULT_BALL_DISTANCE_LOWER_THRESHOLD_M = 0.20
    DEFAULT_BALL_DISTANCE_UPPER_THRESHOLD_M = 5.00
    DEFAULT_TURN_TOLERANCE_DEG = 3.0
    DEFAULT_TURN_TIMEOUT_S = 10.0
    DEFAULT_FORWARD_DURATION_S = 1.0
    DEFAULT_FORWARD_SPEED = 60.0
    DEFAULT_RED_RATIO_THRESHOLD = 0.001

    def __init__(self) -> None:
        self.scan_results: list[dict[str, Any]] = []
        self.last_scan_completed = False

    def detect_ball(
        self,
        driver: Any,
        sensor_manager: SensorManager,
        *,
        scan_angle_deg: float = DEFAULT_SCAN_ANGLE_DEG,
        sample_interval_deg: float = DEFAULT_SAMPLE_INTERVAL_DEG,
        rotation_speed: float = DEFAULT_ROTATION_SPEED,
        rotation_tolerance_deg: float = DEFAULT_TURN_TOLERANCE_DEG,
        clockwise: bool = True,
        timeout_s: Optional[float] = None,
        loop_interval_s: float = DEFAULT_LOOP_INTERVAL_S,
        measurement_pause_s: float = DEFAULT_MEASUREMENT_PAUSE_S,
    ) -> list[dict[str, Any]]:
        """一定角度ずつ旋回し、停止時の距離と9軸方位を保存する。

        旋回開始時を相対角度0度とし、``sample_interval_deg`` ごとに
        ``NavigationController.rotate_by_angle()`` で旋回してから、
        ``sensor_manager.get_distance_m()`` の値を取得する。方位には、
        BNO055の9軸センサ融合（NDOF）による値を返す
        ``sensor_manager.get_heading_deg()`` を使用する。測定結果は
        ``self.scan_results`` に保存するとともに、そのリストを返す。

        各測定結果の形式::

            {
                "relative_angle_deg": 20.4,  # 旋回開始からの角度
                "heading_deg": 135.2,        # IMUが返した絶対方位
                "distance_m": 1.234,         # 測定不能の場合はNone
                "elapsed_s": 0.82,
            }

        ``last_scan_completed`` は、指定角度まで旋回できた場合にTrueになる。
        ``timeout_s`` がNoneの場合は旋回完了まで待ち続ける。モーターは
        正常終了、割り込み、例外のいずれでも必ず停止する。
        """
        scan_angle_deg = float(scan_angle_deg)
        sample_interval_deg = float(sample_interval_deg)
        rotation_speed = float(rotation_speed)
        rotation_tolerance_deg = float(rotation_tolerance_deg)
        if timeout_s is not None:
            timeout_s = float(timeout_s)
        loop_interval_s = float(loop_interval_s)
        measurement_pause_s = float(measurement_pause_s)

        if scan_angle_deg <= 0.0:
            raise ValueError("scan_angle_deg must be greater than 0")
        if sample_interval_deg <= 0.0:
            raise ValueError("sample_interval_deg must be greater than 0")
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

        self.scan_results = []
        self.last_scan_completed = False

        start_time = time.monotonic()
        initial_heading = self._normalize_heading(sensor_manager.get_heading_deg())
        commanded_angle = 0.0
        rotated_angle = 0.0
        navigation_controller = NavigationController()

        # 旋回を始める前の正面方向も測定する。
        self._save_sample(
            sensor_manager,
            relative_angle_deg=rotated_angle,
            heading_deg=initial_heading,
            elapsed_s=0.0,
        )
        time.sleep(measurement_pause_s)

        try:
            while commanded_angle < scan_angle_deg:
                step_angle = min(
                    sample_interval_deg,
                    scan_angle_deg - commanded_angle,
                )
                next_commanded_angle = commanded_angle + step_angle
                direction_sign = 1.0 if clockwise else -1.0
                target_heading = self._normalize_heading(
                    initial_heading + direction_sign * next_commanded_angle
                )
                current_heading = self._normalize_heading(
                    sensor_manager.get_heading_deg()
                )
                turn_angle = self._heading_change_deg(
                    target_heading,
                    current_heading,
                )
                step_tolerance = min(
                    rotation_tolerance_deg,
                    step_angle / 4.0,
                )

                turn_result = navigation_controller.rotate_by_angle(
                    driver,
                    sensor_manager,
                    turn_angle,
                    speed=rotation_speed,
                    tolerance_deg=step_tolerance,
                    timeout_s=timeout_s,
                    loop_interval=loop_interval_s,
                )
                rotated_angle += abs(float(turn_result["rotated_angle_deg"]))

                if not turn_result["reached"]:
                    break

                commanded_angle = next_commanded_angle

                # 360度地点は開始方向と重複するため、保存しない。
                if commanded_angle < scan_angle_deg:
                    current_heading = self._normalize_heading(
                        sensor_manager.get_heading_deg()
                    )
                    self._save_sample(
                        sensor_manager,
                        relative_angle_deg=rotated_angle,
                        heading_deg=current_heading,
                        elapsed_s=time.monotonic() - start_time,
                    )
                    time.sleep(measurement_pause_s)

            self.last_scan_completed = commanded_angle >= scan_angle_deg
        finally:
            driver.stop()

        return self.scan_results

    def rider_forward(
        self,
        driver: Any,
        sensor_manager: SensorManager,
        distance_threshold_m: float,
        *,
        base_speed: float = DEFAULT_FORWARD_SPEED,
        loop_interval_s: float = DEFAULT_LOOP_INTERVAL_S,
    ) -> Optional[float]:
        """開始時の方位を保ち、距離が閾値を超えるまでPD制御で直進する。"""
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
            while distance <= distance_threshold_m:
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
                    return None
                distance = float(measured_distance)
        finally:
            driver.stop()

        print(f"距離閾値超過で停止: {distance:.3f} m")
        return distance

    def judge_ball(
        self,
        driver: Any,
        sensor_manager: SensorManager,
        lower_threshold_m: float = DEFAULT_BALL_DISTANCE_LOWER_THRESHOLD_M,
        upper_threshold_m: float = DEFAULT_BALL_DISTANCE_UPPER_THRESHOLD_M,
        *,
        rotation_speed: float = DEFAULT_ROTATION_SPEED,
        tolerance_deg: float = DEFAULT_TURN_TOLERANCE_DEG,
        timeout_s: float = DEFAULT_TURN_TIMEOUT_S,
        loop_interval_s: float = DEFAULT_LOOP_INTERVAL_S,
        forward_duration_s: float = DEFAULT_FORWARD_DURATION_S,
        forward_speed: float = DEFAULT_FORWARD_SPEED,
        red_ratio_threshold: float = DEFAULT_RED_RATIO_THRESHOLD,
        image_processor: Optional[ImageProcessor] = None,
    ) -> Optional[dict[str, Any]]:
        """最も遠い測定点へ向き、前進後の画像から赤いボールを確認する。

        ``detect_ball()`` が ``self.scan_results`` に保存した結果を使用する。
        下限と上限は両方とも範囲に含む。条件を満たす測定結果がない場合は
        モーターを停止し、``None`` を返す。赤色を検知した場合は、既存の
        ``NavigationController.follow_forward()`` で指定時間だけ直進する。
        """
        lower_threshold_m = float(lower_threshold_m)
        upper_threshold_m = float(upper_threshold_m)
        forward_duration_s = float(forward_duration_s)
        forward_speed = float(forward_speed)
        red_ratio_threshold = float(red_ratio_threshold)

        if lower_threshold_m < 0.0:
            raise ValueError("lower_threshold_m must be 0 or greater")
        if upper_threshold_m < lower_threshold_m:
            raise ValueError(
                "upper_threshold_m must be greater than or equal to "
                "lower_threshold_m"
            )
        if forward_duration_s <= 0.0:
            raise ValueError("forward_duration_s must be greater than 0")
        if not 0.0 < forward_speed <= 100.0:
            raise ValueError("forward_speed must be in the range 0 to 100")
        if not 0.0 <= red_ratio_threshold <= 1.0:
            raise ValueError("red_ratio_threshold must be in the range 0 to 1")

        candidates = [
            sample
            for sample in self.scan_results
            if sample.get("distance_m") is not None
            and lower_threshold_m <= float(sample["distance_m"]) <= upper_threshold_m
        ]

        if not candidates:
            driver.stop()
            return None

        selected = max(candidates, key=lambda sample: float(sample["distance_m"]))
        target_heading = self._normalize_heading(selected["heading_deg"])
        current_heading = self._normalize_heading(sensor_manager.get_heading_deg())
        turn_angle = self._heading_change_deg(target_heading, current_heading)

        navigation_controller = NavigationController()
        turn_result = navigation_controller.rotate_by_angle(
            driver,
            sensor_manager,
            turn_angle,
            speed=rotation_speed,
            tolerance_deg=tolerance_deg,
            timeout_s=timeout_s,
            loop_interval=loop_interval_s,
        )

        result = {
            "target_heading_deg": target_heading,
            "selected_distance_m": float(selected["distance_m"]),
            "turn_angle_deg": turn_angle,
            "reached": bool(turn_result["reached"]),
            "rotated_angle_deg": float(turn_result["rotated_angle_deg"]),
            "selected_sample": selected.copy(),
            "forward_completed": False,
            "forward_duration_s": None,
            "distance_after_forward_m": None,
            "ball_detected": False,
            "red_ratio": 0.0,
            "red_result": None,
        }

        if not turn_result["reached"]:
            print("目標方位への旋回に失敗したため、前進を中止します")
            return result

        print("直進前にボール確認用の正面画像を撮影します")
        frame = sensor_manager.capture_front_frame()

        if image_processor is None:
            processor = ImageProcessor()
        else:
            processor = image_processor

        red_result = processor.detect_color(
            frame,
            hsv_ranges=processor.RED_HSV_RANGES,
            color_threshold=red_ratio_threshold,
            block_threshold=red_ratio_threshold,
        )
        ball_detected = bool(red_result["is_color_detected"])
        red_ratio = float(red_result["total_color_ratio"])

        result["ball_detected"] = ball_detected
        result["red_ratio"] = red_ratio
        result["red_result"] = red_result

        if ball_detected:
            print("ボール検知成功")
            print(f"ボール方向へ{forward_duration_s:.1f}秒間直進します")
            navigation_controller.follow_forward(
                driver,
                sensor_manager,
                duration_time=forward_duration_s,
                base_speed=forward_speed,
                loop_interval=loop_interval_s,
                stop_ramp_steps=1,
                stop_ramp_interval=0.0,
            )
            result["forward_completed"] = True
            result["forward_duration_s"] = forward_duration_s
            print(f"{forward_duration_s:.1f}秒間の直進が完了しました")

            distance_after_forward = sensor_manager.get_distance_m()
            if distance_after_forward is None:
                print("停止後の距離を測定できませんでした")
            else:
                distance_after_forward = float(distance_after_forward)
                result["distance_after_forward_m"] = distance_after_forward
                print(f"停止後の距離: {distance_after_forward:.3f} m")
            return result

        print(
            "ボール検知失敗: "
            f"赤色占有率={red_ratio * 100:.2f}%、"
            f"閾値={red_ratio_threshold * 100:.2f}%"
        )

        return result

    def _save_sample(
        self,
        sensor_manager: SensorManager,
        *,
        relative_angle_deg: float,
        heading_deg: float,
        elapsed_s: float,
    ) -> None:
        distance = sensor_manager.get_distance_m()
        distance_m: Optional[float]
        if distance is None:
            distance_m = None
        else:
            distance_m = float(distance)

        self.scan_results.append(
            {
                "relative_angle_deg": float(relative_angle_deg),
                "heading_deg": float(heading_deg),
                "distance_m": distance_m,
                "elapsed_s": float(elapsed_s),
            }
        )

    @staticmethod
    def _normalize_heading(heading_deg: float) -> float:
        """方位を0度以上360度未満に正規化する。"""
        return float(heading_deg) % 360.0

    @staticmethod
    def _heading_change_deg(current_deg: float, previous_deg: float) -> float:
        """2つの方位間の最短角度差を-180度以上180度未満で返す。"""
        return (current_deg - previous_deg + 180.0) % 360.0 - 180.0
