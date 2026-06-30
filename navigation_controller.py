import math
import numbers
import time

from sensor_manager import CAMERA_FULL_HD_HEIGHT, CAMERA_FULL_HD_WIDTH


class NavigationController:

    DEFAULT_TARGET_LATITUDE_DEG = 35.0        #目標緯度
    DEFAULT_TARGET_LONGITUDE_DEG = 139.0      #目標経度
    DEFAULT_PARACHUTE_RED_THRESHOLD = 0.05
    DEFAULT_PARACHUTE_MOVE_SPEED = 60.0
    DEFAULT_PARACHUTE_MOVE_DURATION_S = 2.0

    def __init__(
        self,
        target_latitude_deg=DEFAULT_TARGET_LATITUDE_DEG,
        target_longitude_deg=DEFAULT_TARGET_LONGITUDE_DEG,
    ):
        self.target_latitude_deg = self._validate_latitude(target_latitude_deg)
        self.target_longitude_deg = self._validate_longitude(target_longitude_deg)

    def bearing_to_target(self, current_latitude_deg, current_longitude_deg):
        current_latitude_deg = self._validate_latitude(current_latitude_deg)
        current_longitude_deg = self._validate_longitude(current_longitude_deg)

        lat1 = math.radians(current_latitude_deg)
        lat2 = math.radians(self.target_latitude_deg)
        delta_lon = math.radians(self.target_longitude_deg - current_longitude_deg)

        x = math.sin(delta_lon) * math.cos(lat2)
        y = (
            math.cos(lat1) * math.sin(lat2)
            - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
        )
        return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0

    def distance_to_target_m(self, current_latitude_deg, current_longitude_deg):
        current_latitude_deg = self._validate_latitude(current_latitude_deg)
        current_longitude_deg = self._validate_longitude(current_longitude_deg)

        earth_radius_m = 6371000.0
        lat1 = math.radians(current_latitude_deg)
        lat2 = math.radians(self.target_latitude_deg)
        delta_lat = math.radians(self.target_latitude_deg - current_latitude_deg)
        delta_lon = math.radians(self.target_longitude_deg - current_longitude_deg)

        a = (
            math.sin(delta_lat / 2.0) ** 2
            + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon / 2.0) ** 2
        )
        return earth_radius_m * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    def follow_target(
        self,
        driver,
        sensor_manager,
        duration_time,
        base_speed=100.0,
        kp=0.80,
        kd=0.05,
        loop_interval=0.02,
        target_update_interval=60.0,
        stop_ramp_steps=100,
        stop_ramp_interval=0.03,
    ):
        """一定間隔で目標方位を更新しながらPD制御で目標地点へ向かう。"""
        base_speed = self._clamp_speed(base_speed)
        target = self._bearing_from_sensor_manager(sensor_manager)
        if target is None:
            raise RuntimeError("GPS現在地が取得できないため目標方位を計算できません")
        prev_error = 0.0
        left_speed = base_speed
        right_speed = base_speed
        start_time = time.monotonic()
        last_target_update = start_time

        try:
            driver.forward_differential(left_speed, right_speed)

            while time.monotonic() - start_time <= duration_time:
                now = time.monotonic()
                if now - last_target_update >= target_update_interval:
                    updated_target = self._bearing_from_sensor_manager(sensor_manager)
                    if updated_target is not None:
                        target = updated_target
                    last_target_update = now

                current = float(sensor_manager.get_heading_deg())
                error = self.heading_error(current, target)
                d_error = (error - prev_error) / loop_interval
                correction = kp * error + kd * d_error

                left_speed = self._clamp_speed(base_speed - correction)
                right_speed = self._clamp_speed(base_speed + correction)
                driver.forward_differential(left_speed, right_speed)

                prev_error = error
                time.sleep(loop_interval)
        finally:
            driver.ramp_stop_forward(
                left_speed,
                right_speed,
                steps=stop_ramp_steps,
                interval=stop_ramp_interval,
            )

    def follow_forward(
        self,
        driver,
        sensor_manager,
        duration_time,
        base_speed=100.0,
        kp=0.80,
        kd=0.05,
        loop_interval=0.02,
        stop_ramp_steps=100,
        stop_ramp_interval=0.03,
    ):
        """PD制御で方位を補正しながらduration_time秒だけ前進する。"""
        base_speed = self._clamp_speed(base_speed)

        target = float(sensor_manager.get_heading_deg())
        prev_error = 0.0
        left_speed = base_speed
        right_speed = base_speed
        start_time = time.monotonic()

        try:
            driver.forward_differential(left_speed, right_speed)

            while time.monotonic() - start_time <= duration_time:
                current = float(sensor_manager.get_heading_deg())
                error = self.heading_error(current, target)
                d_error = (error - prev_error) / loop_interval
                correction = kp * error + kd * d_error

                left_speed = self._clamp_speed(base_speed - correction)
                right_speed = self._clamp_speed(base_speed + correction)
                driver.forward_differential(left_speed, right_speed)

                prev_error = error
                time.sleep(loop_interval)
        finally:
            driver.ramp_stop_forward(
                left_speed,
                right_speed,
                steps=stop_ramp_steps,
                interval=stop_ramp_interval,
            )

    def follow_petit_forward(
        self,
        driver,
        sensor_manager,
        duration_time,
        base_speed=80.0,
        kp=0.80,
        kd=0.05,
        loop_interval=0.10,
    ):
        """短い減速で停止するPD制御前進。"""
        self.follow_forward(
            driver,
            sensor_manager,
            duration_time,
            base_speed=base_speed,
            kp=kp,
            kd=kd,
            loop_interval=loop_interval,
            stop_ramp_steps=20,
            stop_ramp_interval=0.01,
        )

    def rotate_by_angle(
        self,
        driver,
        sensor_manager,
        angle_deg,
        speed=30.0,
        tolerance_deg=3.0,
        timeout_s=10.0,
        loop_interval=0.01,
    ):
        """IMUの方位を見ながら指定角度だけその場旋回する。

        angle_degが正なら右旋回、負なら左旋回する。
        """
        angle_deg = self._validate_number(angle_deg, "angle_deg")
        speed = self._validate_motor_output(speed)
        tolerance_deg = self._validate_non_negative_number(tolerance_deg, "tolerance_deg")
        timeout_s = self._validate_duration(timeout_s, "timeout_s")
        loop_interval = self._validate_duration(loop_interval, "loop_interval")

        if abs(angle_deg) <= tolerance_deg:
            driver.stop()
            return {
                "target_angle_deg": angle_deg,
                "rotated_angle_deg": 0.0,
                "reached": True,
            }

        start_time = time.monotonic()
        previous_heading = float(sensor_manager.get_heading_deg())
        rotated_angle = 0.0
        reached = False

        try:
            if angle_deg > 0:
                driver.turn_right(speed)
            else:
                driver.turn_left(speed)

            while time.monotonic() - start_time <= timeout_s:
                time.sleep(loop_interval)
                current_heading = float(sensor_manager.get_heading_deg())
                rotated_angle += self.heading_error(current_heading, previous_heading)
                previous_heading = current_heading

                remaining_angle = angle_deg - rotated_angle
                if abs(remaining_angle) <= tolerance_deg:
                    reached = True
                    break
                if angle_deg > 0 and remaining_angle < 0:
                    break
                if angle_deg < 0 and remaining_angle > 0:
                    break
        finally:
            driver.stop()

        return {
            "target_angle_deg": angle_deg,
            "rotated_angle_deg": rotated_angle,
            "reached": reached,
        }

    def avoid_parachute(
        self,
        driver,
        sensor_manager,
        *,
        red_threshold=DEFAULT_PARACHUTE_RED_THRESHOLD,
        move_speed=DEFAULT_PARACHUTE_MOVE_SPEED,
        turn_speed=DEFAULT_PARACHUTE_MOVE_SPEED,
        safe_forward_duration_s=DEFAULT_PARACHUTE_MOVE_DURATION_S,
        cautious_forward_duration_s=0.4,
        turn_duration_s=0.35,
        far_turn_duration_s=0.65,
        max_attempts=10,
        image_processor=None,
        capture_width=CAMERA_FULL_HD_WIDTH,
        capture_height=CAMERA_FULL_HD_HEIGHT,
        capture_hdr=False,
        capture_timeout_ms=2000,
    ):
        """前方カメラ画像から赤色パラシュートを検知し、5分割結果を使って回避する。

        手順:
            1. 前方カメラ画像を撮影する。
            2. ImageProcessor.detect_red() で赤色を検出する。
            3. 赤色が検出されなければ、そのまま直進して終了する。
            4. 赤色が検出された場合、5分割領域のうち赤色が最も少ない方向を探す。
            5. その方向へ旋回する。
            6. 最も赤色が少ない領域が0%なら、安全とみなして直進して終了する。
            7. 最も赤色が少ない領域にも赤色が少しでもあれば、少しだけ直進して再撮影する。

        5分割の対応:
            0: left_far
            1: left
            2: center
            3: right
            4: right_far
        """

        import time

        if image_processor is None:
            from image_processor import ImageProcessor
            processor = ImageProcessor()
        else:
            processor = image_processor

        move_speed = self._validate_motor_output(move_speed)
        turn_speed = self._validate_motor_output(turn_speed)
        safe_forward_duration_s = self._validate_duration(
            safe_forward_duration_s,
            "safe_forward_duration_s",
        )
        cautious_forward_duration_s = self._validate_duration(
            cautious_forward_duration_s,
            "cautious_forward_duration_s",
        )
        turn_duration_s = self._validate_duration(
            turn_duration_s,
            "turn_duration_s",
        )
        far_turn_duration_s = self._validate_duration(
            far_turn_duration_s,
            "far_turn_duration_s",
        )

        if max_attempts <= 0:
            raise ValueError("max_attemptsは1以上にしてください")

        block_names = [
            "left_far",
            "left",
            "center",
            "right",
            "right_far",
        ]

        # 同じ赤色比率だった場合は、なるべく中央に近い方向を選ぶ
        # center -> left/right -> left_far/right_far の優先順
        center_priority = {
            2: 0,
            1: 1,
            3: 1,
            0: 2,
            4: 2,
        }

        history = []

        def stop_after_motion():
            try:
                driver.stop()
            except Exception:
                # stop中の例外で元の例外を潰さないための保険
                pass

        for attempt in range(1, max_attempts + 1):
            print(f"パラシュート回避: 試行 {attempt}/{max_attempts}")

            frame = sensor_manager.capture_front_frame(
                width=capture_width,
                height=capture_height,
                hdr=capture_hdr,
                timeout_ms=capture_timeout_ms,
            )

            red_result = processor.detect_red(
                frame,
                red_threshold=red_threshold,
            )

            total_red_ratio = float(red_result["total_red_ratio"])
            red_block_ratios = [
                float(value)
                for value in red_result["red_block_ratios"]
            ]

            is_red_detected = bool(red_result["is_red_detected"])

            print(
                "パラシュート回避: "
                f"total_red_ratio={total_red_ratio:.3f}, "
                f"blocks={[round(x, 3) for x in red_block_ratios]}"
            )

            # 赤色がそもそも検出されない場合は、そのまま進む
            if not is_red_detected:
                print("パラシュート回避: 赤色なし。安全と判断して直進します")

                try:
                    driver.drive(move_speed)
                    time.sleep(safe_forward_duration_s)
                finally:
                    driver.stop()

                result = {
                    "action": "forward_clear",
                    "completed": True,
                    "attempts": attempt,
                    "red_ratio": total_red_ratio,
                    "red_threshold": float(red_threshold),
                    "best_direction": "center",
                    "best_block_index": 2,
                    "best_block_ratio": 0.0,
                    "move_speed": move_speed,
                    "turn_speed": turn_speed,
                    "forward_duration_s": safe_forward_duration_s,
                    "red_result": red_result,
                    "history": history,
                }
                return result

            # 5分割のうち、赤色が最も少ないブロックを選ぶ
            best_block_index = min(
                range(len(red_block_ratios)),
                key=lambda i: (red_block_ratios[i], center_priority[i]),
            )
            best_direction = block_names[best_block_index]
            best_block_ratio = red_block_ratios[best_block_index]

            print(
                "パラシュート回避: "
                f"最小赤色領域={best_direction}, "
                f"ratio={best_block_ratio:.3f}"
            )

            attempt_info = {
                "attempt": attempt,
                "total_red_ratio": total_red_ratio,
                "red_block_ratios": red_block_ratios,
                "best_direction": best_direction,
                "best_block_index": best_block_index,
                "best_block_ratio": best_block_ratio,
                "red_result": red_result,
            }
            history.append(attempt_info)

            # 最も赤色が少ない方向へ旋回する
            # centerの場合は旋回せず、そのまま直進方向とする
            if best_direction == "left_far":
                print("パラシュート回避: 左遠方が最も安全。長めに左旋回します")
                try:
                    driver.turn_left(turn_speed)
                    time.sleep(far_turn_duration_s)
                finally:
                    driver.stop()

            elif best_direction == "left":
                print("パラシュート回避: 左が最も安全。左旋回します")
                try:
                    driver.turn_left(turn_speed)
                    time.sleep(turn_duration_s)
                finally:
                    driver.stop()

            elif best_direction == "center":
                print("パラシュート回避: 中央が最も安全。旋回しません")

            elif best_direction == "right":
                print("パラシュート回避: 右が最も安全。右旋回します")
                try:
                    driver.turn_right(turn_speed)
                    time.sleep(turn_duration_s)
                finally:
                    driver.stop()

            elif best_direction == "right_far":
                print("パラシュート回避: 右遠方が最も安全。長めに右旋回します")
                try:
                    driver.turn_right(turn_speed)
                    time.sleep(far_turn_duration_s)
                finally:
                    driver.stop()

            # 最も少ない領域が0%なら、そこは完全に赤色なしとみなして直進終了
            if best_block_ratio == 0.0:
                print("パラシュート回避: 選択方向の赤色が0%。そのまま直進します")

                try:
                    driver.drive(move_speed)
                    time.sleep(safe_forward_duration_s)
                finally:
                    driver.stop()

                result = {
                    "action": "avoid_and_forward_clear",
                    "completed": True,
                    "attempts": attempt,
                    "red_ratio": total_red_ratio,
                    "red_threshold": float(red_threshold),
                    "best_direction": best_direction,
                    "best_block_index": best_block_index,
                    "best_block_ratio": best_block_ratio,
                    "move_speed": move_speed,
                    "turn_speed": turn_speed,
                    "forward_duration_s": safe_forward_duration_s,
                    "red_result": red_result,
                    "history": history,
                }
                return result

            # 少しでも赤色が残っている場合は、慎重に少しだけ前進して再撮影
            print(
                "パラシュート回避: "
                "選択方向にも赤色が残っています。少し直進して再判定します"
            )

            try:
                driver.drive(move_speed)
                time.sleep(cautious_forward_duration_s)
            finally:
                driver.stop()

            time.sleep(0.2)

        # 最大試行回数を超えた場合
        print("パラシュート回避: 最大試行回数に達しました。停止します")
        driver.stop()

        return {
            "action": "failed_max_attempts",
            "completed": False,
            "attempts": max_attempts,
            "red_ratio": history[-1]["total_red_ratio"] if history else None,
            "red_threshold": float(red_threshold),
            "best_direction": history[-1]["best_direction"] if history else None,
            "best_block_index": history[-1]["best_block_index"] if history else None,
            "best_block_ratio": history[-1]["best_block_ratio"] if history else None,
            "move_speed": move_speed,
            "turn_speed": turn_speed,
            "forward_duration_s": cautious_forward_duration_s,
            "red_result": history[-1]["red_result"] if history else None,
            "history": history,
        }

    def guide_to_red_cone(
        self,
        driver,
        sensor_manager,
        *,
        red_threshold=0.01,
        goal_center_threshold=0.10,
        goal_total_threshold=0.60,
        red_block_threshold=0.03,
        scan_angle_deg=60.0,
        camera_fov_deg=75.0,
        max_scan_steps=6,
        max_steps=20,
        forward_duration_s=0.5,
        forward_duration_by_red_ratio=None,
        forward_speed=60.0,
        image_processor=None,
        capture_width=CAMERA_FULL_HD_WIDTH,
        capture_height=CAMERA_FULL_HD_HEIGHT,
        capture_hdr=True,
        capture_timeout_ms=2000,
        rotate_speed=30.0,
        rotate_tolerance_deg=3.0,
        rotate_timeout_s=10.0,
        forward_kp=0.80,
        forward_kd=0.05,
        loop_interval=0.10,
    ):
        """画像で赤コーンを探し、正面へ回頭して一定時間前進する。"""
        if forward_duration_by_red_ratio is None:
            forward_duration_by_red_ratio = (
                (0.40, 0.4),
                (0.20, 0.7),
            )

        if image_processor is None:
            from image_processor import ImageProcessor

            processor = ImageProcessor()
        else:
            processor = image_processor
        red_threshold = self._validate_ratio(red_threshold, "red_threshold")
        goal_center_threshold = self._validate_ratio(
            goal_center_threshold,
            "goal_center_threshold",
        )
        goal_total_threshold = self._validate_ratio(
            goal_total_threshold,
            "goal_total_threshold",
        )
        red_block_threshold = self._validate_ratio(
            red_block_threshold,
            "red_block_threshold",
        )
        scan_angle_deg = self._validate_number(scan_angle_deg, "scan_angle_deg")
        camera_fov_deg = self._validate_duration(camera_fov_deg, "camera_fov_deg")
        max_scan_steps = self._validate_positive_integer(max_scan_steps, "max_scan_steps")
        max_steps = self._validate_positive_integer(max_steps, "max_steps")
        forward_duration_s = self._validate_duration(
            forward_duration_s,
            "forward_duration_s",
        )
        forward_duration_by_red_ratio = self._validate_red_cone_forward_durations(
            forward_duration_by_red_ratio,
        )
        forward_speed = self._validate_motor_output(forward_speed)

        history = []
        last_goal_result = None

        for step in range(max_steps):
            _frame, red_result, scan_history = self._find_red_cone_in_view(
                driver,
                sensor_manager,
                processor,
                red_threshold=red_threshold,
                red_block_threshold=red_block_threshold,
                scan_angle_deg=scan_angle_deg,
                max_scan_steps=max_scan_steps,
                capture_width=capture_width,
                capture_height=capture_height,
                capture_hdr=capture_hdr,
                capture_timeout_ms=capture_timeout_ms,
                rotate_speed=rotate_speed,
                rotate_tolerance_deg=rotate_tolerance_deg,
                rotate_timeout_s=rotate_timeout_s,
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

            turn_angle = self._red_direction_to_turn_angle(
                red_result["red_direction"],
                camera_fov_deg,
            )
            turn_result = None
            if turn_angle != 0.0:
                turn_result = self.rotate_by_angle(
                    driver,
                    sensor_manager,
                    turn_angle,
                    speed=rotate_speed,
                    tolerance_deg=rotate_tolerance_deg,
                    timeout_s=rotate_timeout_s,
                )

            forward_duration = self._red_cone_forward_duration(
                red_result["total_red_ratio"],
                forward_duration_s,
                forward_duration_by_red_ratio,
            )

            self.follow_petit_forward(
                driver,
                sensor_manager,
                forward_duration,
                base_speed=forward_speed,
                kp=forward_kp,
                kd=forward_kd,
                loop_interval=loop_interval,
            )

            goal_frame = sensor_manager.capture_front_frame(
                width=capture_width,
                height=capture_height,
                hdr=capture_hdr,
                timeout_ms=capture_timeout_ms,
            )
            last_goal_result = processor.judge_red_goal_reached(
                goal_frame,
                red_threshold=red_threshold,
                goal_center_threshold=goal_center_threshold,
                goal_total_threshold=goal_total_threshold,
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

            if last_goal_result["goal_reached"]:
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
            "steps": max_steps,
            "history": history,
            "last_goal_result": last_goal_result,
        }

    def _find_red_cone_in_view(
        self,
        driver,
        sensor_manager,
        processor,
        *,
        red_threshold,
        red_block_threshold,
        scan_angle_deg,
        max_scan_steps,
        capture_width,
        capture_height,
        capture_hdr,
        capture_timeout_ms,
        rotate_speed,
        rotate_tolerance_deg,
        rotate_timeout_s,
    ):
        scan_history = []
        for scan_index in range(max_scan_steps):
            frame = sensor_manager.capture_front_frame(
                width=capture_width,
                height=capture_height,
                hdr=capture_hdr,
                timeout_ms=capture_timeout_ms,
            )
            red_result = processor.detect_red(
                frame,
                red_threshold=red_threshold,
                block_threshold=red_block_threshold,
            )
            scan_history.append({
                "scan_index": scan_index,
                "red_result": red_result,
            })

            if float(red_result["total_red_ratio"]) > red_threshold:
                return frame, red_result, scan_history

            if scan_index < max_scan_steps - 1:
                self.rotate_by_angle(
                    driver,
                    sensor_manager,
                    scan_angle_deg,
                    speed=rotate_speed,
                    tolerance_deg=rotate_tolerance_deg,
                    timeout_s=rotate_timeout_s,
                )

        return None, None, scan_history

    @staticmethod
    def _red_direction_to_turn_angle(red_direction, camera_fov_deg):
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

    @staticmethod
    def _red_cone_forward_duration(red_ratio, default_duration_s, duration_table):
        red_ratio = float(red_ratio)
        for threshold, duration_s in duration_table:
            if red_ratio > threshold:
                return duration_s
        return default_duration_s

    def _bearing_from_sensor_manager(self, sensor_manager):
        gnss = sensor_manager.get_gnss()
        latitude = gnss.get("latitude_deg")
        longitude = gnss.get("longitude_deg")
        if latitude is None or longitude is None:
            return None
        return self.bearing_to_target(latitude, longitude)

    @staticmethod
    def heading_error(current, target):
        """現在方位と目標方位の最短角度差を-180度から+180度で返す。"""
        return (current - target + 180.0) % 360.0 - 180.0

    @staticmethod
    def _clamp_speed(speed):
        return max(0.0, min(100.0, float(speed)))

    @staticmethod
    def _validate_motor_output(value):
        value = NavigationController._validate_number(value, "move_speed")
        if not 0.0 <= value <= 100.0:
            raise ValueError("move_speedは0から100の範囲にしてください")
        return value

    @staticmethod
    def _validate_duration(value, name):
        value = NavigationController._validate_number(value, name)
        if value <= 0:
            raise ValueError(f"{name}は0より大きい値にしてください")
        return value

    @staticmethod
    def _validate_ratio(value, name):
        value = NavigationController._validate_number(value, name)
        if not 0.0 <= value <= 1.0:
            raise ValueError(f"{name}は0から1の範囲にしてください")
        return value

    @staticmethod
    def _validate_positive_integer(value, name):
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name}は整数にしてください")
        if value <= 0:
            raise ValueError(f"{name}は1以上にしてください")
        return value

    @staticmethod
    def _validate_red_cone_forward_durations(duration_table):
        validated = []
        for threshold, duration_s in duration_table:
            validated.append((
                NavigationController._validate_ratio(threshold, "red_ratio_threshold"),
                NavigationController._validate_duration(duration_s, "forward_duration_s"),
            ))
        return tuple(sorted(validated, reverse=True))

    @staticmethod
    def _validate_non_negative_number(value, name):
        value = NavigationController._validate_number(value, name)
        if value < 0:
            raise ValueError(f"{name}は0以上にしてください")
        return value

    @staticmethod
    def _validate_latitude(value):
        value = NavigationController._validate_number(value, "latitude")
        if not -90.0 <= value <= 90.0:
            raise ValueError("latitudeは-90から90の範囲にしてください")
        return value

    @staticmethod
    def _validate_longitude(value):
        value = NavigationController._validate_number(value, "longitude")
        if not -180.0 <= value <= 180.0:
            raise ValueError("longitudeは-180から180の範囲にしてください")
        return value

    @staticmethod
    def _validate_number(value, name):
        if isinstance(value, bool) or not isinstance(value, numbers.Real):
            raise TypeError(f"{name}は数値にしてください")
        return float(value)
