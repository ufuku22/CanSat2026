import math
import time

from config import (
    DriveControllerConfig,
    FollowTargetConfig,
    NavigationMotionConfig,
    NavigationPdConfig,
    NavigationTargetConfig,
    ParachuteAvoidanceConfig,
    PostureRestoreConfig,
    StuckAvoidanceConfig,
)


class NavigationController:

    # 目標座標を保持する
    def __init__(
        self,
        target_latitude_deg=NavigationTargetConfig.TARGET_LATITUDE_DEG,
        target_longitude_deg=NavigationTargetConfig.TARGET_LONGITUDE_DEG,
    ):
        self.target_latitude_deg = float(target_latitude_deg)
        self.target_longitude_deg = float(target_longitude_deg)
        self.pd_config = NavigationPdConfig()
        self.posture_restore_config = PostureRestoreConfig()
        self.follow_target_config = FollowTargetConfig()
        self.stuck_avoidance_config = StuckAvoidanceConfig()
        self.parachute_avoidance_config = ParachuteAvoidanceConfig()
        self._collision_monitor_started_at = None
        self._collision_last_sample_time = None
        self._collision_previous_forward_accel = None
        self._stuck_previous_motor_outputs = None
        self._stuck_start_checked = False
        self._stuck_delta_v_samples = []
        self._stuck_deceleration_started_at = None

    # 現在地から目標地点への方位を計算する
    def bearing_to_target(self, current_latitude_deg, current_longitude_deg):
        current_latitude_deg = float(current_latitude_deg)
        current_longitude_deg = float(current_longitude_deg)

        lat1 = math.radians(current_latitude_deg)
        lat2 = math.radians(self.target_latitude_deg)
        delta_lon = math.radians(self.target_longitude_deg - current_longitude_deg)

        x = math.sin(delta_lon) * math.cos(lat2)
        y = (
            math.cos(lat1) * math.sin(lat2)
            - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
        )
        return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0

    # 現在地から目標地点までの距離を計算する
    def distance_to_target_m(self, current_latitude_deg, current_longitude_deg):
        current_latitude_deg = float(current_latitude_deg)
        current_longitude_deg = float(current_longitude_deg)

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

    # 9軸センサの加速度から機体の姿勢を正常に戻す
    def restore_posture(self, driver, sensor_manager):
        config = self.posture_restore_config
        pulse_time = config.INITIAL_FLIP_PULSE_TIME_S
        for _ in range(config.MAX_ATTEMPTS):
            accel_x = float(sensor_manager.get_imu()["accel_mps2"][0])
            if abs(accel_x) < config.ACCEL_THRESHOLD_MPS2:
                break
            driver.flip(pulse_time=pulse_time)
            pulse_time += config.FLIP_PULSE_INCREMENT_S
            time.sleep(config.ACTION_WAIT_S)

        for _ in range(config.MAX_ATTEMPTS):
            accel_y = float(sensor_manager.get_imu()["accel_mps2"][1])
            if accel_y > -config.ACCEL_THRESHOLD_MPS2:
                break
            driver.reverse_stabilizer(speed=config.REVERSE_STABILIZER_SPEED)
            time.sleep(config.ACTION_WAIT_S)

        for _ in range(config.MAX_ATTEMPTS):
            accel_z = float(sensor_manager.get_imu()["accel_mps2"][2])
            if accel_z > -config.ACCEL_THRESHOLD_MPS2:
                break
            driver.reverse_stabilizer()
            time.sleep(config.ACTION_WAIT_S)

    # 前進中の逆向き線形加速度を検知した場合に回避行動を行う
    def avoid_stuck(
        self,
        driver,
        sensor_manager,
    ):
        """モーター出力と線形加速度から前進中のスタックを検知して回避する。

        既存の急衝突判定に加え、前進出力に対して発進加速がない場合と、
        出力が安定しているのに減速後の再加速がない場合を検知する。
        加速度は短時間だけ積分し、速度変化として使用する。

        走行開始直後の加速は設定時間だけ無視する。1回の呼び出しでは最大
        1サンプルだけ取得し、衝突を検知して回避した場合だけTrueを返す。
        """
        config = self.stuck_avoidance_config
        now = time.monotonic()

        motor_outputs = driver.get_forward_motor_outputs()
        if motor_outputs is None or max(motor_outputs) <= 0.0:
            self._reset_stuck_detection()
            return False

        if (
            self._collision_last_sample_time is not None
            and now - self._collision_last_sample_time
            < config.SAMPLE_INTERVAL_S
        ):
            return False

        try:
            accel = sensor_manager.get_linear_acceleration()
            forward_axis = str(config.SENSOR_FORWARD_AXIS).lower()
            forward_axis_index = {"x": 0, "y": 1, "z": 2}[forward_axis]
            forward_sign = float(config.SENSOR_FORWARD_SIGN)
            if forward_sign not in (-1.0, 1.0):
                raise ValueError
            forward_accel = float(accel[forward_axis_index]) * forward_sign
        except (KeyError, TypeError, IndexError, ValueError) as exc:
            self._reset_stuck_detection()
            raise RuntimeError(
                "IMUの前方向線形加速度またはセンサー前方向設定が不正です"
            ) from exc

        previous_time = self._collision_last_sample_time
        previous_accel = self._collision_previous_forward_accel
        motor_outputs = tuple(float(output) for output in motor_outputs)
        previous_motor_outputs = self._stuck_previous_motor_outputs
        motor_output = sum(motor_outputs) / 2.0
        self._collision_last_sample_time = now
        self._collision_previous_forward_accel = forward_accel
        self._stuck_previous_motor_outputs = motor_outputs

        if self._collision_monitor_started_at is None:
            self._collision_monitor_started_at = now

        if (
            previous_time is None
            or previous_accel is None
            or previous_motor_outputs is None
        ):
            return False

        sample_interval = now - previous_time
        if sample_interval <= 0.0:
            return False

        forward_jerk = (forward_accel - previous_accel) / sample_interval
        collision_detected = (
            now - self._collision_monitor_started_at >= config.STARTUP_IGNORE_S
            and forward_accel <= config.FORWARD_ACCEL_THRESHOLD_MPS2
            and forward_jerk <= config.FORWARD_JERK_THRESHOLD_MPS3
        )

        motor_output_is_stable = (
            max(
                abs(current - previous)
                for current, previous in zip(
                    motor_outputs,
                    previous_motor_outputs,
                )
            )
            <= config.MOTOR_OUTPUT_CHANGE_THRESHOLD_PERCENT
        )
        motor_output_is_high = (
            min(motor_outputs)
            >= config.MOTOR_OUTPUT_THRESHOLD_PERCENT
        )
        motion_stuck_detected = False
        motion_stuck_reason = None
        if motor_output_is_high and motor_output_is_stable:
            self._stuck_delta_v_samples.append(
                (
                    now,
                    (previous_accel + forward_accel) / 2.0 * sample_interval,
                )
            )
            window_start = now - config.MOTION_WINDOW_S
            self._stuck_delta_v_samples = [
                sample
                for sample in self._stuck_delta_v_samples
                if sample[0] >= window_start
            ]
            delta_v = sum(sample[1] for sample in self._stuck_delta_v_samples)

            if not self._stuck_start_checked:
                if (
                    now - self._collision_monitor_started_at
                    >= config.MOTION_WINDOW_S
                ):
                    self._stuck_start_checked = True
                    motion_stuck_detected = (
                        delta_v < config.MOTION_DELTA_V_THRESHOLD_MPS
                    )
                    if motion_stuck_detected:
                        motion_stuck_reason = "発進応答なし"
            elif delta_v <= -config.MOTION_DELTA_V_THRESHOLD_MPS:
                if self._stuck_deceleration_started_at is None:
                    self._stuck_deceleration_started_at = now
            elif delta_v >= config.MOTION_DELTA_V_THRESHOLD_MPS:
                self._stuck_deceleration_started_at = None

            if self._stuck_deceleration_started_at is not None:
                motion_stuck_detected = (
                    now - self._stuck_deceleration_started_at
                    >= config.MOTION_WINDOW_S
                )
                if motion_stuck_detected:
                    motion_stuck_reason = "減速後の再加速なし"
        else:
            self._stuck_delta_v_samples.clear()
            self._stuck_deceleration_started_at = None
            if not motor_output_is_high:
                self._stuck_start_checked = True

        if not collision_detected and not motion_stuck_detected:
            return False

        self._reset_stuck_detection()

        print(
            f"{'衝突' if collision_detected else motion_stuck_reason}検知: "
            f"sensor_forward={forward_axis}{'+' if forward_sign > 0 else '-'}, "
            f"forward_accel={forward_accel:+.3f} m/s^2, "
            f"forward_jerk={forward_jerk:+.3f} m/s^3, "
            f"motor_output={motor_output:.1f}%"
        )

        self._run_stuck_escape(driver, sensor_manager)
        return True

    def _run_stuck_escape(self, driver, sensor_manager):
        """後退してから、角度指定の右旋回を実行する。"""
        config = self.stuck_avoidance_config
        ramp_duration_s = (
            config.STOP_RAMP_STEPS * config.STOP_RAMP_INTERVAL_S
        )
        print(f"衝突回避: 約{ramp_duration_s:g}秒かけて減速停止します")
        driver.ramp_stop_current_forward(
            steps=config.STOP_RAMP_STEPS,
            interval=config.STOP_RAMP_INTERVAL_S,
        )

        print(f"衝突回避: {config.REVERSE_DURATION_S:g}秒後退します")
        try:
            driver.drive(-config.REVERSE_SPEED)
            time.sleep(config.REVERSE_DURATION_S)
        finally:
            driver.stop()

        print(
            "衝突回避: "
            f"右へ{config.RIGHT_TURN_ANGLE_DEG:g}度回頭します"
        )
        rotate_result = self.rotate_by_angle(
            driver,
            sensor_manager,
            config.RIGHT_TURN_ANGLE_DEG,
            speed=config.RIGHT_TURN_SPEED,
            tolerance_deg=config.RIGHT_TURN_TOLERANCE_DEG,
            timeout_s=config.RIGHT_TURN_TIMEOUT_S,
        )
        print(
            "衝突回避: 旋回結果 "
            f"rotated={rotate_result['rotated_angle_deg']:.1f}度, "
            f"reached={rotate_result['reached']}"
        )

    def _reset_stuck_detection(self):
        """衝突検知のサンプリング状態を破棄する。"""
        self._collision_monitor_started_at = None
        self._collision_last_sample_time = None
        self._collision_previous_forward_accel = None
        self._stuck_previous_motor_outputs = None
        self._stuck_start_checked = False
        self._stuck_delta_v_samples.clear()
        self._stuck_deceleration_started_at = None

    # GNSSで目標方位を更新しながらゴールまで走行する
    def follow_target(
        self,
        driver,
        sensor_manager,
        status_callback=None,
        stuck_avoidance_callback=None,
    ):
        """GNSS現在地を確認しながら目標地点までPD制御で走行する。"""
        config = self.follow_target_config
        base_speed = float(config.BASE_SPEED)
        if stuck_avoidance_callback is None:
            stuck_avoidance_callback = lambda: self.avoid_stuck(
                driver,
                sensor_manager,
            )

        # 初回実行時にlast_valid_gnss_timeとlast_target_bearingを初期化する
        if not hasattr(self, 'last_valid_gnss_time'):
            self.last_valid_gnss_time = (
                time.monotonic() - config.GNSS_LOST_GRACE_S
            )
        if not hasattr(self, 'last_target_bearing'):
            self.last_target_bearing = None

        deadline = time.monotonic() + config.TIMEOUT_S
        last_target_update = 0.0
        prev_error = 0.0
        left_speed = base_speed
        right_speed = base_speed
        moving = False
        waiting_for_gnss = False
        gnss_recovery_failure_count = 0
        gnss_recovery_move_count = 0
        self._reset_stuck_detection()

        while time.monotonic() < deadline:
            now = time.monotonic()
            # 目標方位を更新するかどうかの判定
            should_update_target = (
                self.last_target_bearing is None
                or now - last_target_update >= config.TARGET_UPDATE_INTERVAL_S
            )

            if should_update_target:
                # GNSS現在地を取得して目標方位を更新する
                position = self._position_from_sensor_manager(sensor_manager)
                last_target_update = now

                if position is not None:
                    # GNSSが取れたら距離と方位を更新する
                    latitude, longitude = position
                    distance_m = self.distance_to_target_m(latitude, longitude)
                    bearing_deg = self.bearing_to_target(latitude, longitude)
                    self.last_target_bearing = bearing_deg
                    waiting_for_gnss = False
                    gnss_recovery_failure_count = 0
                    gnss_recovery_move_count = 0
                    # ステータスコールバックに現在地と目標までの距離を通知する
                    if status_callback is not None:
                        status_callback(
                            f"現在地: lat={latitude:.7f}, lon={longitude:.7f}, "
                            f"目標まで {distance_m:.1f} m, 方位 {bearing_deg:.1f} deg"
                    )
                    # ゴール判定
                    if distance_m <= config.GOAL_RADIUS_M:
                        self._reset_stuck_detection()
                        driver.stop()
                        return True
                elif (
                    self.last_target_bearing is None
                    or now - self.last_valid_gnss_time
                    >= config.GNSS_LOST_GRACE_S
                ):
                    # GNSSロストが続いたら停止して復帰を待つ
                    if moving:
                        driver.ramp_stop_forward(
                            left_speed,
                            right_speed,
                        )
                        moving = False
                    self._reset_stuck_detection()

                    gnss_recovery_failure_count += 1
                    if (
                        gnss_recovery_failure_count
                        >= config.GNSS_RECOVERY_FAILURE_LIMIT
                    ):
                        gnss_recovery_move_count += 1
                        gnss_recovery_failure_count = 0
                        waiting_for_gnss = False
                        self.last_target_bearing = None
                        if status_callback is not None:
                            status_callback(
                                "GNSS再取得に失敗したため場所を移動します。"
                                f"移動回数={gnss_recovery_move_count}"
                            )
                        self._move_for_gnss_recovery(
                            driver,
                            sensor_manager,
                        )
                        continue

                    if not waiting_for_gnss and status_callback is not None:
                        status_callback(
                            "GNSS現在地が取得できません。"
                            "再取得できるまで待機します。"
                        )
                    waiting_for_gnss = True
                    time.sleep(
                        min(
                            config.GNSS_RETRY_INTERVAL_S,
                            max(0.0, deadline - time.monotonic()),
                        )
                    )
                    continue
                elif status_callback is not None:
                    status_callback(
                        f"GNSS取得失敗。{config.GNSS_LOST_GRACE_S:g}秒未満のため"
                        "直近の方位を維持して走行を継続します。"
                    )

            # 最後に得た目標方位へPD制御で進む
            left_speed, right_speed, prev_error = self.drive_toward_heading(
                driver,
                sensor_manager,
                target_heading=self.last_target_bearing,
                base_speed=base_speed,
                prev_error=prev_error,
                loop_interval=config.LOOP_INTERVAL_S,
            )
            moving = True

            if (
                self.stuck_avoidance_config.ENABLED
                and stuck_avoidance_callback()
            ):
                driver.stop()
                if status_callback is not None:
                    status_callback(
                        "衝突回避完了。"
                        "GNSSを再取得してからGPS誘導を再開します。"
                    )
                prev_error = 0.0
                left_speed = base_speed
                right_speed = base_speed
                moving = False
                self.last_target_bearing = None
                gnss_recovery_failure_count = 0
                gnss_recovery_move_count = 0
                continue

            time.sleep(config.LOOP_INTERVAL_S)

        self._reset_stuck_detection()
        driver.stop()
        return False

    def _move_for_gnss_recovery(self, driver, sensor_manager):
        """現在方位を維持して短時間移動し、GNSSを再取得しやすい場所へ移る。"""
        config = self.follow_target_config
        self.follow_forward(
            driver,
            sensor_manager,
            config.GNSS_RECOVERY_MOVE_DURATION_S,
            base_speed=config.GNSS_RECOVERY_MOVE_SPEED,
            loop_interval=config.LOOP_INTERVAL_S,
        )

    # 開始時の方位を保ちながら一定時間前進する
    def follow_forward(
        self,
        driver,
        sensor_manager,
        duration_time,
        base_speed=NavigationMotionConfig.FOLLOW_FORWARD_BASE_SPEED,
        loop_interval=NavigationMotionConfig.FOLLOW_FORWARD_LOOP_INTERVAL_S,
        stop_ramp_steps=DriveControllerConfig.RAMP_STOP_STEPS,
        stop_ramp_interval=DriveControllerConfig.RAMP_STOP_INTERVAL_S,
    ):
        """PD制御で方位を補正しながらduration_time秒だけ前進する。"""
        base_speed = float(base_speed)

        target = float(sensor_manager.get_heading_deg())
        prev_error = 0.0
        left_speed = base_speed
        right_speed = base_speed
        start_time = time.monotonic()
        stuck_avoided = False
        self._reset_stuck_detection()

        try:
            driver.forward_differential(left_speed, right_speed)

            while time.monotonic() - start_time <= duration_time:
                left_speed, right_speed, prev_error = self.drive_toward_heading(
                    driver,
                    sensor_manager,
                    target_heading=target,
                    base_speed=base_speed,
                    prev_error=prev_error,
                    loop_interval=loop_interval,
                )
                if self.stuck_avoidance_config.ENABLED and self.avoid_stuck(
                    driver,
                    sensor_manager,
                ):
                    stuck_avoided = True
                    break
                time.sleep(loop_interval)
        finally:
            if stuck_avoided:
                driver.stop()
            else:
                self._pd_ramp_stop_forward(
                    driver,
                    sensor_manager,
                    left_speed,
                    right_speed,
                    target_heading=target,
                    prev_error=prev_error,
                    steps=stop_ramp_steps,
                    interval=stop_ramp_interval,
                )
            self._reset_stuck_detection()

    def _pd_ramp_stop_forward(
        self,
        driver,
        sensor_manager,
        left_speed,
        right_speed,
        *,
        target_heading,
        prev_error,
        steps,
        interval,
    ):
        """方位PD補正を続けながら、現在と同じ時間で前進出力を下げる。"""
        steps = max(1, int(steps))
        interval = max(0.0, float(interval))
        pd_interval = max(interval, 1e-6)
        stop_base_speed = (
            max(0.0, float(left_speed))
            + max(0.0, float(right_speed))
        ) / 2.0

        try:
            for step in range(steps - 1, -1, -1):
                output_scale = step / steps
                _, _, prev_error = self.drive_toward_heading(
                    driver,
                    sensor_manager,
                    target_heading=target_heading,
                    base_speed=stop_base_speed,
                    prev_error=prev_error,
                    loop_interval=pd_interval,
                    output_scale=output_scale,
                )
                time.sleep(interval)
        finally:
            driver.stop()

    # IMUの変化量を見ながら指定角度だけ旋回する
    def rotate_by_angle(
        self,
        driver,
        sensor_manager,
        angle_deg,
        turn_gain=1.0,
        speed=NavigationMotionConfig.ROTATE_SPEED,
        tolerance_deg=NavigationMotionConfig.ROTATE_TOLERANCE_DEG,
        timeout_s=NavigationMotionConfig.ROTATE_TIMEOUT_S,
        loop_interval=NavigationMotionConfig.ROTATE_LOOP_INTERVAL_S,
    ):
        """IMUの方位を見ながら指定角度だけその場旋回する。

        angle_degが正なら右旋回、負なら左旋回する。
        timeout_sがNoneの場合は、指定角度へ到達するまで待ち続ける。
        """
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

        if timeout_s is not None:
            timeout_s = float(timeout_s)
            if timeout_s <= 0.0:
                raise ValueError("timeout_s must be greater than 0")

        try:
            if angle_deg > 0:
                driver.turn_right(speed)
            else:
                driver.turn_left(speed)

            while timeout_s is None or time.monotonic() - start_time <= timeout_s:
                time.sleep(loop_interval)
                current_heading = float(sensor_manager.get_heading_deg())
                rotated_angle += self.heading_error(current_heading, previous_heading)
                previous_heading = current_heading

                remaining_angle = angle_deg*turn_gain - rotated_angle
                if abs(remaining_angle) <= tolerance_deg:
                    reached = True
                    break
                if angle_deg > 0 and remaining_angle < 0:
                    # 目標を通過した場合も旋回自体は完了したものとして扱う。
                    reached = True
                    break
                if angle_deg < 0 and remaining_angle > 0:
                    # 目標を通過した場合も旋回自体は完了したものとして扱う。
                    reached = True
                    break
        finally:
            driver.stop()

        return {
            "target_angle_deg": angle_deg,
            "rotated_angle_deg": rotated_angle,
            "reached": reached,
        }

    # 前方に紫色パラシュートがあれば右へ避けて前進する
    def avoid_parachute(
        self,
        driver,
        sensor_manager,
        image_processor=None,
    ):
        """前方の紫色を確認し、必要なら右旋回してPD制御で前進する。"""

        if image_processor is None:
            from image_processor import ImageProcessor
            processor = ImageProcessor()
        else:
            processor = image_processor

        config = self.parachute_avoidance_config
        purple_result = processor.detect_color(
            sensor_manager.capture_front_frame(),
            hsv_ranges=processor.PURPLE_HSV_RANGES,
            color_threshold=config.PURPLE_THRESHOLD,
        )
        purple_result.pop("color_mask", None)
        is_purple_detected = bool(purple_result["is_color_detected"])
        total_purple_ratio = float(purple_result["total_color_ratio"])
        rotate_result = None

        if is_purple_detected:
            print(
                "パラシュート回避: "
                f"紫色を検知したため右へ{config.ROTATE_ANGLE_DEG:.1f}度旋回します"
            )
            rotate_result = self.rotate_by_angle(
                driver,
                sensor_manager,
                config.ROTATE_ANGLE_DEG,
                speed=config.ROTATE_SPEED,
                tolerance_deg=config.ROTATE_TOLERANCE_DEG,
                timeout_s=config.ROTATE_TIMEOUT_S,
            )
            action = "avoid_right"
        else:
            print("パラシュート回避: 紫色なし。目標方向へ直進します")
            action = "forward_clear"

        self.follow_forward(
            driver,
            sensor_manager,
            config.MOVE_DURATION_S,
            base_speed=config.MOVE_SPEED,
        )

        return {
            "action": action,
            "completed": True,
            "attempts": 1,
            "purple_detected": is_purple_detected,
            "purple_ratio": total_purple_ratio,
            "purple_threshold": float(config.PURPLE_THRESHOLD),
            "move_speed": config.MOVE_SPEED,
            "move_duration_s": config.MOVE_DURATION_S,
            "rotate_angle_deg": config.ROTATE_ANGLE_DEG,
            "rotate_speed": config.ROTATE_SPEED,
            "rotate_result": rotate_result,
            "last_purple_result": purple_result,
        }

    # SensorManagerからGNSS現在地を取り出す
    def _position_from_sensor_manager(self, sensor_manager):
        gnss = sensor_manager.get_gnss()
        if not gnss.get("has_fix"):
            return None
        latitude = gnss.get("latitude_deg")
        longitude = gnss.get("longitude_deg")
        if latitude is None or longitude is None:
            return None
        self.last_valid_gnss_time = time.monotonic()
        return float(latitude), float(longitude)

    # 現在方位を読み取り、指定方位へ進むための左右モーター出力を1回更新する
    def drive_toward_heading(
        self,
        driver,
        sensor_manager,
        target_heading,
        base_speed=DriveControllerConfig.PD_FORWARD_SPEED,
        prev_error=0.0,
        loop_interval=NavigationMotionConfig.FOLLOW_FORWARD_LOOP_INTERVAL_S,
        output_scale=1.0,
    ):
        """指定方位を目標に、PD補正した左右出力で前進する。"""
        current = float(sensor_manager.get_heading_deg())
        error = self.heading_error(current, target_heading)
        d_error = (error - prev_error) / loop_interval
        correction = self.pd_config.KP * error + self.pd_config.KD * d_error

        output_scale = max(0.0, min(1.0, float(output_scale)))
        left_speed = (
            max(0.0, min(100.0, base_speed - correction)) * output_scale
        )
        right_speed = (
            max(0.0, min(100.0, base_speed + correction)) * output_scale
        )
        driver.forward_differential(left_speed, right_speed)
        return left_speed, right_speed, error

    # 2つの方位の最短角度差を求める
    @staticmethod
    def heading_error(current, target):
        """現在方位と目標方位の最短角度差を-180度から+180度で返す。"""
        return (current - target + 180.0) % 360.0 - 180.0
