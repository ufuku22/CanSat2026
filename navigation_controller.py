import math
import time

from config import (
    CameraCaptureConfig,
    DriveControllerConfig,
    FollowTargetConfig,
    NavigationMotionConfig,
    NavigationPdConfig,
    NavigationTargetConfig,
    ParachuteAvoidanceConfig,
    PostureRestoreConfig,
    RedConeConfig,
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
        self.camera_config = CameraCaptureConfig()
        self.pd_config = NavigationPdConfig()
        self.posture_restore_config = PostureRestoreConfig()
        self.follow_target_config = FollowTargetConfig()
        self.stuck_avoidance_config = StuckAvoidanceConfig()
        self.parachute_avoidance_config = ParachuteAvoidanceConfig()
        self.red_cone_config = RedConeConfig()
        self._collision_monitor_started_at = None
        self._collision_last_sample_time = None
        self._collision_previous_forward_accel = None
        self._collision_candidate_times = []

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
        """重力除去済み線形加速度から前進中の衝突を検知して回避する。

        このメソッドを前進中の走行制御ループから繰り返し呼び出す。設定した
        センサー前方向へ線形加速度を投影し、逆向き加速度と負の変化率が両方の
        閾値を超えた時刻を保持する。設定時間内に必要回数へ達した場合だけ
        衝突と判定して、後退してから右へ90度旋回する。

        走行開始直後の加速は設定時間だけ無視する。1回の呼び出しでは最大
        1サンプルだけ取得し、衝突を検知して回避した場合だけTrueを返す。
        """
        config = self.stuck_avoidance_config
        now = time.monotonic()
        if (
            self._collision_last_sample_time is not None
            and now - self._collision_last_sample_time
            < config.SAMPLE_INTERVAL_S
        ):
            return False

        try:
            accel = sensor_manager.get_altitude_motion()["linear_accel_mps2"]
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
        self._collision_last_sample_time = now
        self._collision_previous_forward_accel = forward_accel

        if self._collision_monitor_started_at is None:
            self._collision_monitor_started_at = now

        if previous_time is None or previous_accel is None:
            return False

        if now - self._collision_monitor_started_at < config.STARTUP_IGNORE_S:
            return False

        sample_interval = now - previous_time
        if sample_interval <= 0.0:
            return False

        forward_jerk = (forward_accel - previous_accel) / sample_interval

        if (
            forward_accel > config.FORWARD_ACCEL_THRESHOLD_MPS2
            or forward_jerk > config.FORWARD_JERK_THRESHOLD_MPS3
        ):
            return False

        window_start = now - config.COLLISION_CONFIRM_WINDOW_S
        self._collision_candidate_times = [
            detected_at
            for detected_at in self._collision_candidate_times
            if detected_at >= window_start
        ]
        self._collision_candidate_times.append(now)
        candidate_count = len(self._collision_candidate_times)
        print(
            "衝突候補: "
            f"{candidate_count}/{config.COLLISION_CONFIRM_COUNT}, "
            f"window={config.COLLISION_CONFIRM_WINDOW_S:g}秒, "
            f"forward_accel={forward_accel:+.3f} m/s^2, "
            f"forward_jerk={forward_jerk:+.3f} m/s^3"
        )
        if candidate_count < config.COLLISION_CONFIRM_COUNT:
            return False

        self._reset_stuck_detection()

        print(
            "衝突検知: "
            f"sensor_forward={forward_axis}{'+' if forward_sign > 0 else '-'}, "
            f"forward_accel={forward_accel:+.3f} m/s^2, "
            f"forward_jerk={forward_jerk:+.3f} m/s^3"
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
        self._collision_candidate_times = []

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
                time.sleep(loop_interval)
        finally:
            driver.ramp_stop_forward(
                left_speed,
                right_speed,
                steps=stop_ramp_steps,
                interval=stop_ramp_interval,
            )

    # IMUの変化量を見ながら指定角度だけ旋回する
    def rotate_by_angle(
        self,
        driver,
        sensor_manager,
        angle_deg,
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

                remaining_angle = angle_deg - rotated_angle
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

    # 紫色パラシュートが前方から消えるまで旋回して避ける
    def avoid_parachute(
        self,
        driver,
        sensor_manager,
        image_processor=None,
    ):
        """前方カメラ画像から紫色パラシュートを検知し、紫色が消えるまで90度右旋回する。

        手順:
            1. 前方カメラ画像を撮影する。
            2. ImageProcessor.detect_color() で紫色を検知する。
            3. 紫色が検知されなければ、前方安全とみなして直進する。
            4. 紫色が検知されたら、rotate_by_angle() で時計回りに90度旋回する。
            5. 再度前方カメラ画像を撮影する。
            6. 紫色が検知されなくなるまで、撮影と90度旋回を繰り返す。

        注意:
            この処理はパラシュート回避テスト用です。
            本来はGPS目標方向へ復帰する処理が必要ですが、
            このテストでは紫色が見えなくなったらそのまま直進します。
        """

        if image_processor is None:
            from image_processor import ImageProcessor
            processor = ImageProcessor()
        else:
            processor = image_processor

        config = self.parachute_avoidance_config
        camera = self.camera_config
        history = []

        for attempt in range(1, config.MAX_ATTEMPTS + 1):
            print(
                "パラシュート回避: "
                f"紫色確認 {attempt}/{config.MAX_ATTEMPTS}"
            )

            frame = sensor_manager.capture_front_frame(
                width=camera.WIDTH,
                height=camera.HEIGHT,
                hdr=camera.HDR,
                timeout_ms=camera.TIMEOUT_MS,
            )

            purple_result = processor.detect_color(
                frame,
                hsv_ranges=processor.PURPLE_HSV_RANGES,
                color_threshold=config.PURPLE_THRESHOLD,
            )

            is_purple_detected = bool(purple_result["is_color_detected"])
            total_purple_ratio = float(purple_result["total_color_ratio"])

            history.append({
                "attempt": attempt,
                "is_purple_detected": is_purple_detected,
                "total_purple_ratio": total_purple_ratio,
                "purple_result": purple_result,
            })

            print(
                "パラシュート回避: "
                f"purple_detected={is_purple_detected}, "
                f"total_purple_ratio={total_purple_ratio:.3f}, "
                f"threshold={config.PURPLE_THRESHOLD:.3f}"
            )

            # 紫色が検知されなければ、前方安全とみなして直進する
            if not is_purple_detected:
                print("パラシュート回避: 紫色なし。直進します")

                try:
                    driver.drive(config.MOVE_SPEED)
                    time.sleep(config.MOVE_DURATION_S)
                finally:
                    driver.stop()

                return {
                    "action": "forward_clear",
                    "completed": True,
                    "attempts": attempt,
                    "purple_detected": False,
                    "purple_ratio": total_purple_ratio,
                    "purple_threshold": float(config.PURPLE_THRESHOLD),
                    "move_speed": config.MOVE_SPEED,
                    "move_duration_s": config.MOVE_DURATION_S,
                    "rotate_angle_deg": config.ROTATE_ANGLE_DEG,
                    "rotate_speed": config.ROTATE_SPEED,
                    "last_purple_result": purple_result,
                    "history": history,
                }

            # 紫色が検知されたら時計回りに90度旋回する
            print(
                "パラシュート回避: "
                f"紫色を検知しました。時計回りに"
                f"{config.ROTATE_ANGLE_DEG:.1f}度旋回します"
            )

            rotate_result = self.rotate_by_angle(
                driver,
                sensor_manager,
                config.ROTATE_ANGLE_DEG,
                speed=config.ROTATE_SPEED,
                tolerance_deg=config.ROTATE_TOLERANCE_DEG,
                timeout_s=config.ROTATE_TIMEOUT_S,
            )

            history[-1]["rotate_result"] = rotate_result

            print(
                "パラシュート回避: 旋回結果 "
                f"target={rotate_result['target_angle_deg']:.1f}, "
                f"rotated={rotate_result['rotated_angle_deg']:.1f}, "
                f"reached={rotate_result['reached']}"
            )

            time.sleep(config.POST_ROTATION_PAUSE_S)

        # 最大試行回数まで紫色が消えなかった場合
        print(
            "パラシュート回避: "
            "最大試行回数まで紫色が消えませんでした。停止します"
        )
        driver.stop()

        last = history[-1] if history else None

        return {
            "action": "failed_purple_still_detected",
            "completed": False,
            "attempts": config.MAX_ATTEMPTS,
            "purple_detected": (
                True if last is None else last["is_purple_detected"]
            ),
            "purple_ratio": (
                None if last is None else last["total_purple_ratio"]
            ),
            "purple_threshold": float(config.PURPLE_THRESHOLD),
            "move_speed": config.MOVE_SPEED,
            "move_duration_s": config.MOVE_DURATION_S,
            "rotate_angle_deg": config.ROTATE_ANGLE_DEG,
            "rotate_speed": config.ROTATE_SPEED,
            "last_purple_result": (
                None if last is None else last["purple_result"]
            ),
            "history": history,
        }

    # カメラ画像内に赤コーンが入るまで探索する
    def _find_red_cone_in_view(
        self,
        driver,
        sensor_manager,
        processor,
    ):
        config = self.red_cone_config
        camera = self.camera_config
        scan_history = []
        for scan_index in range(config.MAX_SCAN_STEPS):
            # 正面画像から赤色の量と方向を確認する。
            print(
                "赤コーン探索: "
                f"scan {scan_index + 1}/{config.MAX_SCAN_STEPS} 撮影します"
            )
            frame = sensor_manager.capture_front_frame(
                width=camera.WIDTH,
                height=camera.HEIGHT,
                hdr=camera.HDR,
                timeout_ms=camera.TIMEOUT_MS,
            )
            red_result = processor.detect_color(
                frame,
                hsv_ranges=processor.RED_HSV_RANGES,
                color_threshold=config.RED_THRESHOLD,
                block_threshold=config.RED_BLOCK_THRESHOLD,
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

            if float(red_result["total_color_ratio"]) > config.RED_THRESHOLD:
                print("赤コーン探索: 赤コーンを検出しました")
                return frame, red_result, scan_history

            # 赤コーンが見つからなければ、次の撮影前に一定角度だけ向きを変える。
            if scan_index < config.MAX_SCAN_STEPS - 1:
                print(
                    "赤コーン探索: "
                    f"赤コーンなし。{config.SCAN_ANGLE_DEG:.1f}度旋回して"
                    "再探索します"
                )
                self.rotate_by_angle(
                    driver,
                    sensor_manager,
                    config.SCAN_ANGLE_DEG,
                    speed=config.ROTATE_SPEED,
                    tolerance_deg=config.ROTATE_TOLERANCE_DEG,
                    timeout_s=config.ROTATE_TIMEOUT_S,
                )

        return None, None, scan_history

    # 赤コーンの画面位置を旋回角度に変換する
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

    # 赤色の大きさに応じて前進時間を選ぶ
    @staticmethod
    def _red_cone_forward_duration(red_ratio, default_duration_s, duration_table):
        red_ratio = float(red_ratio)
        for threshold, duration_s in duration_table:
            if red_ratio > threshold:
                return duration_s
        return default_duration_s

    # SensorManagerのGNSS現在地から目標方位を作る
    def _bearing_from_sensor_manager(self, sensor_manager):
        gnss = sensor_manager.get_gnss()
        latitude = gnss.get("latitude_deg")
        longitude = gnss.get("longitude_deg")
        if latitude is None or longitude is None:
            return None
        return self.bearing_to_target(latitude, longitude)

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

    # 指定方位へ向けて1周期分のPD制御を実行する
    def drive_toward_heading(
        self,
        driver,
        sensor_manager,
        target_heading,
        base_speed,
        prev_error,
        loop_interval,
    ):
        """指定方位を目標に、PD補正した左右出力で1周期分前進する。"""
        current = float(sensor_manager.get_heading_deg())
        error = self.heading_error(current, target_heading)
        d_error = (error - prev_error) / loop_interval
        correction = self.pd_config.KP * error + self.pd_config.KD * d_error

        left_speed = max(0.0, min(100.0, base_speed - correction))
        right_speed = max(0.0, min(100.0, base_speed + correction))
        driver.forward_differential(left_speed, right_speed)
        return left_speed, right_speed, error

    # 2つの方位の最短角度差を求める
    @staticmethod
    def heading_error(current, target):
        """現在方位と目標方位の最短角度差を-180度から+180度で返す。"""
        return (current - target + 180.0) % 360.0 - 180.0
