import math
import time

from sensor_manager import CAMERA_FULL_HD_HEIGHT, CAMERA_FULL_HD_WIDTH


class NavigationController:

    DEFAULT_TARGET_LATITUDE_DEG = 35.0        #目標緯度
    DEFAULT_TARGET_LONGITUDE_DEG = 139.0      #目標経度

    # 目標座標を保持する
    def __init__(
        self,
        target_latitude_deg=DEFAULT_TARGET_LATITUDE_DEG,
        target_longitude_deg=DEFAULT_TARGET_LONGITUDE_DEG,
    ):
        self.target_latitude_deg = float(target_latitude_deg)
        self.target_longitude_deg = float(target_longitude_deg)

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

    # GNSSで目標方位を更新しながらゴールまで走行する
    def follow_target(
        self,
        driver,
        sensor_manager,
        timeout_s=120.0,
        goal_radius_m=3.0,
        base_speed=70.0,
        kp=0.80,
        kd=0.05,
        loop_interval=0.02,
        target_update_interval=1.0,
        stop_ramp_steps=100,
        stop_ramp_interval=0.03,
        gnss_lost_grace_s=6.0,
        gnss_retry_interval=5.0,
        status_callback=None,
    ):
        """GNSS現在地を確認しながら目標地点までPD制御で走行する。"""
        base_speed = float(base_speed)

        # 初回実行時にlast_valid_gnss_timeとlast_target_bearingを初期化する
        if not hasattr(self, 'last_valid_gnss_time'):
            self.last_valid_gnss_time = time.monotonic() - gnss_lost_grace_s
        if not hasattr(self, 'last_target_bearing'):
            self.last_target_bearing = None

        deadline = time.monotonic() + timeout_s
        last_target_update = 0.0
        prev_error = 0.0
        left_speed = base_speed
        right_speed = base_speed
        moving = False
        waiting_for_gnss = False

        while time.monotonic() < deadline:
            now = time.monotonic()
            # 目標方位を更新するかどうかの判定
            should_update_target = (
                self.last_target_bearing is None
                or now - last_target_update >= target_update_interval
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
                    # ステータスコールバックに現在地と目標までの距離を通知する
                    if status_callback is not None:
                        status_callback(
                            f"現在地: lat={latitude:.7f}, lon={longitude:.7f}, "
                            f"目標まで {distance_m:.1f} m, 方位 {bearing_deg:.1f} deg"
                        )
                    # ゴール判定
                    if distance_m <= goal_radius_m:
                        driver.stop()
                        return True
                elif (
                    self.last_target_bearing is None
                    or now - self.last_valid_gnss_time >= gnss_lost_grace_s
                ):
                    # GNSSロストが続いたら停止して復帰を待つ
                    if moving:
                        driver.ramp_stop_forward(
                            left_speed,
                            right_speed,
                            steps=stop_ramp_steps,
                            interval=stop_ramp_interval,
                        )
                        moving = False
                    if not waiting_for_gnss and status_callback is not None:
                        status_callback("GNSS現在地が取得できません。取得できるまで停止します。")
                    waiting_for_gnss = True
                    time.sleep(min(gnss_retry_interval, max(0.0, deadline - time.monotonic())))
                    continue
                elif status_callback is not None:
                    status_callback(
                        f"GNSS取得失敗。{gnss_lost_grace_s:g}秒未満のため直近の方位を維持して走行を継続します。"
                    )

            # 最後に得た目標方位へPD制御で進む
            left_speed, right_speed, prev_error = self._drive_pd_toward_heading(
                driver,
                sensor_manager,
                target_heading=self.last_target_bearing,
                base_speed=base_speed,
                kp=kp,
                kd=kd,
                prev_error=prev_error,
                loop_interval=loop_interval,
            )
            moving = True

            time.sleep(loop_interval)

        driver.stop()
        return False

    # 開始時の方位を保ちながら一定時間前進する
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
        base_speed = float(base_speed)

        target = float(sensor_manager.get_heading_deg())
        prev_error = 0.0
        left_speed = base_speed
        right_speed = base_speed
        start_time = time.monotonic()

        try:
            driver.forward_differential(left_speed, right_speed)

            while time.monotonic() - start_time <= duration_time:
                left_speed, right_speed, prev_error = self._drive_pd_toward_heading(
                    driver,
                    sensor_manager,
                    target_heading=target,
                    base_speed=base_speed,
                    kp=kp,
                    kd=kd,
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

    # 短い停止処理で前進する
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

    # IMUの変化量を見ながら指定角度だけ旋回する
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

    # 赤色パラシュートが前方から消えるまで旋回して避ける
    def avoid_parachute(
        self,
        driver,
        sensor_manager,
        *,
        red_threshold=0.05,
        move_speed=60.0,
        move_duration_s=2.0,
        rotate_angle_deg=90.0,
        rotate_speed=30.0,
        rotate_tolerance_deg=3.0,
        rotate_timeout_s=10.0,
        max_attempts=10,
        image_processor=None,
        capture_width=CAMERA_FULL_HD_WIDTH,
        capture_height=CAMERA_FULL_HD_HEIGHT,
        capture_hdr=False,
        capture_timeout_ms=2000,
    ):
        """前方カメラ画像から赤色パラシュートを検知し、赤色が消えるまで90度右旋回する。

        手順:
            1. 前方カメラ画像を撮影する。
            2. ImageProcessor.detect_red() で赤色を検知する。
            3. 赤色が検知されなければ、前方安全とみなして直進する。
            4. 赤色が検知されたら、rotate_by_angle() で時計回りに90度旋回する。
            5. 再度前方カメラ画像を撮影する。
            6. 赤色が検知されなくなるまで、撮影と90度旋回を繰り返す。

        注意:
            この処理はパラシュート回避テスト用です。
            本来はGPS目標方向へ復帰する処理が必要ですが、
            このテストでは赤色が見えなくなったらそのまま直進します。
        """

        if image_processor is None:
            from image_processor import ImageProcessor
            processor = ImageProcessor()
        else:
            processor = image_processor

        history = []

        for attempt in range(1, max_attempts + 1):
            print(f"パラシュート回避: 赤色確認 {attempt}/{max_attempts}")

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

            is_red_detected = bool(red_result["is_red_detected"])
            total_red_ratio = float(red_result["total_red_ratio"])

            history.append({
                "attempt": attempt,
                "is_red_detected": is_red_detected,
                "total_red_ratio": total_red_ratio,
                "red_result": red_result,
            })

            print(
                "パラシュート回避: "
                f"red_detected={is_red_detected}, "
                f"total_red_ratio={total_red_ratio:.3f}, "
                f"threshold={red_threshold:.3f}"
            )

            # 赤色が検知されなければ、前方安全とみなして直進する
            if not is_red_detected:
                print("パラシュート回避: 赤色なし。直進します")

                try:
                    driver.drive(move_speed)
                    time.sleep(move_duration_s)
                finally:
                    driver.stop()

                return {
                    "action": "forward_clear",
                    "completed": True,
                    "attempts": attempt,
                    "red_detected": False,
                    "red_ratio": total_red_ratio,
                    "red_threshold": float(red_threshold),
                    "move_speed": move_speed,
                    "move_duration_s": move_duration_s,
                    "rotate_angle_deg": rotate_angle_deg,
                    "rotate_speed": rotate_speed,
                    "last_red_result": red_result,
                    "history": history,
                }

            # 赤色が検知されたら時計回りに90度旋回する
            print(
                "パラシュート回避: "
                f"赤色を検知しました。時計回りに{rotate_angle_deg:.1f}度旋回します"
            )

            rotate_result = self.rotate_by_angle(
                driver,
                sensor_manager,
                rotate_angle_deg,
                speed=rotate_speed,
                tolerance_deg=rotate_tolerance_deg,
                timeout_s=rotate_timeout_s,
            )

            history[-1]["rotate_result"] = rotate_result

            print(
                "パラシュート回避: 旋回結果 "
                f"target={rotate_result['target_angle_deg']:.1f}, "
                f"rotated={rotate_result['rotated_angle_deg']:.1f}, "
                f"reached={rotate_result['reached']}"
            )

            time.sleep(0.2)

        # 最大試行回数まで赤色が消えなかった場合
        print("パラシュート回避: 最大試行回数まで赤色が消えませんでした。停止します")
        driver.stop()

        last = history[-1] if history else None

        return {
            "action": "failed_red_still_detected",
            "completed": False,
            "attempts": max_attempts,
            "red_detected": True if last is None else last["is_red_detected"],
            "red_ratio": None if last is None else last["total_red_ratio"],
            "red_threshold": float(red_threshold),
            "move_speed": move_speed,
            "move_duration_s": move_duration_s,
            "rotate_angle_deg": rotate_angle_deg,
            "rotate_speed": rotate_speed,
            "last_red_result": None if last is None else last["red_result"],
            "history": history,
        }

    # 赤コーンを画像で探して近づく
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
        forward_duration_by_red_ratio = tuple(sorted(forward_duration_by_red_ratio, reverse=True))

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

    # カメラ画像内に赤コーンが入るまで探索する
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
    def _drive_pd_toward_heading(
        self,
        driver,
        sensor_manager,
        target_heading,
        base_speed,
        kp,
        kd,
        prev_error,
        loop_interval,
    ):
        current = float(sensor_manager.get_heading_deg())
        error = self.heading_error(current, target_heading)
        d_error = (error - prev_error) / loop_interval
        correction = kp * error + kd * d_error

        left_speed = max(0.0, min(100.0, base_speed - correction))
        right_speed = max(0.0, min(100.0, base_speed + correction))
        driver.forward_differential(left_speed, right_speed)
        return left_speed, right_speed, error

    # 2つの方位の最短角度差を求める
    @staticmethod
    def heading_error(current, target):
        """現在方位と目標方位の最短角度差を-180度から+180度で返す。"""
        return (current - target + 180.0) % 360.0 - 180.0

