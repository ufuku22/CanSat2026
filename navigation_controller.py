import math
import time

from sensor_manager import CAMERA_FULL_HD_HEIGHT, CAMERA_FULL_HD_WIDTH


class NavigationController:

    DEFAULT_TARGET_LATITUDE_DEG = 35.0        #目標緯度
    DEFAULT_TARGET_LONGITUDE_DEG = 139.0      #目標経度
    PD_KP = 0.80
    PD_KD = 0.05
    CAPTURE_WIDTH = CAMERA_FULL_HD_WIDTH
    CAPTURE_HEIGHT = CAMERA_FULL_HD_HEIGHT
    CAPTURE_HDR = True
    CAPTURE_TIMEOUT_MS = 2000
    FOLLOW_TARGET_TIMEOUT_S = 120.0
    FOLLOW_TARGET_GOAL_RADIUS_M = 3.0
    FOLLOW_TARGET_BASE_SPEED = 70.0
    FOLLOW_TARGET_LOOP_INTERVAL = 0.02
    FOLLOW_TARGET_UPDATE_INTERVAL = 1.0
    FOLLOW_TARGET_STOP_RAMP_STEPS = 100
    FOLLOW_TARGET_STOP_RAMP_INTERVAL = 0.02
    FOLLOW_TARGET_GNSS_LOST_GRACE_S = 6.0
    FOLLOW_TARGET_GNSS_RETRY_INTERVAL = 1.0
    STUCK_AVOIDANCE_ENABLED = True
    STUCK_ACCEL_X_UPPER_MPS2 = 0.30   #X軸スタック判定の閾値の上限
    STUCK_ACCEL_Y_UPPER_MPS2 = 0.30   #Y軸スタック判定の閾値の上限
    STUCK_DETECTION_DURATION_S = 2.0
    STUCK_SAMPLE_INTERVAL_S = 0.05
    STUCK_REVERSE_SPEED = 60.0        
    STUCK_REVERSE_DURATION_S = 1.0
    STUCK_RIGHT_TURN_SPEED = 30.0
    STUCK_RIGHT_TURN_90_DURATION_S = 1.0
    STUCK_FORWARD_SPEED = 60.0
    STUCK_FORWARD_DURATION_S = 1.5
    AVOID_PARACHUTE_RED_THRESHOLD = 0.01  #赤検知の割合[%]
    AVOID_PARACHUTE_MOVE_SPEED = 100.0    
    AVOID_PARACHUTE_MOVE_DURATION_S = 3.0
    AVOID_PARACHUTE_ROTATE_ANGLE_DEG = 90.0
    AVOID_PARACHUTE_ROTATE_SPEED = 30.0
    AVOID_PARACHUTE_ROTATE_TOLERANCE_DEG = 3.0
    AVOID_PARACHUTE_ROTATE_TIMEOUT_S = 10.0
    AVOID_PARACHUTE_MAX_ATTEMPTS = 10
    RED_CONE_RED_THRESHOLD = 0.001
    RED_CONE_GOAL_CENTER_THRESHOLD = 0.90
    RED_CONE_RED_BLOCK_THRESHOLD = 0.005
    RED_CONE_SCAN_ANGLE_DEG = 60.0
    RED_CONE_CAMERA_FOV_DEG = 75.0
    RED_CONE_MAX_SCAN_STEPS = 6
    RED_CONE_MAX_STEPS = 30
    RED_CONE_FORWARD_DURATION_S = 1.5
    RED_CONE_FORWARD_DURATION_BY_RED_RATIO = (
        (0.30, 0.10),
        (0.25, 0.15),
        (0.20, 0.20),
        (0.10, 0.50),
        (0.05, 0.80),
    )
    RED_CONE_FORWARD_SPEED = 60.0
    RED_CONE_STOP_RAMP_STEPS = 8
    RED_CONE_STOP_RAMP_INTERVAL = 0.01
    RED_CONE_GOAL_FINAL_FORWARD_DURATION_S = 0.30
    RED_CONE_ROTATE_SPEED = 30.0
    RED_CONE_ROTATE_TOLERANCE_DEG = 3.0
    RED_CONE_ROTATE_TIMEOUT_S = 10.0
    RED_CONE_LOOP_INTERVAL = 0.10

    # 目標座標を保持する
    def __init__(
        self,
        target_latitude_deg=DEFAULT_TARGET_LATITUDE_DEG,
        target_longitude_deg=DEFAULT_TARGET_LONGITUDE_DEG,
    ):
        self.target_latitude_deg = float(target_latitude_deg)
        self.target_longitude_deg = float(target_longitude_deg)
        self._stuck_candidate_since = None
        self._stuck_last_sample_time = None

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
        pulse_time = 1.0
        for _ in range(3):
            accel_x = float(sensor_manager.get_imu()["accel_mps2"][0])
            if abs(accel_x) < 7.0:
                break
            driver.flip(pulse_time=pulse_time)
            pulse_time += 0.5
            time.sleep(3.0)

        for _ in range(3):
            accel_y = float(sensor_manager.get_imu()["accel_mps2"][1])
            if accel_y > -7.0:
                break
            driver.reverse_stabilizer(speed=50)
            time.sleep(3.0)

        for _ in range(3):
            accel_z = float(sensor_manager.get_imu()["accel_mps2"][2])
            if accel_z > -7.0:
                break
            driver.reverse_stabilizer()
            time.sleep(3.0)

    # 加速度が一定時間変化しない場合にスタックから離脱する
    def avoid_stuck(
        self,
        driver,
        sensor_manager,
    ):
        """X/Y加速度を1回確認し、継続時間に応じてスタックから離脱する。

        このメソッドを走行制御ループから繰り返し呼び出す。X/Y加速度の
        絶対値がそれぞれの設定上限以下である状態が指定時間継続したら、
        スタックと判定して後退、右90度旋回、直進の順に動作する。

        1回の呼び出しでは最大1サンプルだけ取得するため、走行中のPD制御を
        停止させない。スタックを検知して離脱動作を行った場合だけTrueを返す。
        """
        now = time.monotonic()
        if (
            self._stuck_last_sample_time is not None
            and now - self._stuck_last_sample_time < self.STUCK_SAMPLE_INTERVAL_S
        ):
            return False
        self._stuck_last_sample_time = now

        try:
            accel = sensor_manager.get_imu()["accel_mps2"]
            accel_x = float(accel[0])
            accel_y = float(accel[1])
        except (KeyError, TypeError, IndexError, ValueError) as exc:
            self._reset_stuck_detection()
            raise RuntimeError("IMUからX/Y加速度を取得できません") from exc

        accel_in_range = all(
            magnitude <= upper
            for magnitude, upper in (
                (abs(accel_x), self.STUCK_ACCEL_X_UPPER_MPS2),
                (abs(accel_y), self.STUCK_ACCEL_Y_UPPER_MPS2),
            )
        )
        if not accel_in_range:
            self._stuck_candidate_since = None
            return False

        self._stuck_candidate_since = (
            now
            if self._stuck_candidate_since is None
            else self._stuck_candidate_since
        )
        if now - self._stuck_candidate_since < self.STUCK_DETECTION_DURATION_S:
            return False

        self._reset_stuck_detection()

        print(
            "スタック検知: "
            f"accel_x={accel_x:+.3f} m/s^2, "
            f"accel_y={accel_y:+.3f} m/s^2"
        )

        self._run_stuck_escape(driver)
        return True

    def _run_stuck_escape(self, driver):
        """設定された時間で後退、右旋回、直進を順番に実行する。"""
        phases = (
            (
                f"{self.STUCK_REVERSE_DURATION_S:g}秒後退します",
                driver.drive,
                -self.STUCK_REVERSE_SPEED,
                self.STUCK_REVERSE_DURATION_S,
            ),
            (
                f"{self.STUCK_RIGHT_TURN_90_DURATION_S:g}秒右旋回して90度回頭します",
                driver.turn_right,
                self.STUCK_RIGHT_TURN_SPEED,
                self.STUCK_RIGHT_TURN_90_DURATION_S,
            ),
            (
                f"{self.STUCK_FORWARD_DURATION_S:g}秒直進します",
                driver.drive,
                self.STUCK_FORWARD_SPEED,
                self.STUCK_FORWARD_DURATION_S,
            ),
        )

        for message, start_motion, speed, duration_s in phases:
            print(f"スタック離脱: {message}")
            try:
                start_motion(speed)
                time.sleep(duration_s)
            finally:
                driver.stop()

    def _reset_stuck_detection(self):
        """継続中のスタック判定時間を破棄する。"""
        self._stuck_candidate_since = None
        self._stuck_last_sample_time = None

    # GNSSで目標方位を更新しながらゴールまで走行する
    def follow_target(
        self,
        driver,
        sensor_manager,
        status_callback=None,
    ):
        """GNSS現在地を確認しながら目標地点までPD制御で走行する。"""
        base_speed = float(self.FOLLOW_TARGET_BASE_SPEED)

        # 初回実行時にlast_valid_gnss_timeとlast_target_bearingを初期化する
        if not hasattr(self, 'last_valid_gnss_time'):
            self.last_valid_gnss_time = time.monotonic() - self.FOLLOW_TARGET_GNSS_LOST_GRACE_S
        if not hasattr(self, 'last_target_bearing'):
            self.last_target_bearing = None

        deadline = time.monotonic() + self.FOLLOW_TARGET_TIMEOUT_S
        last_target_update = 0.0
        prev_error = 0.0
        left_speed = base_speed
        right_speed = base_speed
        moving = False
        waiting_for_gnss = False
        self._reset_stuck_detection()

        while time.monotonic() < deadline:
            now = time.monotonic()
            # 目標方位を更新するかどうかの判定
            should_update_target = (
                self.last_target_bearing is None
                or now - last_target_update >= self.FOLLOW_TARGET_UPDATE_INTERVAL
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
                    if distance_m <= self.FOLLOW_TARGET_GOAL_RADIUS_M:
                        self._reset_stuck_detection()
                        driver.stop()
                        return True
                elif (
                    self.last_target_bearing is None
                    or now - self.last_valid_gnss_time >= self.FOLLOW_TARGET_GNSS_LOST_GRACE_S
                ):
                    # GNSSロストが続いたら停止して復帰を待つ
                    if moving:
                        driver.ramp_stop_forward(
                            left_speed,
                            right_speed,
                            steps=self.FOLLOW_TARGET_STOP_RAMP_STEPS,
                            interval=self.FOLLOW_TARGET_STOP_RAMP_INTERVAL,
                        )
                        moving = False
                    self._reset_stuck_detection()
                    if not waiting_for_gnss and status_callback is not None:
                        status_callback("GNSS現在地が取得できません。取得できるまで停止します。")
                    waiting_for_gnss = True
                    time.sleep(min(self.FOLLOW_TARGET_GNSS_RETRY_INTERVAL, max(0.0, deadline - time.monotonic())))
                    continue
                elif status_callback is not None:
                    status_callback(
                        f"GNSS取得失敗。{self.FOLLOW_TARGET_GNSS_LOST_GRACE_S:g}秒未満のため直近の方位を維持して走行を継続します。"
                    )

            # 最後に得た目標方位へPD制御で進む
            left_speed, right_speed, prev_error = self._drive_pd_toward_heading(
                driver,
                sensor_manager,
                target_heading=self.last_target_bearing,
                base_speed=base_speed,
                prev_error=prev_error,
                loop_interval=self.FOLLOW_TARGET_LOOP_INTERVAL,
            )
            moving = True

            if self.STUCK_AVOIDANCE_ENABLED and self.avoid_stuck(
                driver,
                sensor_manager,
            ):
                if status_callback is not None:
                    status_callback("スタック離脱完了。GPS誘導を再開します。")
                prev_error = 0.0
                left_speed = base_speed
                right_speed = base_speed
                moving = False
                last_target_update = 0.0
                continue

            time.sleep(self.FOLLOW_TARGET_LOOP_INTERVAL)

        self._reset_stuck_detection()
        driver.stop()
        return False

    # 開始時の方位を保ちながら一定時間前進する
    def follow_forward(
        self,
        driver,
        sensor_manager,
        duration_time,
        base_speed=100.0,
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
        speed=30.0,
        tolerance_deg=3.0,
        timeout_s=10.0,
        loop_interval=0.01,
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

    # 赤色パラシュートが前方から消えるまで旋回して避ける
    def avoid_parachute(
        self,
        driver,
        sensor_manager,
        image_processor=None,
    ):
        """前方カメラ画像から赤色パラシュートを検知し、赤色が消えるまで90度右旋回する。

        手順:
            1. 前方カメラ画像を撮影する。
            2. ImageProcessor.detect_color() で赤色を検知する。
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

        for attempt in range(1, self.AVOID_PARACHUTE_MAX_ATTEMPTS + 1):
            print(f"パラシュート回避: 赤色確認 {attempt}/{self.AVOID_PARACHUTE_MAX_ATTEMPTS}")

            frame = sensor_manager.capture_front_frame(
                width=self.CAPTURE_WIDTH,
                height=self.CAPTURE_HEIGHT,
                hdr=self.CAPTURE_HDR,
                timeout_ms=self.CAPTURE_TIMEOUT_MS,
            )

            red_result = processor.detect_color(
                frame,
                hsv_ranges=processor.ORANGE_HSV_RANGES,
                color_threshold=self.AVOID_PARACHUTE_RED_THRESHOLD,
            )

            is_red_detected = bool(red_result["is_color_detected"])
            total_red_ratio = float(red_result["total_color_ratio"])

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
                f"threshold={self.AVOID_PARACHUTE_RED_THRESHOLD:.3f}"
            )

            # 赤色が検知されなければ、前方安全とみなして直進する
            if not is_red_detected:
                print("パラシュート回避: 赤色なし。直進します")

                try:
                    driver.drive(self.AVOID_PARACHUTE_MOVE_SPEED)
                    time.sleep(self.AVOID_PARACHUTE_MOVE_DURATION_S)
                finally:
                    driver.stop()

                return {
                    "action": "forward_clear",
                    "completed": True,
                    "attempts": attempt,
                    "red_detected": False,
                    "red_ratio": total_red_ratio,
                    "red_threshold": float(self.AVOID_PARACHUTE_RED_THRESHOLD),
                    "move_speed": self.AVOID_PARACHUTE_MOVE_SPEED,
                    "move_duration_s": self.AVOID_PARACHUTE_MOVE_DURATION_S,
                    "rotate_angle_deg": self.AVOID_PARACHUTE_ROTATE_ANGLE_DEG,
                    "rotate_speed": self.AVOID_PARACHUTE_ROTATE_SPEED,
                    "last_red_result": red_result,
                    "history": history,
                }

            # 赤色が検知されたら時計回りに90度旋回する
            print(
                "パラシュート回避: "
                f"赤色を検知しました。時計回りに{self.AVOID_PARACHUTE_ROTATE_ANGLE_DEG:.1f}度旋回します"
            )

            rotate_result = self.rotate_by_angle(
                driver,
                sensor_manager,
                self.AVOID_PARACHUTE_ROTATE_ANGLE_DEG,
                speed=self.AVOID_PARACHUTE_ROTATE_SPEED,
                tolerance_deg=self.AVOID_PARACHUTE_ROTATE_TOLERANCE_DEG,
                timeout_s=self.AVOID_PARACHUTE_ROTATE_TIMEOUT_S,
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
            "attempts": self.AVOID_PARACHUTE_MAX_ATTEMPTS,
            "red_detected": True if last is None else last["is_red_detected"],
            "red_ratio": None if last is None else last["total_red_ratio"],
            "red_threshold": float(self.AVOID_PARACHUTE_RED_THRESHOLD),
            "move_speed": self.AVOID_PARACHUTE_MOVE_SPEED,
            "move_duration_s": self.AVOID_PARACHUTE_MOVE_DURATION_S,
            "rotate_angle_deg": self.AVOID_PARACHUTE_ROTATE_ANGLE_DEG,
            "rotate_speed": self.AVOID_PARACHUTE_ROTATE_SPEED,
            "last_red_result": None if last is None else last["red_result"],
            "history": history,
        }

    # 赤コーンを画像で探して近づく
    def guide_to_red_cone(
        self,
        driver,
        sensor_manager,
        image_processor=None,
    ):
        """画像で赤コーンを探し、正面へ回頭して一定時間前進する。"""
        if image_processor is None:
            from image_processor import ImageProcessor

            processor = ImageProcessor()
        else:
            processor = image_processor
        forward_duration_by_red_ratio = tuple(
            sorted(self.RED_CONE_FORWARD_DURATION_BY_RED_RATIO, reverse=True)
        )

        history = []
        last_goal_result = None

        for step in range(self.RED_CONE_MAX_STEPS):
            print(f"赤コーン誘導: step {step + 1}/{self.RED_CONE_MAX_STEPS} 探索開始")

            # 1. 赤コーンが画面に入るまで、撮影と少しの旋回を繰り返す。
            _found_frame, red_result, scan_history = self._find_red_cone_in_view(
                driver,
                sensor_manager,
                processor,
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
            turn_angle = self._red_direction_to_turn_angle(
                red_result["color_direction"],
                self.RED_CONE_CAMERA_FOV_DEG,
            )
            turn_result = None
            if turn_angle != 0.0:
                print(f"赤コーン誘導: {turn_angle:.1f}度旋回します")
                turn_result = self.rotate_by_angle(
                    driver,
                    sensor_manager,
                    turn_angle,
                    speed=self.RED_CONE_ROTATE_SPEED,
                    tolerance_deg=self.RED_CONE_ROTATE_TOLERANCE_DEG,
                    timeout_s=self.RED_CONE_ROTATE_TIMEOUT_S,
                )
            else:
                print("赤コーン誘導: 旋回なし")

            # 3. 赤色が大きく見えているほど近いとみなし、前進時間を短くする。
            forward_duration = self._red_cone_forward_duration(
                red_result["total_color_ratio"],
                self.RED_CONE_FORWARD_DURATION_S,
                forward_duration_by_red_ratio,
            )

            print(
                "赤コーン誘導: "
                f"前進 {forward_duration:.2f}秒 "
                f"(total={red_result['total_color_ratio'] * 100:.2f}%, "
                f"direction={red_result['color_direction']})"
            )
            self.follow_forward(
                driver,
                sensor_manager,
                forward_duration,
                base_speed=self.RED_CONE_FORWARD_SPEED,
                loop_interval=self.RED_CONE_LOOP_INTERVAL,
                stop_ramp_steps=self.RED_CONE_STOP_RAMP_STEPS,
                stop_ramp_interval=self.RED_CONE_STOP_RAMP_INTERVAL,
            )

            # 4. 前進後にもう一度撮影し、赤コーンに十分近づいたか判定する。
            print("赤コーン誘導: ゴール判定用に撮影します")
            goal_frame = sensor_manager.capture_front_frame(
                width=self.CAPTURE_WIDTH,
                height=self.CAPTURE_HEIGHT,
                hdr=self.CAPTURE_HDR,
                timeout_ms=self.CAPTURE_TIMEOUT_MS,
            )
            last_goal_result = processor.judge_red_goal_reached(
                goal_frame,
                red_threshold=self.RED_CONE_RED_THRESHOLD,
                goal_center_threshold=self.RED_CONE_GOAL_CENTER_THRESHOLD,
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

            # ゴール判定が出たら、最後に少し前進して終了する
            if last_goal_result["goal_reached"]:
                print(
                    "赤コーン誘導: "
                    f"ゴール判定成功。最後に{self.RED_CONE_GOAL_FINAL_FORWARD_DURATION_S:.2f}秒前進します"
                )
                self.follow_forward(
                    driver,
                    sensor_manager,
                    self.RED_CONE_GOAL_FINAL_FORWARD_DURATION_S,
                    base_speed=self.RED_CONE_FORWARD_SPEED,
                    loop_interval=self.RED_CONE_LOOP_INTERVAL,
                    stop_ramp_steps=self.RED_CONE_STOP_RAMP_STEPS,
                    stop_ramp_interval=self.RED_CONE_STOP_RAMP_INTERVAL,
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
            "steps": self.RED_CONE_MAX_STEPS,
            "history": history,
            "last_goal_result": last_goal_result,
        }

    # カメラ画像内に赤コーンが入るまで探索する
    def _find_red_cone_in_view(
        self,
        driver,
        sensor_manager,
        processor,
    ):
        scan_history = []
        for scan_index in range(self.RED_CONE_MAX_SCAN_STEPS):
            # 正面画像から赤色の量と方向を確認する。
            print(
                "赤コーン探索: "
                f"scan {scan_index + 1}/{self.RED_CONE_MAX_SCAN_STEPS} 撮影します"
            )
            frame = sensor_manager.capture_front_frame(
                width=self.CAPTURE_WIDTH,
                height=self.CAPTURE_HEIGHT,
                hdr=self.CAPTURE_HDR,
                timeout_ms=self.CAPTURE_TIMEOUT_MS,
            )
            red_result = processor.detect_color(
                frame,
                hsv_ranges=processor.RED_HSV_RANGES,
                color_threshold=self.RED_CONE_RED_THRESHOLD,
                block_threshold=self.RED_CONE_RED_BLOCK_THRESHOLD,
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

            if float(red_result["total_color_ratio"]) > self.RED_CONE_RED_THRESHOLD:
                print("赤コーン探索: 赤コーンを検出しました")
                return frame, red_result, scan_history

            # 赤コーンが見つからなければ、次の撮影前に一定角度だけ向きを変える。
            if scan_index < self.RED_CONE_MAX_SCAN_STEPS - 1:
                print(
                    "赤コーン探索: "
                    f"赤コーンなし。{self.RED_CONE_SCAN_ANGLE_DEG:.1f}度旋回して再探索します"
                )
                self.rotate_by_angle(
                    driver,
                    sensor_manager,
                    self.RED_CONE_SCAN_ANGLE_DEG,
                    speed=self.RED_CONE_ROTATE_SPEED,
                    tolerance_deg=self.RED_CONE_ROTATE_TOLERANCE_DEG,
                    timeout_s=self.RED_CONE_ROTATE_TIMEOUT_S,
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
        prev_error,
        loop_interval,
    ):
        current = float(sensor_manager.get_heading_deg())
        error = self.heading_error(current, target_heading)
        d_error = (error - prev_error) / loop_interval
        correction = self.PD_KP * error + self.PD_KD * d_error

        left_speed = max(0.0, min(100.0, base_speed - correction))
        right_speed = max(0.0, min(100.0, base_speed + correction))
        driver.forward_differential(left_speed, right_speed)
        return left_speed, right_speed, error

    # 2つの方位の最短角度差を求める
    @staticmethod
    def heading_error(current, target):
        """現在方位と目標方位の最短角度差を-180度から+180度で返す。"""
        return (current - target + 180.0) % 360.0 - 180.0
