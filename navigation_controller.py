from collections import deque
import math
from statistics import median
import time

from config import (
    DriveControllerConfig,
    FollowTargetConfig,
    NavigationMotionConfig,
    NavigationPdConfig,
    ParachuteAvoidanceConfig,
    PostureRestoreConfig,
)


class NavigationController:

    # 目標座標を保持する
    def __init__(
        self,
        target_latitude_deg=None,
        target_longitude_deg=None,
        logger=None,
    ):
        self.target_latitude_deg = (
            None if target_latitude_deg is None else float(target_latitude_deg)
        )
        self.target_longitude_deg = (
            None if target_longitude_deg is None else float(target_longitude_deg)
        )
        self.logger = logger
        self.pd_config = NavigationPdConfig()
        self.posture_restore_config = PostureRestoreConfig()
        self.follow_target_config = FollowTargetConfig()
        self.parachute_avoidance_config = ParachuteAvoidanceConfig()

    def _log(self, message):
        """logger指定時はイベントログへ、未指定時は標準出力へ出す。"""
        if self.logger is not None:
            self.logger.event(message)
        else:
            print(message, flush=True)

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

    @staticmethod
    def _distance_between_positions_m(first, second):
        """2つのGNSS座標間の距離を返す。"""
        lat1, lon1 = map(math.radians, first)
        lat2, lon2 = map(math.radians, second)
        delta_lat = lat2 - lat1
        delta_lon = lon2 - lon1
        a = (
            math.sin(delta_lat / 2.0) ** 2
            + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon / 2.0) ** 2
        )
        return 6371000.0 * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    # 9軸センサの加速度から機体の姿勢を正常に戻す
    def restore_posture(self, driver, sensor_manager) -> bool:
        config = self.posture_restore_config
        time.sleep(0.5)
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

        accel_x, accel_y, accel_z = (
            float(value) for value in sensor_manager.get_imu()["accel_mps2"]
        )
        return (
            abs(accel_x) < config.ACCEL_THRESHOLD_MPS2
            and accel_y > -config.ACCEL_THRESHOLD_MPS2
            and accel_z > -config.ACCEL_THRESHOLD_MPS2
        )

    # GNSSで目標方位を更新しながらゴールまで走行する
    def follow_target(
        self,
        driver,
        sensor_manager,
        status_callback=None,
    ):
        """GNSS現在地を確認しながら目標地点までPD制御で走行する。"""
        if self.target_latitude_deg is None or self.target_longitude_deg is None:
            raise ValueError("GNSS誘導には目標緯度・経度の指定が必要です")

        config = self.follow_target_config
        base_speed = float(config.BASE_SPEED)

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
        stuck_positions = deque(
            maxlen=int(config.STUCK_WINDOW_S / config.TARGET_UPDATE_INTERVAL_S) + 1
        )
        stuck_detection_count = 0

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
                if position is None:
                    stuck_positions.clear()
                    stuck_detection_count = 0

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
                        driver.stop()
                        return True

                    stuck_positions.append(position)
                    if len(stuck_positions) == stuck_positions.maxlen:
                        positions = list(stuck_positions)
                        half = len(stuck_positions) // 2
                        old = tuple(
                            median(value) for value in zip(*positions[:half])
                        )
                        new = tuple(
                            median(value) for value in zip(*positions[-half:])
                        )
                        stuck_distance_m = self._distance_between_positions_m(old, new)
                        stuck_detection_count = (
                            stuck_detection_count + 1
                            if stuck_distance_m <= config.STUCK_DISPLACEMENT_THRESHOLD_M
                            else 0
                        )
                        if stuck_detection_count >= config.STUCK_DETECTION_LIMIT:
                            if status_callback is not None:
                                status_callback(
                                    f"スタックを検出しました。"
                                    f"{config.STUCK_WINDOW_S:g}秒間の変位="
                                    f"{stuck_distance_m:.2f} m"
                                )
                            self.stuck_escape(driver, sensor_manager)
                            stuck_positions.clear()
                            stuck_detection_count = 0
                            moving = False
                            prev_error = 0.0
                            last_target_update = 0.0
                            continue
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

            time.sleep(config.LOOP_INTERVAL_S)

        driver.stop()
        return False

    def _move_for_gnss_recovery(self, driver, sensor_manager):
        """現在方位を維持して短時間移動し、GNSSを再取得しやすい場所へ移る。"""
        config = self.follow_target_config
        self.pd_forward(
            driver,
            sensor_manager,
            config.GNSS_RECOVERY_MOVE_DURATION_S,
            base_speed=config.GNSS_RECOVERY_MOVE_SPEED,
            loop_interval=config.LOOP_INTERVAL_S,
        )

    # 開始時の方位を保ちながら一定時間前進する
    def pd_forward(
        self,
        driver,
        sensor_manager,
        duration_time,
        base_speed=NavigationMotionConfig.PD_FORWARD_BASE_SPEED,
        loop_interval=NavigationMotionConfig.PD_FORWARD_LOOP_INTERVAL_S,
        stop_ramp_steps=DriveControllerConfig.RAMP_STOP_STEPS,
        stop_ramp_interval=DriveControllerConfig.RAMP_STOP_INTERVAL_S,
        enable_head_swing=False,
    ):
        """開始時の方位を保ちながらPD制御で前進し、必要なら左右へ首振りする。"""
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

        if enable_head_swing:
            self.rotate_by_angle(driver, sensor_manager, 30.0)
            self.rotate_by_angle(driver, sensor_manager, -30.0)

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
        タイムアウト時は旋回スタック回避を行う。timeout_sがNoneの場合は、
        指定角度へ到達するまで待ち続ける。
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

        if not reached and timeout_s is not None:
            self.stuck_escape(driver, sensor_manager)

        return {
            "target_angle_deg": angle_deg,
            "rotated_angle_deg": rotated_angle,
            "reached": reached,
        }

    def stuck_escape(self, driver, sensor_manager):
        """旋回スタック時に後退、旋回、前進を最高出力で行う。"""
        config = NavigationMotionConfig

        try:
            driver.drive(-config.ROTATE_STUCK_ESCAPE_SPEED)
            time.sleep(config.ROTATE_STUCK_REVERSE_DURATION_S)
        finally:
            driver.stop()

        self.rotate_by_angle(
            driver,
            sensor_manager,
            config.ROTATE_STUCK_ANGLE_DEG,
            speed=config.ROTATE_STUCK_ESCAPE_SPEED,
        )

        try:
            driver.drive(config.ROTATE_STUCK_ESCAPE_SPEED)
            time.sleep(config.ROTATE_STUCK_FORWARD_DURATION_S)
        finally:
            driver.stop()

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
            processor = ImageProcessor(logger=self.logger)
        else:
            processor = image_processor

        config = self.parachute_avoidance_config
        self.restore_posture(driver, sensor_manager)
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
            self._log(
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
            self._log("パラシュート回避: 紫色なし。目標方向へ直進します")
            action = "forward_clear"

        self.pd_forward(
            driver,
            sensor_manager,
            config.MOVE_DURATION_S,
            base_speed=config.MOVE_SPEED,
            enable_head_swing=True,
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
        loop_interval=NavigationMotionConfig.PD_FORWARD_LOOP_INTERVAL_S,
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
