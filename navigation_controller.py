import math
import numbers
import time


class NavigationController:
    """現在地から目標地点までの方位を計算する。"""

    DEFAULT_TARGET_LATITUDE_DEG = 35.0
    DEFAULT_TARGET_LONGITUDE_DEG = 139.0

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

    def follow_forward(
        self,
        driver,
        sensor_manager,
        duration_time,
        base_speed=80.0,
        kp=0.80,
        kd=0.05,
        loop_interval=0.10,
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

    def follow_target(
        self,
        driver,
        sensor_manager,
        duration_time,
        base_speed=80.0,
        kp=0.80,
        kd=0.05,
        loop_interval=0.10,
        target_update_interval=1.0,
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
