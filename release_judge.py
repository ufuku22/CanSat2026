#!/usr/bin/env python3
"""CanSat2026の放出判定を行うモジュール。"""

from __future__ import annotations

import time

from sensor_manager import SensorManager


# 放出判定の調整値。実験結果に合わせてここを書き換える。
PRESSURE_RISE_THRESHOLD_HPA = 5.0
ACCEL_Z_THRESHOLD_MPS2 = 3.0
REQUIRED_CONSECUTIVE_COUNT = 3
PRESSURE_TIMEOUT_S = 60.0
ACCEL_TIMEOUT_S = 60.0
MEASUREMENT_INTERVAL_S = 0.2


class ReleaseJudge:
    """センサの値を使って放出判定を行うクラス。"""

    def __init__(self, sensor_manager: SensorManager) -> None:
        self.sensor_manager = sensor_manager

    def judge_release(self) -> bool:
        """気圧と9軸センサの値から放出を判定する。"""
        # 先に気圧上昇を確認し、その後z方向加速度で放出状態を確認する。
        self.wait_for_pressure_rise()

        if self.wait_for_small_z_acceleration():
            print("放出成功")
            return True

        return False

    def wait_for_pressure_rise(self) -> bool:
        """気圧がしきい値以上に上昇するまで待つ。"""
        start_time = time.monotonic()
        base_pressure_hpa = float(self.sensor_manager.get_environment()["pressure_hpa"])
        consecutive_count = 0

        while time.monotonic() - start_time < PRESSURE_TIMEOUT_S:
            pressure_hpa = float(self.sensor_manager.get_environment()["pressure_hpa"])

            # 基準気圧からの上昇が連続して条件を満たすか確認する。
            if pressure_hpa - base_pressure_hpa >= PRESSURE_RISE_THRESHOLD_HPA:
                consecutive_count += 1
            else:
                consecutive_count = 0

            if consecutive_count >= REQUIRED_CONSECUTIVE_COUNT:
                return True

            time.sleep(MEASUREMENT_INTERVAL_S)

        return False

    def wait_for_small_z_acceleration(self) -> bool:
        """z方向加速度がしきい値以内になるまで待つ。"""
        start_time = time.monotonic()
        consecutive_count = 0

        while time.monotonic() - start_time < ACCEL_TIMEOUT_S:
            accel_z_mps2 = float(self.sensor_manager.get_imu()["accel_mps2"][2])

            # z方向加速度の絶対値が小さい状態が連続したら放出とみなす。
            if abs(accel_z_mps2) <= ACCEL_Z_THRESHOLD_MPS2:
                consecutive_count += 1
            else:
                consecutive_count = 0

            if consecutive_count >= REQUIRED_CONSECUTIVE_COUNT:
                return True

            time.sleep(MEASUREMENT_INTERVAL_S)

        return True
