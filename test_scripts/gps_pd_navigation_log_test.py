#!/usr/bin/env python3
"""GPS PD navigation test with GNSS and heading CSV logging."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import FollowTargetConfig, NavigationPdConfig
from drive_controller import DriveController
from logger import GnssNavigationCsvLogger
from navigation_controller import NavigationController
from sensor_manager import SensorManager


class LoggingNavigationController(NavigationController):
    """既存のGPS誘導に、1回のGNSS取得につき1行のCSV記録を追加する。"""

    def drive_toward_heading(
        self,
        driver,
        sensor_manager,
        target_heading,
        base_speed,
        prev_error=0.0,
        loop_interval=0.1,
    ):
        # 元のdrive_toward_headingと同じ計算を行い、IMU方位を二重取得しない。
        current_heading = float(sensor_manager.get_heading_deg())
        error = self.heading_error(current_heading, target_heading)
        d_error = (error - prev_error) / loop_interval
        correction = self.pd_config.KP * error + self.pd_config.KD * d_error

        left_speed = max(0.0, min(100.0, float(base_speed) - correction))
        right_speed = max(0.0, min(100.0, float(base_speed) + correction))
        driver.forward_differential(left_speed, right_speed)

        # get_gnss()で一時保存した同じ位置情報を記録する。
        # 新たなGNSS取得は行わない。
        pending = getattr(sensor_manager, "_pending_sample", None)
        if pending is not None:
            distance_m = self.distance_to_target_m(
                pending["latitude_deg"],
                pending["longitude_deg"],
            )
            sensor_manager.record_navigation(
                distance_to_goal_m=distance_m,
                heading_deg=current_heading,
            )

        return left_speed, right_speed, error


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="GPS goal navigation test using PD control with CSV logging."
    )
    parser.add_argument("--timeout", type=float, default=FollowTargetConfig.TIMEOUT_S)
    parser.add_argument("--goal-radius", type=float, default=FollowTargetConfig.GOAL_RADIUS_M)
    parser.add_argument("--base-speed", type=float, default=FollowTargetConfig.BASE_SPEED)
    parser.add_argument("--kp", type=float, default=NavigationPdConfig.KP)
    parser.add_argument("--kd", type=float, default=NavigationPdConfig.KD)
    parser.add_argument(
        "--loop-interval",
        type=float,
        default=FollowTargetConfig.LOOP_INTERVAL_S,
    )
    parser.add_argument(
        "--target-update-interval",
        type=float,
        default=FollowTargetConfig.TARGET_UPDATE_INTERVAL_S,
    )
    parser.add_argument(
        "--gnss-lost-grace",
        type=float,
        default=FollowTargetConfig.GNSS_LOST_GRACE_S,
    )
    parser.add_argument(
        "--log-path",
        type=Path,
        default=None,
        help="CSV path. Default: logs/gps_pd_navigation_YYYYmmdd_HHMMSS.csv",
    )
    return parser.parse_args()


def prompt_float(label: str, *, min_value: float, max_value: float) -> float:
    while True:
        raw = input(f"{label}を入力してください: ").strip()
        try:
            value = float(raw)
        except ValueError:
            print("数値で入力してください。")
            continue
        if min_value <= value <= max_value:
            return value
        print(f"{min_value} から {max_value} の範囲で入力してください。")


def setup_navigation_sensors(sensors: SensorManager) -> None:
    sensors.imu.setup()
    sensors.gnss.setup()


def main() -> int:
    args = parse_args()
    target_latitude = prompt_float("目標緯度", min_value=-90.0, max_value=90.0)
    target_longitude = prompt_float("目標経度", min_value=-180.0, max_value=180.0)

    navigator = LoggingNavigationController(
        target_latitude_deg=target_latitude,
        target_longitude_deg=target_longitude,
    )
    navigator.follow_target_config.TIMEOUT_S = args.timeout
    navigator.follow_target_config.GOAL_RADIUS_M = args.goal_radius
    navigator.follow_target_config.BASE_SPEED = args.base_speed
    navigator.pd_config.KP = args.kp
    navigator.pd_config.KD = args.kd
    navigator.follow_target_config.LOOP_INTERVAL_S = args.loop_interval
    navigator.follow_target_config.TARGET_UPDATE_INTERVAL_S = args.target_update_interval
    navigator.follow_target_config.GNSS_LOST_GRACE_S = args.gnss_lost_grace

    log_path = args.log_path or (
        PROJECT_ROOT
        / "logs"
        / f"gps_pd_navigation_{datetime.now():%Y%m%d_%H%M%S}.csv"
    )

    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()
        setup_navigation_sensors(sensors)

        with GnssNavigationCsvLogger(
            sensors,
            log_path,
            goal_latitude_deg=target_latitude,
            goal_longitude_deg=target_longitude,
        ) as logged_sensors:
            print(
                f"目標座標: lat={target_latitude:.7f}, lon={target_longitude:.7f} / "
                f"判定半径: {args.goal_radius:g} m / タイムアウト: {args.timeout:g} 秒"
            )
            print(f"CSVログ: {log_path}")

            def avoid_stuck_during_navigation() -> bool:
                return navigator.avoid_stuck(driver, logged_sensors)

            reached_goal = navigator.follow_target(
                driver,
                logged_sensors,
                status_callback=print,
                stuck_avoidance_callback=avoid_stuck_during_navigation,
            )

        print("ゴール成功" if reached_goal else "ゴール失敗")
        return 0 if reached_goal else 1

    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("ゴール失敗")
        return 130
    finally:
        if sensors is not None:
            sensors.close()
        if driver is not None:
            driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
