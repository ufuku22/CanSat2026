#!/usr/bin/env python3
"""Drive toward a GPS goal with NavigationController PD control."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import FollowTargetConfig, NavigationPdConfig
from drive_controller import DriveController
from navigation_controller import NavigationController
from sensor_manager import SensorManager


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GPS goal navigation test using PD control.")
    parser.add_argument(
        "--timeout",
        type=float,
        default=FollowTargetConfig.TIMEOUT_S,
        help="timeout seconds",
    )
    parser.add_argument(
        "--goal-radius",
        type=float,
        default=FollowTargetConfig.GOAL_RADIUS_M,
        help="goal radius meters",
    )
    parser.add_argument(
        "--base-speed",
        type=float,
        default=FollowTargetConfig.BASE_SPEED,
        help="follow_target base_speed percent",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=NavigationPdConfig.KP,
        help="PD kp gain",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=NavigationPdConfig.KD,
        help="PD kd gain",
    )
    parser.add_argument(
        "--loop-interval",
        type=float,
        default=FollowTargetConfig.LOOP_INTERVAL_S,
        help="PD loop interval seconds",
    )
    parser.add_argument(
        "--target-update-interval",
        type=float,
        default=FollowTargetConfig.TARGET_UPDATE_INTERVAL_S,
        help="target bearing update interval seconds",
    )
    parser.add_argument(
        "--gnss-lost-grace",
        type=float,
        default=FollowTargetConfig.GNSS_LOST_GRACE_S,
        help="seconds to keep moving after GNSS is lost",
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

    navigator = NavigationController(
        target_latitude_deg=target_latitude,
        target_longitude_deg=target_longitude,
    )
    navigator.follow_target_config.TIMEOUT_S = args.timeout
    navigator.follow_target_config.GOAL_RADIUS_M = args.goal_radius
    navigator.follow_target_config.BASE_SPEED = args.base_speed
    navigator.pd_config.KP = args.kp
    navigator.pd_config.KD = args.kd
    navigator.follow_target_config.LOOP_INTERVAL_S = args.loop_interval
    navigator.follow_target_config.TARGET_UPDATE_INTERVAL_S = (
        args.target_update_interval
    )
    navigator.follow_target_config.GNSS_LOST_GRACE_S = args.gnss_lost_grace
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()
        setup_navigation_sensors(sensors)
        print(
            f"目標座標: lat={target_latitude:.7f}, lon={target_longitude:.7f} / "
            f"判定半径: {args.goal_radius:g} m / タイムアウト: {args.timeout:g} 秒"
        )

        def avoid_stuck_during_navigation() -> bool:
            return navigator.avoid_stuck(driver, sensors)

        reached_goal = navigator.follow_target(
            driver,
            sensors,
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
