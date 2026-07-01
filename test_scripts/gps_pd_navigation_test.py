#!/usr/bin/env python3
"""Drive toward a GPS goal with NavigationController PD control."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_controller import NavigationController
from sensor_manager import SensorManager


DEFAULT_TIMEOUT_S = 120.0
DEFAULT_GOAL_RADIUS_M = 5.0
DEFAULT_STEP_DURATION_S = 10
DEFAULT_BASE_SPEED = 100.0
DEFAULT_KP = 0.80
DEFAULT_KD = 0.05
GNSS_RETRY_COUNT = 50
GNSS_RETRY_INTERVAL_S = 5.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GPS goal navigation test using PD control.")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="timeout seconds")
    parser.add_argument("--goal-radius", type=float, default=DEFAULT_GOAL_RADIUS_M, help="goal radius meters")
    parser.add_argument("--step-duration", type=float, default=DEFAULT_STEP_DURATION_S, help="PD drive chunk seconds")
    parser.add_argument("--base-speed", type=float, default=DEFAULT_BASE_SPEED, help="base motor speed percent")
    parser.add_argument("--kp", type=float, default=DEFAULT_KP, help="PD proportional gain")
    parser.add_argument("--kd", type=float, default=DEFAULT_KD, help="PD derivative gain")
    parser.add_argument("--loop-interval", type=float, default=0.10, help="PD loop interval seconds")
    parser.add_argument("--target-update-interval", type=float, default=1.0, help="target bearing update interval seconds")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.goal_radius <= 0:
        parser.error("--goal-radius must be positive")
    if args.step_duration <= 0:
        parser.error("--step-duration must be positive")
    if args.loop_interval <= 0:
        parser.error("--loop-interval must be positive")
    if args.target_update_interval <= 0:
        parser.error("--target-update-interval must be positive")
    return args


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


def read_current_position(
    sensors: SensorManager,
    driver: DriveController | None = None,
) -> tuple[float, float] | None:
    for attempt in range(GNSS_RETRY_COUNT):
        gnss = sensors.get_gnss()
        latitude = gnss.get("latitude_deg")
        longitude = gnss.get("longitude_deg")
        if latitude is not None and longitude is not None:
            return float(latitude), float(longitude)

        if attempt == 0:
            if driver is not None:
                driver.stop()
            print(f"GPS現在地が取得できません。最大{GNSS_RETRY_COUNT}回まで取得を試みます。")

        if attempt < GNSS_RETRY_COUNT - 1:
            time.sleep(GNSS_RETRY_INTERVAL_S)

    return None


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
    driver: DriveController | None = None
    sensors: SensorManager | None = None
    deadline = time.monotonic() + args.timeout
    may_be_moving = False

    try:
        driver = DriveController()
        sensors = SensorManager()
        setup_navigation_sensors(sensors)
        print(
            f"目標座標: lat={target_latitude:.7f}, lon={target_longitude:.7f} / "
            f"判定半径: {args.goal_radius:g} m / タイムアウト: {args.timeout:g} 秒"
        )

        while time.monotonic() < deadline:
            position = read_current_position(sensors, driver if may_be_moving else None)
            if position is None:
                may_be_moving = False
                print("GPS現在地が取得できません。取得できるまで待機します。")
                continue

            latitude, longitude = position
            distance_m = navigator.distance_to_target_m(latitude, longitude)
            bearing_deg = navigator.bearing_to_target(latitude, longitude)
            print(
                f"現在地: lat={latitude:.7f}, lon={longitude:.7f}, "
                f"目標まで {distance_m:.1f} m, 方位 {bearing_deg:.1f} deg"
            )
            if distance_m <= args.goal_radius:
                driver.stop()
                may_be_moving = False
                print("ゴール成功")
                return 0

            remaining_s = max(0.0, deadline - time.monotonic())
            drive_duration = min(args.step_duration, remaining_s)
            if drive_duration <= 0:
                break

            navigator.follow_target(
                driver,
                sensors,
                drive_duration,
                base_speed=args.base_speed,
                kp=args.kp,
                kd=args.kd,
                loop_interval=args.loop_interval,
                target_update_interval=args.target_update_interval,
                stop_ramp_steps=20,
                stop_ramp_interval=0.01,
            )
            may_be_moving = True

        driver.stop()
        print("ゴール失敗")
        return 1

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
