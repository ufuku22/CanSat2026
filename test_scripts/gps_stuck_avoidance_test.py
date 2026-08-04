#!/usr/bin/env python3
"""PD直進中にGPS位置の変化からスタック回避を確認する実機テスト。"""

from __future__ import annotations

from inspect import Parameter, signature
from pathlib import Path
import sys
import time
from typing import Any, Callable

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import MissionConfig, NavigationPdConfig, StuckAvoidanceConfig
from drive_controller import DriveController
from gpiozero import OutputDevice
from navigation_controller import NavigationController
from sensor_manager import SensorManager


FOLLOW_FORWARD_DEFAULT = NavigationController.follow_forward
ARM_SLEEP_PIN = 5


def navigation_default(method: Callable[..., Any], parameter_name: str) -> Any:
    parameter = signature(method).parameters[parameter_name]
    if parameter.default is Parameter.empty:
        raise ValueError(f"{method.__name__}.{parameter_name} has no default")
    return parameter.default


def input_float(label: str, default: float | None = None) -> float:
    while True:
        suffix = f" [{default:g}]" if default is not None else ""
        raw = input(f"{label}{suffix}: ").strip()
        if raw == "" and default is not None:
            value = default
        else:
            try:
                value = float(raw)
            except ValueError:
                print("数値で入力してください。")
                continue
        return value


def print_gnss(gnss: dict[str, Any]) -> None:
    if not gnss.get("has_fix"):
        print("GPS取得: Fixなし")
        return

    latitude = gnss.get("latitude_deg")
    longitude = gnss.get("longitude_deg")
    satellites = gnss.get("satellites")
    print(
        f"GPS取得: lat={latitude}, lon={longitude}, "
        f"satellites={satellites}"
    )


def follow_forward_with_gps_stuck_avoidance(
    navigator: NavigationController,
    driver: DriveController,
    sensors: SensorManager,
    speed: float,
    loop_interval_s: float,
    gnss_interval_s: float,
    tolerance_m: float,
) -> None:
    """PD直進を続け、GNSS更新周期ごとにGPSスタックを判定する。"""
    target_heading = float(sensors.get_heading_deg())
    prev_error = 0.0
    next_gnss_check_at = time.monotonic()
    avoidance_count = 0
    navigator._reset_stuck_detection()

    print(f"初期目標方位: {target_heading:.1f}度")
    try:
        while True:
            _, _, prev_error = navigator.drive_toward_heading(
                driver,
                sensors,
                target_heading=target_heading,
                base_speed=speed,
                prev_error=prev_error,
                loop_interval=loop_interval_s,
            )

            now = time.monotonic()
            if now >= next_gnss_check_at:
                # 先に表示用として取得し、判定ではSensorManagerの同じキャッシュを使う。
                print_gnss(sensors.get_gnss())
                avoided = navigator.avoid_stuck_by_gps(
                    driver,
                    sensors,
                    tolerance_m=tolerance_m,
                    required_consecutive_detections=2,
                )
                next_gnss_check_at = time.monotonic() + gnss_interval_s

                if avoided:
                    avoidance_count += 1
                    print(f"GPSスタック回避{avoidance_count}回目完了。PD直進を再開します")
                    current_heading = float(sensors.get_heading_deg())
                    prev_error = navigator.heading_error(
                        current_heading,
                        target_heading,
                    )

            time.sleep(loop_interval_s)
    except BaseException:
        driver.stop()
        raise
    finally:
        navigator._reset_stuck_detection()


def main() -> int:
    speed = input_float(
        "基準速度[%]",
        navigation_default(FOLLOW_FORWARD_DEFAULT, "base_speed"),
    )
    kp = input_float("Pゲイン", NavigationPdConfig.KP)
    kd = input_float("Dゲイン", NavigationPdConfig.KD)
    loop_interval_s = input_float(
        "制御周期[秒]",
        navigation_default(FOLLOW_FORWARD_DEFAULT, "loop_interval"),
    )
    tolerance_m = input_float(
        "GPSスタック判定の許容誤差[m]",
        StuckAvoidanceConfig.GPS_POSITION_TOLERANCE_M,
    )
    gnss_interval_s = float(MissionConfig.GNSS_CACHE_MAX_AGE_S)
    if gnss_interval_s <= 0.0:
        raise ValueError("GNSS_CACHE_MAX_AGE_S must be greater than 0")

    driver: DriveController | None = None
    sensors: SensorManager | None = None
    arm_sleep: OutputDevice | None = None

    try:
        arm_sleep = OutputDevice(ARM_SLEEP_PIN, active_high=True, initial_value=False)
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        sensors.gnss.setup()
        sensors.set_gnss_cache_max_age_s(gnss_interval_s)

        navigator = NavigationController()
        navigator.pd_config.KP = kp
        navigator.pd_config.KD = kd

        print(
            f"GPSスタック回避テスト: speed={speed:g}%, "
            f"kp={kp:g}, kd={kd:g}, GNSS周期={gnss_interval_s:g}秒, "
            f"許容誤差={tolerance_m:g}m, 連続判定=2回"
        )
        print("Ctrl+Cで終了します")
        follow_forward_with_gps_stuck_avoidance(
            navigator,
            driver,
            sensors,
            speed,
            loop_interval_s,
            gnss_interval_s,
            tolerance_m,
        )
        return 0

    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("GPSスタック回避テストを中断しました")
        return 130
    finally:
        if sensors is not None:
            sensors.close()
        if driver is not None:
            driver.cleanup()
        if arm_sleep is not None:
            arm_sleep.off()
            arm_sleep.close()


if __name__ == "__main__":
    raise SystemExit(main())
