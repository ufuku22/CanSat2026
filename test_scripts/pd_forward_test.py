#!/usr/bin/env python3
"""指定秒数だけPD制御で直進するテスト。"""

from __future__ import annotations

from inspect import Parameter, signature
from pathlib import Path
import sys
import time
from typing import Any, Callable

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import NavigationPdConfig
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


def follow_forward_with_repeated_avoidance(
    navigator,
    driver,
    sensors,
    duration,
    speed,
    loop_interval,
):
    target_heading = float(sensors.get_heading_deg())
    prev_error = 0.0
    left_speed = speed
    right_speed = speed
    started_at = time.monotonic()
    moving_forward = False
    avoidance_count = 0
    navigator._reset_stuck_detection()

    print(f"初期目標方位: {target_heading:.1f}度")
    try:
        while time.monotonic() - started_at <= duration:
            left_speed, right_speed, prev_error = navigator.drive_toward_heading(
                driver,
                sensors,
                target_heading=target_heading,
                base_speed=speed,
                prev_error=prev_error,
                loop_interval=loop_interval,
            )
            moving_forward = True

            if (
                navigator.stuck_avoidance_config.ENABLED
                and navigator.avoid_stuck(driver, sensors)
            ):
                moving_forward = False
                avoidance_count += 1
                remaining = duration - (time.monotonic() - started_at)
                print(
                    f"スタック回避{avoidance_count}回目完了: "
                    f"残り時間={max(0.0, remaining):.1f}秒"
                )
                if remaining <= 0.0:
                    break

                current_heading = float(sensors.get_heading_deg())
                prev_error = navigator.heading_error(
                    current_heading,
                    target_heading,
                )

            time.sleep(loop_interval)

        if moving_forward:
            navigator._pd_ramp_stop_forward(
                driver,
                sensors,
                left_speed,
                right_speed,
                target_heading=target_heading,
                prev_error=prev_error,
                steps=navigation_default(
                    FOLLOW_FORWARD_DEFAULT,
                    "stop_ramp_steps",
                ),
                interval=navigation_default(
                    FOLLOW_FORWARD_DEFAULT,
                    "stop_ramp_interval",
                ),
            )
        else:
            driver.stop()
    except BaseException:
        driver.stop()
        raise
    finally:
        navigator._reset_stuck_detection()

    return avoidance_count


def main() -> int:
    duration = input_float("直進する秒数")
    speed = input_float("基準速度[%]", navigation_default(FOLLOW_FORWARD_DEFAULT, "base_speed"))
    kp = input_float("Pゲイン", NavigationPdConfig.KP)
    kd = input_float("Dゲイン", NavigationPdConfig.KD)
    loop_interval = input_float(
        "制御周期[秒]",
        navigation_default(FOLLOW_FORWARD_DEFAULT, "loop_interval"),
    )

    driver: DriveController | None = None
    sensors: SensorManager | None = None
    arm_sleep: OutputDevice | None = None

    try:
        arm_sleep = OutputDevice(ARM_SLEEP_PIN, active_high=True, initial_value=False)
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        navigator = NavigationController()
        navigator.pd_config.KP = kp
        navigator.pd_config.KD = kd

        print(
            f"PD直進テスト: duration={duration:g}秒, "
            f"speed={speed:g}%, kp={kp:g}, kd={kd:g}"
        )
        avoidance_count = follow_forward_with_repeated_avoidance(
            navigator,
            driver,
            sensors,
            duration,
            speed,
            loop_interval,
        )
        print(f"PD直進テスト終了: スタック回避={avoidance_count}回")
        return 0

    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("PD直進テストを中断しました")
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
