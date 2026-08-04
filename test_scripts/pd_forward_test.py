#!/usr/bin/env python3
"""指定秒数だけPD制御で直進するテスト。"""

from __future__ import annotations

from inspect import Parameter, signature
from pathlib import Path
import sys
from typing import Any, Callable

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import NavigationPdConfig
from drive_controller import DriveController
from gpiozero import OutputDevice
from navigation_controller import NavigationController
from sensor_manager import SensorManager


PD_FORWARD_DEFAULT = NavigationController.pd_forward
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


def main() -> int:
    duration = input_float("直進する秒数")
    speed = input_float("基準速度[%]", navigation_default(PD_FORWARD_DEFAULT, "base_speed"))
    kp = input_float("Pゲイン", NavigationPdConfig.KP)
    kd = input_float("Dゲイン", NavigationPdConfig.KD)
    loop_interval = input_float(
        "制御周期[秒]",
        navigation_default(PD_FORWARD_DEFAULT, "loop_interval"),
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
        navigator.pd_forward(
            driver,
            sensors,
            duration,
            base_speed=speed,
            loop_interval=loop_interval,
        )
        print("PD直進テスト終了")
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
