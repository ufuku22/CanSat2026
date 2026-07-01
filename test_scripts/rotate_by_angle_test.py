#!/usr/bin/env python3
"""指定角度だけIMUを見ながら旋回するテスト。"""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_controller import NavigationController
from sensor_manager import SensorManager


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
    angle = input_float("旋回角度[deg] 正=右旋回 / 負=左旋回")
    speed = input_float("旋回速度[%]", 30.0)
    tolerance = input_float("許容誤差[deg]", 3.0)
    timeout = input_float("タイムアウト[秒]", 10.0)
    loop_interval = input_float("確認周期[秒]", 0.01)

    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        navigator = NavigationController()

        print(
            f"角度指定旋回テスト: angle={angle:g}deg, "
            f"speed={speed:g}%, tolerance={tolerance:g}deg"
        )
        result = navigator.rotate_by_angle(
            driver,
            sensors,
            angle,
            speed=speed,
            tolerance_deg=tolerance,
            timeout_s=timeout,
            loop_interval=loop_interval,
        )
        print(
            f"旋回結果: reached={result['reached']}, "
            f"target={result['target_angle_deg']:.1f}deg, "
            f"rotated={result['rotated_angle_deg']:.1f}deg"
        )
        return 0 if result["reached"] else 1

    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("角度指定旋回テストを中断しました")
        return 130
    finally:
        if sensors is not None:
            sensors.close()
        if driver is not None:
            driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
