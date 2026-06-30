#!/usr/bin/env python3
"""指定秒数だけPD制御で直進するテスト。"""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def input_float(label: str, default: float | None = None, *, positive: bool = False) -> float:
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
        if positive and value <= 0:
            print("0より大きい値を入力してください。")
            continue
        return value


def main() -> int:
    duration = input_float("直進する秒数", positive=True)
    speed = input_float("基準速度[%]", 100.0)
    kp = input_float("Pゲイン", 0.80)
    kd = input_float("Dゲイン", 0.05)
    loop_interval = input_float("制御周期[秒]", 0.02, positive=True)

    from drive_controller import DriveController
    from navigation_controller import NavigationController
    from sensor_manager import SensorManager

    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        navigator = NavigationController()

        print(
            f"PD直進テスト: duration={duration:g}秒, "
            f"speed={speed:g}%, kp={kp:g}, kd={kd:g}"
        )
        navigator.follow_petit_forward(
            driver,
            sensors,
            duration,
            base_speed=speed,
            kp=kp,
            kd=kd,
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


if __name__ == "__main__":
    raise SystemExit(main())
