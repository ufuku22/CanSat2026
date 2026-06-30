#!/usr/bin/env python3
"""赤コーンへの画像誘導を実行するテスト。"""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_controller import NavigationController
from sensor_manager import SensorManager


def input_float(label: str, default: float, *, positive: bool = False) -> float:
    while True:
        raw = input(f"{label} [{default:g}]: ").strip()
        if raw == "":
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


def input_int(label: str, default: int, *, positive: bool = False) -> int:
    while True:
        raw = input(f"{label} [{default}]: ").strip()
        if raw == "":
            value = default
        else:
            try:
                value = int(raw)
            except ValueError:
                print("整数で入力してください。")
                continue
        if positive and value <= 0:
            print("1以上の値を入力してください。")
            continue
        return value


def input_bool(label: str, default: bool) -> bool:
    default_text = "Y" if default else "n"
    while True:
        raw = input(f"{label} [Y/n] ({default_text}): ").strip().lower()
        if raw == "":
            return default
        if raw in ("y", "yes"):
            return True
        if raw in ("n", "no"):
            return False
        print("y または n で入力してください。")


def main() -> int:
    max_steps = input_int("誘導の最大試行回数", 20, positive=True)
    max_scan_steps = input_int("1回の探索で撮影する最大回数", 6, positive=True)
    red_threshold = input_float("画面内赤検知しきい値", 0.05)
    red_block_threshold = input_float("5分割方向判定しきい値", 0.08)
    goal_total_threshold = input_float("ゴール判定の全体赤割合", 0.90)
    goal_center_threshold = input_float("ゴール判定の中央赤割合", 0.10)
    forward_duration = input_float("通常前進時間[秒]", 1.0, positive=True)
    forward_speed = input_float("前進速度[%]", 60.0)
    rotate_speed = input_float("旋回速度[%]", 30.0)
    scan_angle = input_float("探索時の旋回角度[deg]", 60.0)
    camera_width = input_int("撮影幅[px]", 1280, positive=True)
    camera_height = input_int("撮影高さ[px]", 720, positive=True)
    camera_timeout_ms = input_int("カメラ起動待ち[ms]", 2000, positive=True)
    capture_hdr = input_bool("HDRを使う", True)
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        navigator = NavigationController()

        print(
            f"赤コーン誘導テスト: max_steps={max_steps}, "
            f"red_threshold={red_threshold:g}, goal_total={goal_total_threshold:g}"
        )
        result = navigator.guide_to_red_cone(
            driver,
            sensors,
            red_threshold=red_threshold,
            red_block_threshold=red_block_threshold,
            goal_center_threshold=goal_center_threshold,
            goal_total_threshold=goal_total_threshold,
            scan_angle_deg=scan_angle,
            max_scan_steps=max_scan_steps,
            max_steps=max_steps,
            forward_duration_s=forward_duration,
            forward_speed=forward_speed,
            capture_width=camera_width,
            capture_height=camera_height,
            capture_hdr=capture_hdr,
            capture_timeout_ms=camera_timeout_ms,
            rotate_speed=rotate_speed,
        )
        print(f"誘導結果: goal_reached={result['goal_reached']}, reason={result['reason']}")
        print(f"試行回数: {result['steps']}")
        return 0 if result["goal_reached"] else 1

    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("赤コーン誘導テストを中断しました")
        return 130
    finally:
        if sensors is not None:
            sensors.close()
        if driver is not None:
            driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
