#!/usr/bin/env python3
"""赤コーンへの画像誘導を実行するテスト。"""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import CameraCaptureConfig, RedConeConfig
from drive_controller import DriveController
from image_processor import ImageProcessor
from navigation_controller import NavigationController
from navigation_goal import guide_to_red_cone
from sensor_manager import SensorManager


def input_float(label: str, default: float) -> float:
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
        return value


def input_int(label: str, default: int) -> int:
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


def print_goal_results(result: dict) -> None:
    goal_results = []
    if result.get("history"):
        for step_result in result["history"]:
            goal_result = step_result.get("goal_result")
            if goal_result:
                goal_results.append((step_result.get("step"), goal_result))
    elif result.get("last_goal_result"):
        goal_results.append((None, result["last_goal_result"]))

    if not goal_results:
        return

    print("ゴール判定ログ:")
    for step, goal_result in goal_results:
        step_text = "" if step is None else f"step {step} "
        block_ratios = goal_result.get("color_block_ratios") or []
        block_text = ", ".join(f"{ratio * 100:.2f}%" for ratio in block_ratios)
        print(
            f"  {step_text}"
            f"reached={goal_result.get('goal_reached')} "
            f"total={goal_result.get('total_color_ratio', 0.0) * 100:.2f}% "
            f"center={goal_result.get('center_block_color_ratio', 0.0) * 100:.2f}% "
            f"direction={goal_result.get('color_direction')} "
            f"blocks=[{block_text}] "
            f"reason={goal_result.get('goal_reason')}"
        )


def main() -> int:
    max_steps = input_int(
        "誘導の最大試行回数",
        RedConeConfig.MAX_GUIDANCE_STEPS,
    )
    max_scan_steps = input_int(
        "1回の探索で撮影する最大回数",
        RedConeConfig.MAX_SCAN_STEPS,
    )
    red_threshold = input_float(
        "画面内赤検知しきい値",
        RedConeConfig.RED_THRESHOLD,
    )
    red_block_threshold = input_float(
        "5分割方向判定しきい値",
        RedConeConfig.RED_BLOCK_THRESHOLD,
    )
    goal_center_threshold = input_float(
        "ゴール判定の中央赤割合",
        RedConeConfig.GOAL_CENTER_THRESHOLD,
    )
    forward_duration = input_float(
        "通常前進時間[秒]",
        RedConeConfig.FORWARD_DURATION_S,
    )
    forward_speed = input_float("前進速度[%]", RedConeConfig.FORWARD_SPEED)
    rotate_speed = input_float("旋回速度[%]", RedConeConfig.ROTATE_SPEED)
    scan_angle = input_float(
        "探索時の旋回角度[deg]",
        RedConeConfig.SCAN_ANGLE_DEG,
    )
    camera_width = input_int("撮影幅[px]", CameraCaptureConfig.WIDTH)
    camera_height = input_int("撮影高さ[px]", CameraCaptureConfig.HEIGHT)
    camera_timeout_ms = input_int(
        "カメラ起動待ち[ms]",
        CameraCaptureConfig.TIMEOUT_MS,
    )
    capture_hdr = input_bool("HDRを使う", CameraCaptureConfig.HDR)
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        navigator = NavigationController()
        red_cone_config = RedConeConfig()
        red_cone_config.MAX_GUIDANCE_STEPS = max_steps
        red_cone_config.MAX_SCAN_STEPS = max_scan_steps
        red_cone_config.RED_THRESHOLD = red_threshold
        red_cone_config.RED_BLOCK_THRESHOLD = red_block_threshold
        red_cone_config.GOAL_CENTER_THRESHOLD = goal_center_threshold
        red_cone_config.FORWARD_DURATION_S = forward_duration
        red_cone_config.FORWARD_SPEED = forward_speed
        red_cone_config.ROTATE_SPEED = rotate_speed
        red_cone_config.SCAN_ANGLE_DEG = scan_angle
        camera_config = CameraCaptureConfig()
        camera_config.WIDTH = camera_width
        camera_config.HEIGHT = camera_height
        camera_config.TIMEOUT_MS = camera_timeout_ms
        camera_config.HDR = capture_hdr
        image_processor = ImageProcessor()

        print(
            f"赤コーン誘導テスト: max_steps={max_steps}, "
            f"red_threshold={red_threshold:g}, goal_center={goal_center_threshold:g}"
        )
        result = guide_to_red_cone(
            navigator,
            driver,
            sensors,
            image_processor=image_processor,
            red_cone_config=red_cone_config,
            camera_config=camera_config,
        )
        print(f"誘導結果: goal_reached={result['goal_reached']}, reason={result['reason']}")
        print(f"試行回数: {result['steps']}")
        print_goal_results(result)
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
