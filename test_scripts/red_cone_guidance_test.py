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
from navigation_goal import guide_to_red_cone
from sensor_manager import SensorManager


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
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        navigator = NavigationController()
        print("赤コーン誘導テスト: config.pyのデフォルト設定を使用します")
        result = guide_to_red_cone(navigator, driver, sensors)
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
