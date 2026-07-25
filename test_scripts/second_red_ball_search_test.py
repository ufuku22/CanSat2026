#!/usr/bin/env python3
"""1回目だけしきい値を変えて、赤ボール探索を5回行う実機テスト。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_controller import NavigationController
from navigation_goal import search_second_red_ball_and_advance
from sensor_manager import SensorManager


@dataclass(frozen=True)
class TrialThresholds:
    """1回分の探索で使用するしきい値。"""

    distance_min_m: float
    distance_max_m: float
    center_red_ratio_threshold: float


# 1回目と、2～5回目のしきい値をここで変更する。
# center_red_ratio_thresholdは、1%なら0.01のように0.0～1.0で指定する。
FIRST_TRIAL_THRESHOLDS = TrialThresholds(1.5, 2.5, 0.01)
OTHER_TRIAL_THRESHOLDS = TrialThresholds(0.8, 1.3, 0.01)
TRIAL_THRESHOLDS = (
    FIRST_TRIAL_THRESHOLDS,
    *(OTHER_TRIAL_THRESHOLDS for _ in range(4)),
)

LIDAR_STOP_DISTANCE_M = 0.6
INTER_TRIAL_INTERVAL_S = 1.0


def main() -> int:
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    if len(TRIAL_THRESHOLDS) != 5:
        print(
            "TRIAL_THRESHOLDSには5回分のしきい値を指定してください: "
            f"現在{len(TRIAL_THRESHOLDS)}件"
        )
        return 2

    try:
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        sensors.distance.setup()

        results = []
        total_trials = len(TRIAL_THRESHOLDS)

        for trial_number, thresholds in enumerate(
            TRIAL_THRESHOLDS,
            start=1,
        ):
            print("\n" + "=" * 60)
            print(
                f"試行 {trial_number}/{total_trials}: "
                f"距離={thresholds.distance_min_m:.3f}"
                f"..{thresholds.distance_max_m:.3f} m, "
                "中央赤色割合しきい値="
                f"{thresholds.center_red_ratio_threshold * 100:.2f}%"
            )
            print(
                "前進方式=lidar_forward, "
                f"停止距離={LIDAR_STOP_DISTANCE_M:.3f} m"
            )

            result = search_second_red_ball_and_advance(
                NavigationController(),
                driver,
                sensors,
                distance_min_m=thresholds.distance_min_m,
                distance_max_m=thresholds.distance_max_m,
                center_red_ratio_threshold=(
                    thresholds.center_red_ratio_threshold
                ),
                lidar_distance_threshold_m=LIDAR_STOP_DISTANCE_M,
            )
            results.append(result)

            print(
                f"試行 {trial_number} 結果: "
                f"target_found={result['target_found']}, "
                f"moved_forward={result['moved_forward']}, "
                f"reason={result['reason']}"
            )
            print(f"探索回数: {result['steps']}")
            print(f"最終距離: {result['last_distance_m']}")
            print(f"前進方式: {result['forward_mode']}")
            if result["forward_mode"] == "lidar":
                print(
                    "LiDAR停止距離: "
                    f"{result['lidar_final_distance_m']}"
                )
            else:
                print(f"前進時間: {result['forward_duration_s']}")

            red_result = result["last_red_result"]
            if red_result is not None:
                center_ratio = red_result["goal_angle_color_ratio"]
                print(f"最終中央赤色割合: {center_ratio * 100:.2f}%")

            if (
                trial_number < total_trials
                and INTER_TRIAL_INTERVAL_S > 0.0
            ):
                print(
                    f"次の試行まで{INTER_TRIAL_INTERVAL_S:g}秒待機します"
                )
                time.sleep(INTER_TRIAL_INTERVAL_S)

        success_count = sum(
            result["moved_forward"]
            for result in results
        )
        print("\n" + "=" * 60)
        print(
            f"5回の試行完了: 成功={success_count}, "
            f"失敗={total_trials - success_count}"
        )
        return 0 if success_count == total_trials else 1

    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("2つ目の赤ボール探索テストを中断しました")
        return 130
    finally:
        if sensors is not None:
            sensors.close()
        if driver is not None:
            driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
