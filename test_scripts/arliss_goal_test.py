#!/usr/bin/env python3
"""ARLISS向けのボール接近・赤色検知を確認する実機テスト。"""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_goal import GoalNavigator
from sensor_manager import SensorManager


LOWER_DISTANCE_THRESHOLD_M = 2.5
UPPER_DISTANCE_THRESHOLD_M = 3
FORWARD_STOP_DISTANCE_M = 0.5


def main() -> int:
    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()

        # このテストで使用する9軸センサと距離センサだけを初期化する。
        sensors.imu.setup()
        sensors.distance.setup()

        navigator = GoalNavigator()

        print("周囲の距離と方位を測定します")
        scan_results = navigator.detect_ball(driver, sensors)

        for index, sample in enumerate(scan_results, start=1):
            distance = sample["distance_m"]
            distance_text = "測定不能" if distance is None else f"{distance:.3f} m"
            print(
                f"測定{index:02d}: "
                f"相対角度={sample['relative_angle_deg']:.1f} deg, "
                f"方位={sample['heading_deg']:.1f} deg, "
                f"距離={distance_text}"
            )

        print(
            f"{LOWER_DISTANCE_THRESHOLD_M:.1f}～"
            f"{UPPER_DISTANCE_THRESHOLD_M:.1f} mの範囲で最も遠い方向を選び、"
            f"赤色検知後に距離{FORWARD_STOP_DISTANCE_M:.1f} mまで直進します"
        )
        result = navigator.judge_ball(
            driver,
            sensors,
            LOWER_DISTANCE_THRESHOLD_M,
            UPPER_DISTANCE_THRESHOLD_M,
            forward_stop_distance_m=FORWARD_STOP_DISTANCE_M,
        )

        if result is None:
            print("閾値内の測定結果が見つかりませんでした")
            return 1

        if not result["reached"]:
            print("選択した方位へ旋回できませんでした")
            return 1

        print(f"赤色占有率: {result['red_ratio'] * 100:.2f}%")

        if not result["ball_detected"]:
            return 1

        if not result["forward_completed"]:
            print("rider_forwardによる直進を実行できませんでした")
            return 1

        return 0

    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("テストを中断しました")
        return 130
    finally:
        if sensors is not None:
            sensors.close()
        if driver is not None:
            driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
