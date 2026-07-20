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


LOWER_DISTANCE_THRESHOLD_M = 1.0
UPPER_DISTANCE_THRESHOLD_M = 1.5
FORWARD_DISTANCE_M = 0.3


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
            f"{FORWARD_DISTANCE_M:.1f} m前進します"
        )
        result = navigator.judge_ball(
            driver,
            sensors,
            LOWER_DISTANCE_THRESHOLD_M,
            UPPER_DISTANCE_THRESHOLD_M,
            forward_distance_m=FORWARD_DISTANCE_M,
        )

        if result is None:
            print("閾値内の測定結果が見つかりませんでした")
            return 1

        if not result["reached"]:
            print("選択した方位へ旋回できませんでした")
            return 1

        forward_result = result["forward_result"]
        if forward_result is None or not forward_result["completed"]:
            reason = "前進処理が実行されませんでした"
            if forward_result is not None:
                reason = forward_result["reason"]
            print(f"指定距離を前進できませんでした: {reason}")
            return 1

        print(
            f"前進完了: 開始距離={forward_result['start_distance_m']:.3f} m, "
            f"終了距離={forward_result['end_distance_m']:.3f} m"
        )
        print(f"赤色占有率: {result['red_ratio'] * 100:.2f}%")
        return 0 if result["ball_detected"] else 1

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
