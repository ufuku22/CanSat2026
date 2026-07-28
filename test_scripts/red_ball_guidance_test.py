#!/usr/bin/env python3
"""赤ボール誘導を実行する実機テスト。"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def _format_m(value) -> str:
    return "None" if value is None else f"{float(value):.3f}m"


def _print_red_ball_result(result: dict) -> None:
    print(
        f"赤ボール誘導結果: target_reached={result['target_reached']}, "
        f"reason={result['reason']}"
    )
    print(f"試行回数: {result['steps']}")
    print(f"最終距離: {_format_m(result['last_distance_m'])}")
    if result.get("target_distance_m") is not None:
        print(f"A停止目標距離: {_format_m(result['target_distance_m'])}")
    if result.get("target_tolerance_m") is not None:
        print(f"A停止許容幅: ±{_format_m(result['target_tolerance_m'])}")


def _print_square_summary(square_result: dict) -> None:
    print(
        "スクエアゾーン誘導結果: "
        f"square_zone_reached={square_result['square_zone_reached']}, "
        f"reason={square_result['reason']}"
    )
    print(f"接近したボール数: {square_result['approached_balls']}")
    print(f"最終距離: {_format_m(square_result['last_distance_m'])}")

    last_target = None
    for record in square_result.get("history", []):
        if record.get("adjacent_ball") is not None:
            last_target = record
    if last_target is None:
        return

    print(
        "誘導サマリ: "
        f"target={last_target.get('target_index')}, "
        f"turn={last_target.get('turn_angle_deg'):.2f}deg, "
        f"approach_steps={len(last_target.get('approach_history', []))}"
    )


def _print_center_summary(center_result: dict) -> None:
    print(
        "スクエアゾーン中心誘導結果: "
        f"center_reached={center_result['center_reached']}, "
        f"reason={center_result['reason']}"
    )
    print(f"中心誘導サイクル数: {len(center_result.get('history', []))}")
    print(f"最終距離: {_format_m(center_result['last_distance_m'])}")
    for record in center_result.get("history", []):
        turn_180_result = record.get("turn_180_result") or {}
        opposite_45_result = record.get("opposite_45_result")
        opposite_45_text = "なし"
        if opposite_45_result is not None:
            opposite_45_text = (
                f"{opposite_45_result.get('target_angle_deg')}deg, "
                f"reached={opposite_45_result.get('reached')}"
            )
        print(
            "中心誘導判定: "
            f"cycle={record.get('cycle')}, "
            f"180度転回={turn_180_result.get('reached')}, "
            f"選択方向={record.get('turn_direction')}, "
            f"選択角度={record.get('detected_turn_angle_deg')}deg, "
            f"測定距離={_format_m(record.get('measured_distance_m'))}, "
            f"対角判定={record.get('is_diagonal_ball')}, "
            f"逆45度旋回={opposite_45_text}, "
            "接近目標="
            f"{_format_m(record.get('approach_target_distance_m'))}"
        )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="赤ボール接近・スクエアゾーン進入・中心誘導の実機テスト"
    )
    parser.add_argument(
        "--square-only",
        action="store_true",
        help="A接近済みの状態からスクエアゾーン進入と中心誘導を実行する",
    )
    parser.add_argument(
        "--red-ball-only",
        action="store_true",
        help="Aへの赤ボール誘導だけを実行して終了する",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    driver = None
    sensors = None

    try:
        from drive_controller import DriveController
        from navigation_controller import NavigationController
        from navigation_goal import (
            guide_to_center_of_zone,
            guide_to_red_ball,
            guide_to_square_zone,
        )
        from sensor_manager import SensorManager

        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        sensors.distance.setup()

        navigation_controller = NavigationController()
        if not args.square_only:
            first_ball_result = guide_to_red_ball(
                navigation_controller,
                driver,
                sensors,
            )
            _print_red_ball_result(first_ball_result)
            if not first_ball_result["target_reached"]:
                return 1
            if args.red_ball_only:
                return 0

        square_result = guide_to_square_zone(
            navigation_controller,
            driver,
            sensors,
        )
        _print_square_summary(square_result)
        if not square_result["square_zone_reached"]:
            return 1

        center_result = guide_to_center_of_zone(
            navigation_controller,
            driver,
            sensors,
        )
        _print_center_summary(center_result)
        return 0 if center_result["center_reached"] else 1

    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("赤ボール誘導テストを中断しました")
        return 130
    finally:
        if sensors is not None:
            sensors.close()
        if driver is not None:
            driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
