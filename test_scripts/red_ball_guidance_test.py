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
    if result.get("target_heading_deg") is not None:
        print(f"A方位: {float(result['target_heading_deg']):.1f}deg")


def _print_square_gate_summary(square_result: dict) -> None:
    print(
        "スクエアゾーン誘導結果: "
        f"square_zone_reached={square_result['square_zone_reached']}, "
        f"reason={square_result['reason']}"
    )
    print(f"接近したボール数: {square_result['approached_balls']}")
    print(f"最終距離: {_format_m(square_result['last_distance_m'])}")

    for record in square_result.get("history", []):
        if record.get("classification") != "adjacent":
            continue

        geometry = record.get("geometry") or {}
        print(
            "入口ゲート: "
            f"B候補={record.get('candidate_index')}, "
            f"AB距離={_format_m(record.get('ab_distance_m'))}, "
            f"B距離={_format_m(record.get('b_surface_distance_m'))}, "
            f"B方位={float(record['b_heading_deg']):.1f}deg"
        )
        print(
            "Q目標: "
            f"QB_LiDAR={_format_m(geometry.get('qb_lidar_distance_m'))}, "
            f"LiDAR前方オフセット={_format_m(geometry.get('lidar_forward_offset_m'))}, "
            f"中央方位={float(geometry['center_heading_deg']):.1f}deg"
        )

        q_history = record.get("q_advance_history", [])
        if q_history:
            last_q = q_history[-1]
            reverse_count = sum(
                1
                for item in q_history
                if item.get("reverse_duration_s") is not None
            )
            print(
                "Q微前進: "
                f"steps={len(q_history)}, "
                f"後退回数={reverse_count}, "
                f"最後のLiDAR={_format_m(last_q.get('distance_m'))}"
            )
        center_rotate = record.get("center_rotate_result")
        if center_rotate is not None:
            print(
                "中央方向旋回: "
                f"target={center_rotate['target_angle_deg']:.2f}deg, "
                f"rotated={center_rotate['rotated_angle_deg']:.2f}deg, "
                f"reached={center_rotate['reached']}"
            )
        break


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="赤ボール接近とスクエアゾーン進入の実機テスト"
    )
    parser.add_argument(
        "--square-only",
        action="store_true",
        help="A接近済みの状態からスクエアゾーン誘導だけを実行する",
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
        from navigation_goal import guide_to_red_ball, guide_to_square_zone
        from sensor_manager import SensorManager

        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        sensors.distance.setup()

        navigation_controller = NavigationController()
        if not args.square_only:
            result = guide_to_red_ball(navigation_controller, driver, sensors)
            _print_red_ball_result(result)
            if not result["target_reached"]:
                return 1
            if args.red_ball_only:
                return 0

        square_result = guide_to_square_zone(
            navigation_controller,
            driver,
            sensors,
        )
        _print_square_gate_summary(square_result)
        return 0 if square_result["square_zone_reached"] else 1

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
