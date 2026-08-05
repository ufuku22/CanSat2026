#!/usr/bin/env python3
"""avoid_parachute()を5回連続で実行する実機テスト。"""

from __future__ import annotations

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from navigation_controller import NavigationController
from sensor_manager import SensorManager


TEST_COUNT = 5


def main() -> int:
    input(
        "機体の周囲が安全であることを確認してください。"
        f"avoid_parachute()を{TEST_COUNT}回連続で実行するにはEnterを押してください。"
    )

    driver: DriveController | None = None
    sensors: SensorManager | None = None

    try:
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        navigator = NavigationController()

        for attempt in range(1, TEST_COUNT + 1):
            print(f"\n[{attempt}/{TEST_COUNT}] パラシュート回避を開始します", flush=True)
            result = navigator.avoid_parachute(driver, sensors)
            print(
                f"[{attempt}/{TEST_COUNT}] 完了: "
                f"action={result['action']}, "
                f"purple_detected={result['purple_detected']}, "
                f"purple_ratio={result['purple_ratio']:.4f}",
                flush=True,
            )

        print(f"\navoid_parachute()を{TEST_COUNT}回実行しました。")
        return 0

    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("テストを中断しました。")
        return 130
    except Exception as exc:
        if driver is not None:
            driver.stop()
        print(f"テストに失敗しました: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        if sensors is not None:
            sensors.close()
        if driver is not None:
            driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
