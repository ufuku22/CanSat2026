#!/usr/bin/env python3
"""電力試験用にモーター動作シーケンスを繰り返す。"""

from __future__ import annotations

from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController


DRIVE_SPEED = 100
HOLD_SECONDS = 30.0
STOP_AFTER_DRIVE_SECONDS = 5.0
TURN_SPEED = 100
TURN_SECONDS = 5.0
STOP_AFTER_TURN_SECONDS = 3.0
RAMP_DOWN_STEPS = 100
RAMP_DOWN_INTERVAL_S = 0.03


def main() -> int:
    print("=== 電力試験用モーターテスト ===")
    print("Ctrl+Cで終了します。終了時はモーターを停止してGPIOを解放します。")

    driver = DriveController()
    try:
        cycle = 1
        while True:
            print(f"\n--- cycle {cycle} ---")

            print("前進: デューティ比100%まで加速")
            driver.drive(DRIVE_SPEED)

            print(f"前進: デューティ比100%を{HOLD_SECONDS:g}秒維持")
            time.sleep(HOLD_SECONDS)

            print("前進: デューティ比0%まで減速")
            driver.ramp_stop_forward(
                DRIVE_SPEED,
                DRIVE_SPEED,
                steps=RAMP_DOWN_STEPS,
                interval=RAMP_DOWN_INTERVAL_S,
            )

            print(f"停止: {STOP_AFTER_DRIVE_SECONDS:g}秒")
            time.sleep(STOP_AFTER_DRIVE_SECONDS)

            print(f"右回頭: {TURN_SECONDS:g}秒")
            driver.turn_right(TURN_SPEED)
            time.sleep(TURN_SECONDS)

            print(f"左回頭: {TURN_SECONDS:g}秒")
            driver.turn_left(TURN_SPEED)
            time.sleep(TURN_SECONDS)

            print(f"停止: {STOP_AFTER_TURN_SECONDS:g}秒")
            driver.stop()
            time.sleep(STOP_AFTER_TURN_SECONDS)

            cycle += 1

    except KeyboardInterrupt:
        print("\n電力試験を中断しました")
        return 130
    finally:
        driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
