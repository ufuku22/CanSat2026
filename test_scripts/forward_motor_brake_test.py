#!/usr/bin/env python3
"""両モーターを前進させ、最後に短絡ブレーキで止める。"""

from __future__ import annotations

from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController


DRIVE_SPEED = 100
DRIVE_SECONDS = 1.0
BRAKE_HOLD_SECONDS = 0.5


def main() -> int:
    print("=== 前進モーターブレーキテスト ===")
    print(f"前進速度: {DRIVE_SPEED}%")
    print(f"前進時間: {DRIVE_SECONDS:g}秒")
    input("準備できたらEnterを押してください")

    driver = DriveController()
    try:
        print("前進開始")
        driver.drive(DRIVE_SPEED)
        time.sleep(DRIVE_SECONDS)

        print("急ブレーキ")
        driver.brake()
        time.sleep(BRAKE_HOLD_SECONDS)
        print("終了")
        return 0
    except KeyboardInterrupt:
        print("\n中断しました。急ブレーキします。")
        driver.brake()
        time.sleep(BRAKE_HOLD_SECONDS)
        return 130
    finally:
        driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
