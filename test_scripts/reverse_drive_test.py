#!/usr/bin/env python3
"""指定した出力で、指定秒数だけ後退するテスト。"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController


def input_float(label: str, maximum: float | None = None) -> float:
    while True:
        try:
            value = float(input(f"{label}: ").strip())
        except ValueError:
            print("数値で入力してください。")
            continue

        if not math.isfinite(value) or value <= 0:
            print("0より大きい値を入力してください。")
            continue
        if maximum is not None and value > maximum:
            print(f"{maximum:g}以下の値を入力してください。")
            continue
        return value


def main() -> int:
    driver: DriveController | None = None

    try:
        seconds = input_float("後退する秒数")
        output = input_float("モーター出力[%]", maximum=100)
        print(f"後退設定: {seconds:g}秒, 出力={output:g}%")
        input("準備できたらEnterを押してください")

        driver = DriveController()
        print("後退開始")
        driver.drive(-output)
        time.sleep(seconds)
        print("後退終了")
        return 0
    except KeyboardInterrupt:
        print("\n中断しました")
        return 130
    finally:
        if driver is not None:
            driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
