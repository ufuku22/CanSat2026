#!/usr/bin/env python3
"""スタビライザー反転だけを実行する。"""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController

def main() -> int:
    print("=== スタビライザー反転テスト ===")
    input("準備できたらEnterを押してください")

    driver = DriveController()
    try:
        driver.reverse_stabilizer()
        print("終了")
        return 0
    except KeyboardInterrupt:
        print("\n中断しました")
        return 130
    finally:
        driver.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
