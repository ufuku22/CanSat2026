#!/usr/bin/env python3
"""SelfieManager を使って、アームの収納だけを確認するテストプログラム。"""

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from selfie_manager import SelfieManager


def main() -> None:
    """アームを収納する。"""
    arm = SelfieManager()

    try:
        print("=== アーム収納テスト開始 ===")
        print("アームを収納します")
        arm.retract()
        print("アーム収納完了")
        print("=== アーム収納テスト終了 ===")
    except KeyboardInterrupt:
        print("\nアーム収納テストを中断しました")


if __name__ == "__main__":
    main()
