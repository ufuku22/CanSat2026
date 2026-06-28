"""
test_arm.py

SelfieManager の既存関数を使って、アームの展開と収納を確認するテストプログラムです。
"""

import time
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from selfie_manager import SelfieManager


WAIT_AFTER_EXPAND_S = 1.0


def main():
    """アームの展開と収納を実行する。"""
    arm = SelfieManager()

    try:
        print("=== アーム展開・収納テスト開始 ===")

        print("アームを展開します")
        arm.expand()
        print("アーム展開完了")

        time.sleep(WAIT_AFTER_EXPAND_S)

        print("アームを収納します")
        arm.retract()
        print("アーム収納完了")

        print("=== アーム展開・収納テスト終了 ===")

    except KeyboardInterrupt:
        print("\nアーム展開・収納テストを中断しました")


if __name__ == "__main__":
    main()
