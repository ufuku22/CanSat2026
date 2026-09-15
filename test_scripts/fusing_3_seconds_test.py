#!/usr/bin/env python3
"""本番と同じ設定で溶断回路を1回だけ作動させるテスト。"""

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import FusingConfig
from fusing import GPIO_PIN, fuse


def main() -> int:
    """確認入力後、本番設定の時間だけ溶断回路をONにする。"""
    duration_s = FusingConfig.FUSE_DURATION_S

    print("=== 3秒溶断テスト ===")
    print(f"GPIO {GPIO_PIN} の溶断回路を {duration_s:.1f} 秒間ONにします。")
    print("周囲の安全と機体の配線を確認してください。")

    if not sys.stdin.isatty():
        print("対話端末ではないため、溶断を中止しました。")
        return 1

    try:
        input("実行するにはEnterキーを押してください（中止はCtrl+C）: ")
        print("溶断回路: ON")
        fuse()
    except (EOFError, KeyboardInterrupt):
        print("\n溶断を中止しました。")
        return 1

    print("溶断回路: OFF")
    print("溶断処理が完了しました。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
