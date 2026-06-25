#!/usr/bin/env python3
"""SelfieManagerの動作確認用スクリプト。"""

from pathlib import Path
import sys


# リポジトリ直下の selfie_manager.py を読み込む。
sys.path.append(str(Path(__file__).resolve().parents[1]))

from selfie_manager import SelfieManager, log  # noqa: E402


def main() -> None:
    """AP起動、撮影、画像保存、Wi-Fi復帰までを1回だけ実行する。"""
    log("SelfieManager test started")
    with SelfieManager() as selfie:
        saved_path = selfie.capture()
    log(f"test saved: {saved_path}")


if __name__ == "__main__":
    main()
