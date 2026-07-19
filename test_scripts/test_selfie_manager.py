#!/usr/bin/env python3
"""SelfieManagerの動作確認用スクリプト。"""

from pathlib import Path
import sys


# リポジトリ直下の selfie_manager.py を読み込む。
sys.path.append(str(Path(__file__).resolve().parents[1]))

from logger import Logger  # noqa: E402
from selfie_manager import SelfieManager  # noqa: E402


def main() -> None:
    """AP起動、撮影、画像保存、Wi-Fi復帰までを1回だけ実行する。"""
    logger = Logger(filename="selfie_test.log")
    logger.event("selfie test started")
    try:
        with SelfieManager(logger=logger) as selfie:
            saved_path = selfie.capture()
    except KeyboardInterrupt:
        logger.event("selfie test interrupted")
        return
    except TimeoutError:
        logger.event("selfie test timed out waiting for ESP32S3")
        return
    logger.event(f"test saved: {saved_path}")


if __name__ == "__main__":
    main()
