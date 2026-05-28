#!/usr/bin/env python3
"""ESP32S3カメラ受信の動作確認用スクリプト。"""

from pathlib import Path
import sys


# リポジトリ直下の esp32s3_camera_receiver.py を読み込む。
sys.path.append(str(Path(__file__).resolve().parents[1]))

from esp32s3_camera_receiver import log, run_capture_sequence  # noqa: E402


def main() -> None:
    """AP起動、撮影、画像保存、Wi-Fi復帰までを1回だけ実行する。"""
    log("ESP32S3 camera receive test started")
    saved_path = run_capture_sequence()
    log(f"test saved: {saved_path}")


if __name__ == "__main__":
    main()
