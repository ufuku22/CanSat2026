#!/usr/bin/env python3
"""Enterキーで自撮りを繰り返す手動テスト。"""

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from logger import Logger  # noqa: E402
from selfie_manager import SelfieManager  # noqa: E402


LOG_FILE = "selfie_interactive_test.log"


def wait_for_esp32s3(selfie: SelfieManager, logger: Logger) -> None:
    """ESP32S3が接続するまでTCPサーバーを維持して待機する。"""
    while True:
        try:
            selfie.wait_connection()
            logger.event("ESP32S3 connection ready")
            return
        except TimeoutError:
            logger.event("Still waiting for ESP32S3 connection")


def main() -> None:
    """APを起動し、Enterごとに撮影して、終了時にWi-Fiを復帰する。"""
    logger = Logger(filename=LOG_FILE)
    selfie = SelfieManager(logger=logger)

    logger.event("Interactive selfie test started")
    try:
        selfie.start_ap()
        selfie.start_server()
        print("ESP32S3の接続を待っています。終了するには Ctrl+C を押してください。")
        wait_for_esp32s3(selfie, logger)
        print("ESP32S3が接続しました。")

        while True:
            input("Enterキーで撮影します（終了: Ctrl+C）: ")
            try:
                saved_path = selfie.capture_connected()
            except (ConnectionError, OSError, TimeoutError) as exc:
                logger.event(
                    "Capture failed; waiting for the next request "
                    f"({type(exc).__name__}: {exc})"
                )
                print(f"撮影できませんでした: {exc}")
                print("次のEnterでESP32S3の再接続を待ってから再試行します。")
                continue

            logger.event(f"Interactive capture saved: {saved_path}")
            print(f"保存しました: {saved_path}")
    except KeyboardInterrupt:
        logger.event("Interactive selfie test interrupted")
        print("\nテストを終了します。")
    finally:
        logger.event("Selfie TCP server stopping")
        selfie.close_server()
        logger.event("Wi-Fi restore started")
        selfie.restore_wifi()
        logger.event("Interactive selfie test finished")
        print("Wi-Fi APを閉じ、元のWi-Fiへの復帰処理を完了しました。")


if __name__ == "__main__":
    main()
