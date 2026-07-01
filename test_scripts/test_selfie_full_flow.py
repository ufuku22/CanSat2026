#!/usr/bin/env python3
"""AP起動からWi-Fi復帰までの自撮り一連動作テスト。"""

from pathlib import Path
import socket
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from logger import Logger  # noqa: E402
from selfie_manager import SelfieManager  # noqa: E402


LOG_FILE = "selfie_full_flow.log"


def request_capture(selfie: SelfieManager, logger: Logger) -> int:
    """ESP32S3へ撮影を指示し、画像サイズ通知まで受け取る。"""
    selfie.ensure_connection()
    if selfie.connection is None:
        raise RuntimeError("ESP32S3 is not connected")

    try:
        logger.event("Capture request start")
        selfie._send_line(selfie.connection, "CAPTURE")
        size_line = selfie._receive_line(selfie.connection)
        if not size_line.startswith("SIZE "):
            raise RuntimeError(f"Unexpected response: {size_line}")
        image_size = int(size_line.removeprefix("SIZE "))
        logger.event(f"Capture complete on ESP32S3: {image_size} bytes")
        return image_size
    except (ConnectionError, OSError, socket.timeout):
        selfie.close_connection()
        raise


def receive_and_save_image(selfie: SelfieManager, image_size: int, logger: Logger) -> Path:
    """アーム収納後に画像データを受信して保存する。"""
    if selfie.connection is None:
        raise RuntimeError("ESP32S3 is not connected")

    try:
        logger.event("Image receive start")
        selfie._send_line(selfie.connection, "OK")
        image = selfie._receive_exact(selfie.connection, image_size)
        saved_path = selfie._save_image(image)
        selfie._send_line(selfie.connection, "COMPLETE")

        if selfie._receive_line(selfie.connection) != "READY":
            selfie.close_connection()
        logger.event(f"Image saved: {saved_path} ({saved_path.stat().st_size} bytes)")
        return saved_path
    except (ConnectionError, OSError, socket.timeout):
        selfie.close_connection()
        raise


def main() -> None:
    """AP起動、同期、アーム展開、撮影、収納、画像保存、Wi-Fi復帰を実行する。"""
    logger = Logger(filename=LOG_FILE)
    arm_expanded = False

    logger.event("Selfie full flow test started")
    with SelfieManager(logger=logger) as selfie:
        try:
            logger.event("AP start")
            selfie.start_ap()

            logger.event("ESP32S3 sync start")
            selfie.wait_connection()
            logger.event("ESP32S3 sync complete")

            logger.event("Arm expand start")
            selfie.expand()
            arm_expanded = True
            logger.event("Arm expand complete")

            image_size = request_capture(selfie, logger)

            logger.event("Arm retract start")
            selfie.retract()
            arm_expanded = False
            logger.event("Arm retract complete")

            saved_path = receive_and_save_image(selfie, image_size, logger)
            logger.event(f"Selfie full flow test complete: {saved_path}")
        except BaseException:
            if arm_expanded:
                try:
                    logger.event("Arm retract start after error")
                    selfie.retract()
                    logger.event("Arm retract complete after error")
                except Exception as exc:
                    logger.event(f"Arm retract failed after error: {type(exc).__name__}: {exc}")
            raise


if __name__ == "__main__":
    main()
