#!/usr/bin/env python3
"""AP起動からWi-Fi復帰までの自撮り一連動作テスト。"""

import argparse
from pathlib import Path
import socket
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from logger import Logger  # noqa: E402
from selfie_manager import SELFIE_EV_VALUES, SelfieManager  # noqa: E402


LOG_FILE = "selfie_full_flow.log"


def request_capture(
    selfie: SelfieManager,
    logger: Logger,
    ev: float | None = None,
) -> int:
    """ESP32S3へ撮影を指示し、画像サイズ通知まで受け取る。"""
    selfie.ensure_connection()
    if selfie.connection is None:
        raise RuntimeError("ESP32S3 is not connected")

    try:
        capture_command = "CAPTURE" if ev is None else f"CAPTURE {round(ev * 2):d}"
        logger.event(f"Capture request start: ev={ev}")
        selfie._send_line(selfie.connection, capture_command)
        size_line = selfie._receive_line(selfie.connection)
        if not size_line.startswith("SIZE "):
            raise RuntimeError(f"Unexpected response: {size_line}")
        image_size = int(size_line.removeprefix("SIZE "))
        logger.event(f"Capture complete on ESP32S3: {image_size} bytes")
        return image_size
    except (ConnectionError, OSError, socket.timeout):
        selfie.close_connection()
        raise


def receive_and_save_image(
    selfie: SelfieManager,
    image_size: int,
    logger: Logger,
    ev: float | None = None,
) -> Path:
    """アーム収納後に画像データを受信して保存する。"""
    if selfie.connection is None:
        raise RuntimeError("ESP32S3 is not connected")

    try:
        logger.event("Image receive start")
        selfie._send_line(selfie.connection, "OK")
        image = selfie._receive_exact(selfie.connection, image_size)
        saved_path = selfie._save_image(image, ev=ev)
        selfie._send_line(selfie.connection, "COMPLETE")

        if selfie._receive_line(selfie.connection) != "READY":
            selfie.close_connection()
        logger.event(f"Image saved: {saved_path} ({saved_path.stat().st_size} bytes)")
        return saved_path
    except (ConnectionError, OSError, socket.timeout):
        selfie.close_connection()
        raise


def parse_args() -> argparse.Namespace:
    """コマンドライン引数を解析する。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ev",
        type=float,
        choices=SELFIE_EV_VALUES,
        help="撮影EV。省略時は従来どおり自動露出で撮影する。",
    )
    return parser.parse_args()


def main() -> None:
    """AP起動、同期、アーム展開、撮影、収納、画像保存、Wi-Fi復帰を実行する。"""
    args = parse_args()
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

            image_size = request_capture(selfie, logger, args.ev)

            logger.event("Arm retract start")
            selfie.retract()
            arm_expanded = False
            logger.event("Arm retract complete")

            saved_path = receive_and_save_image(selfie, image_size, logger, args.ev)
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
