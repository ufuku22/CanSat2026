#!/usr/bin/env python3
"""AP起動からWi-Fi復帰までの自撮り一連動作テスト。"""

from pathlib import Path
import socket
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from selfie_manager import SelfieManager  # noqa: E402


def request_capture(selfie: SelfieManager) -> int:
    """ESP32S3へ撮影を指示し、画像サイズ通知まで受け取る。"""
    selfie.ensure_connection()

    if selfie.connection is None:
        raise RuntimeError("ESP32S3 is not connected")

    try:
        selfie._send_line(selfie.connection, "CAPTURE")

        size_line = selfie._receive_line(selfie.connection)
        if not size_line.startswith("SIZE "):
            raise RuntimeError(f"Unexpected response: {size_line}")

        image_size = int(size_line.removeprefix("SIZE "))
        return image_size

    except (ConnectionError, OSError, socket.timeout):
        selfie.close_connection()
        raise


def receive_and_save_image(selfie: SelfieManager, image_size: int) -> Path:
    """アーム収納後に画像データを受信して保存する。"""
    if selfie.connection is None:
        raise RuntimeError("ESP32S3 is not connected")

    try:
        selfie._send_line(selfie.connection, "OK")

        image = selfie._receive_exact(selfie.connection, image_size)
        saved_path = selfie._save_image(image)

        selfie._send_line(selfie.connection, "COMPLETE")

        if selfie._receive_line(selfie.connection) != "READY":
            selfie.close_connection()

        return saved_path

    except (ConnectionError, OSError, socket.timeout):
        selfie.close_connection()
        raise


def main() -> None:
    """AP起動、同期、アーム展開、撮影、収納、画像保存、Wi-Fi復帰を実行する。"""
    arm_expanded = False

    with SelfieManager() as selfie:
        try:
            selfie.start_ap()

            selfie.wait_connection()

            selfie.expand()
            arm_expanded = True

            image_size = request_capture(selfie)

            selfie.retract()
            arm_expanded = False

            saved_path = receive_and_save_image(selfie, image_size)

            print(f"Selfie full flow test complete: {saved_path}")

        except BaseException:
            if arm_expanded:
                try:
                    selfie.retract()
                except Exception as exc:
                    print(
                        f"Arm retract failed after error: "
                        f"{type(exc).__name__}: {exc}"
                    )
            raise


if __name__ == "__main__":
    main()