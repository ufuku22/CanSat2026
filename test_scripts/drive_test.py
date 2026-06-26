import time
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drive_controller import DriveController
from sensor_manager import CameraV3


# ===== テスト条件 =====
FORWARD_SPEED = 60       # 前進速度 [%] 0〜100
FORWARD_TIME_S = 3.0     # 前進する時間 [秒]
STOP_TIME_S = 1.0        # 停止後に待つ時間 [秒]
CAPTURE_WIDTH = 1920     # 撮影画像の幅 [px]
CAPTURE_HEIGHT = 1080    # 撮影画像の高さ [px]
CAPTURE_HDR = False      # HDR撮影を使うか
CAPTURE_TIMEOUT_MS = 2000
CAMERA_SAVE_DIR = PROJECT_ROOT / "cansat_camera_images"


def main():
    """直進テストを実行する。"""
    driver = DriveController()
    camera = CameraV3(save_dir=CAMERA_SAVE_DIR)

    try:
        print("=== 直進テスト開始 ===")
        print(f"速度: {FORWARD_SPEED}%")
        print(f"走行時間: {FORWARD_TIME_S}秒")

        # 前進
        driver.drive(FORWARD_SPEED)
        time.sleep(FORWARD_TIME_S)

        # 停止
        driver.stop()
        time.sleep(STOP_TIME_S)

        # 前方カメラで撮影
        print("前方カメラで撮影します")
        image_path = camera.capture(
            width=CAPTURE_WIDTH,
            height=CAPTURE_HEIGHT,
            hdr=CAPTURE_HDR,
            timeout_ms=CAPTURE_TIMEOUT_MS,
        )
        print(f"撮影画像: {image_path}")

        print("=== 直進テスト終了 ===")

    except KeyboardInterrupt:
        print("\n直進テストを中断しました")

    finally:
        # 必ずGPIOを解放する
        driver.cleanup()


if __name__ == "__main__":
    main()
