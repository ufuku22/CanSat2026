#!/usr/bin/env python3
"""Raspberry Pi Camera Module V3で静止画を撮影して保存する簡易テスト。"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor_manager import CameraV3


DEFAULT_SAVE_DIR = PROJECT_ROOT / "cansat_camera_images"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ラズパイカメラで静止画を撮影して保存します。")
    parser.add_argument("--width", type=int, default=1920, help="撮影画像の幅[px]")
    parser.add_argument("--height", type=int, default=1080, help="撮影画像の高さ[px]")
    parser.add_argument("--timeout-ms", type=int, default=2000, help="撮影前の待ち時間[ms]")
    parser.add_argument("--hdr", action="store_true", help="HDR撮影を有効にする")
    parser.add_argument(
        "--save-dir",
        type=Path,
        default=DEFAULT_SAVE_DIR,
        help="画像の保存先ディレクトリ",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    camera = CameraV3(save_dir=args.save_dir)

    try:
        image_path = camera.capture(
            width=args.width,
            height=args.height,
            hdr=args.hdr,
            timeout_ms=args.timeout_ms,
        )
    finally:
        camera.close()

    print(f"撮影画像を保存しました: {image_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
