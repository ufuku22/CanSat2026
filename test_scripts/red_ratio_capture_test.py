#!/usr/bin/env python3
"""Enterのたびに撮影して赤検知率を表示するテスト。"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import CameraCaptureConfig, RedConeConfig
from image_processor import ImageProcessor
from sensor_manager import SensorManager


DEFAULT_SAVE_DIR = PROJECT_ROOT / "cansat_camera_images"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="撮影画像の赤検知率を繰り返し表示します。")
    parser.add_argument("--width", type=int, default=CameraCaptureConfig.WIDTH)
    parser.add_argument("--height", type=int, default=CameraCaptureConfig.HEIGHT)
    parser.add_argument("--timeout-ms", type=int, default=CameraCaptureConfig.TIMEOUT_MS)
    parser.add_argument("--hdr", action="store_true", dest="hdr")
    parser.add_argument("--no-hdr", action="store_false", dest="hdr")
    parser.set_defaults(hdr=CameraCaptureConfig.HDR)
    parser.add_argument("--red-threshold", type=float, default=RedConeConfig.RED_THRESHOLD)
    parser.add_argument(
        "--column-threshold",
        type=float,
        default=RedConeConfig.RED_COLUMN_THRESHOLD,
    )
    parser.add_argument(
        "--column-average-width",
        type=int,
        default=RedConeConfig.RED_COLUMN_AVERAGE_WIDTH,
    )
    parser.add_argument("--save-dir", type=Path, default=DEFAULT_SAVE_DIR)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    processor = ImageProcessor()

    print("赤検知率テスト: Enterで撮影、q + Enterで終了")
    print(f"red_threshold={args.red_threshold}")

    with SensorManager(camera_save_dir=args.save_dir) as sensors:
        count = 1
        while True:
            command = input(f"\n[{count}] Enterで撮影 > ").strip().lower()
            if command in {"q", "quit", "exit"}:
                break

            frame = sensors.camera.capture_frame(
                width=args.width,
                height=args.height,
                hdr=args.hdr,
                timeout_ms=args.timeout_ms,
            )
            result = processor.detect_color(
                frame,
                hsv_ranges=processor.RED_HSV_RANGES,
                color_threshold=args.red_threshold,
                column_threshold=args.column_threshold,
                column_average_width=args.column_average_width,
            )
            result.pop("color_mask", None)

            print(
                f"赤検知率={result['total_color_ratio'] * 100:.3f}% "
                f"detected={result['is_color_detected']} "
                f"peak_column={result['color_peak_column_x']}"
            )
            count += 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
