#!/usr/bin/env python3
"""image_processorのcompress_imageだけを実行する簡易スクリプト。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from image_processor import ImageProcessor


SCRIPT_DIR = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compress an image with ImageProcessor.compress_image.")
    parser.add_argument("--image", type=Path, required=True, help="圧縮元画像のパス")
    parser.add_argument(
        "--output",
        type=Path,
        help="圧縮後画像の保存先。省略時はこのスクリプトと同じ階層に保存します。",
    )
    return parser.parse_args()


def default_output_path(image_path: Path) -> Path:
    return SCRIPT_DIR / f"{image_path.stem}_compressed.jpg"


def main() -> None:
    args = parse_args()

    processor = ImageProcessor()
    image = processor.load_image(args.image)
    output_path = args.output if args.output else default_output_path(args.image)

    processor.compress_image(image, output_path)


if __name__ == "__main__":
    main()
