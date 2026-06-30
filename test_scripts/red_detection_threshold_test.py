#!/usr/bin/env python3
"""赤検知のHSVしきい値と赤色割合しきい値を単体で確認するテスト。"""

from __future__ import annotations

import argparse
import time
from datetime import datetime
from pathlib import Path
import sys

try:
    import cv2
    import numpy as np
except ModuleNotFoundError as exc:
    cv2 = None
    np = None
    CV_IMPORT_ERROR = exc
else:
    CV_IMPORT_ERROR = None

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from sensor_manager import CAMERA_FULL_HD_HEIGHT, CAMERA_FULL_HD_WIDTH, SensorManager


DEFAULT_SAVE_DIR = PROJECT_ROOT / "red_detection_results"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "赤コーン検知をモータなしで確認します。"
            "画像ファイル指定がなければ前方カメラで撮影します。"
        )
    )
    parser.add_argument("--image", type=Path, help="既存画像でテストする場合の画像パス")
    parser.add_argument("--width", type=int, default=CAMERA_FULL_HD_WIDTH, help="撮影画像の幅[px]")
    parser.add_argument("--height", type=int, default=CAMERA_FULL_HD_HEIGHT, help="撮影画像の高さ[px]")
    parser.add_argument("--timeout-ms", type=int, default=2000, help="撮影前の待ち時間[ms]")
    parser.add_argument("--hdr", action="store_true", help="HDR撮影を有効にする")
    parser.add_argument("--repeat", type=int, default=1, help="撮影して判定する回数")
    parser.add_argument("--interval", type=float, default=1.0, help="繰り返し時の待ち時間[秒]")

    parser.add_argument("--h1-min", type=int, default=0, help="赤範囲1のH下限")
    parser.add_argument("--h1-max", type=int, default=10, help="赤範囲1のH上限")
    parser.add_argument("--h2-min", type=int, default=160, help="赤範囲2のH下限")
    parser.add_argument("--h2-max", type=int, default=180, help="赤範囲2のH上限")
    parser.add_argument("--s-min", type=int, default=100, help="S下限")
    parser.add_argument("--s-max", type=int, default=255, help="S上限")
    parser.add_argument("--v-min", type=int, default=100, help="V下限")
    parser.add_argument("--v-max", type=int, default=255, help="V上限")
    parser.add_argument("--morph-kernel", type=int, default=0, help="ノイズ除去用カーネルサイズ。0で無効")

    parser.add_argument("--red-threshold", type=float, default=0.02, help="全体赤検知しきい値")
    parser.add_argument(
        "--block-threshold",
        type=float,
        default=0.03,
        help="5分割方向判定しきい値",
    )
    parser.add_argument("--save-dir", type=Path, default=DEFAULT_SAVE_DIR, help="結果画像の保存先")
    parser.add_argument("--no-save", action="store_true", help="マスクと重ね合わせ画像を保存しない")
    parser.add_argument("--show", action="store_true", help="OpenCVウィンドウで結果を表示する")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    for name in ("h1_min", "h1_max", "h2_min", "h2_max"):
        value = getattr(args, name)
        if not 0 <= value <= 180:
            raise ValueError(f"{name}は0から180で指定してください: {value}")
    for name in ("s_min", "s_max", "v_min", "v_max"):
        value = getattr(args, name)
        if not 0 <= value <= 255:
            raise ValueError(f"{name}は0から255で指定してください: {value}")
    if args.h1_min > args.h1_max or args.h2_min > args.h2_max:
        raise ValueError("Hの下限は上限以下にしてください。")
    if args.s_min > args.s_max or args.v_min > args.v_max:
        raise ValueError("S/Vの下限は上限以下にしてください。")
    if args.repeat <= 0:
        raise ValueError("--repeatは1以上で指定してください。")
    if not 0.0 <= args.red_threshold <= 1.0:
        raise ValueError("--red-thresholdは0.0から1.0で指定してください。")
    if not 0.0 <= args.block_threshold <= 1.0:
        raise ValueError("--block-thresholdは0.0から1.0で指定してください。")


def load_or_capture_image(args: argparse.Namespace, sensors: SensorManager | None) -> tuple[np.ndarray, Path | None]:
    if args.image is not None:
        image = cv2.imread(str(args.image))
        if image is None:
            raise ValueError(f"画像を読み込めませんでした: {args.image}")
        return image, args.image

    if sensors is None:
        raise RuntimeError("センサ管理クラスが初期化されていません。")
    image_path = sensors.capture_front_image(
        width=args.width,
        height=args.height,
        hdr=args.hdr,
        timeout_ms=args.timeout_ms,
    )
    image = cv2.imread(str(image_path))
    if image is None:
        raise ValueError(f"撮影画像を読み込めませんでした: {image_path}")
    return image, image_path


def detect_red(image: np.ndarray, args: argparse.Namespace) -> dict[str, object]:
    height, width = image.shape[:2]
    total_pixels = height * width
    if total_pixels == 0:
        raise ValueError("画像サイズが不正です。")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([args.h1_min, args.s_min, args.v_min])
    upper_red1 = np.array([args.h1_max, args.s_max, args.v_max])
    lower_red2 = np.array([args.h2_min, args.s_min, args.v_min])
    upper_red2 = np.array([args.h2_max, args.s_max, args.v_max])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    if args.morph_kernel > 1:
        kernel_size = args.morph_kernel
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    total_red_ratio = cv2.countNonZero(red_mask) / total_pixels

    block_count = 5
    block_width = width // block_count
    block_ratios = []
    for index in range(block_count):
        start_x = index * block_width
        end_x = (index + 1) * block_width if index < block_count - 1 else width
        block_mask = red_mask[:, start_x:end_x]
        block_area = block_mask.shape[0] * block_mask.shape[1]
        block_ratios.append(cv2.countNonZero(block_mask) / block_area if block_area else 0.0)

    direction_names = ["left_far", "left", "center", "right", "right_far"]
    max_index = int(np.argmax(block_ratios))
    max_ratio = block_ratios[max_index]
    red_direction = direction_names[max_index] if max_ratio >= args.block_threshold else "none"

    return {
        "red_mask": red_mask,
        "total_red_ratio": total_red_ratio,
        "is_red_detected": total_red_ratio >= args.red_threshold,
        "red_block_ratios": block_ratios,
        "red_direction": red_direction,
        "red_block_number": max_index + 1 if red_direction != "none" else None,
    }


def make_overlay(image: np.ndarray, mask: np.ndarray, result: dict[str, object], args: argparse.Namespace) -> np.ndarray:
    overlay = image.copy()
    red_layer = np.zeros_like(image)
    red_layer[:, :, 2] = mask
    overlay = cv2.addWeighted(overlay, 0.75, red_layer, 0.25, 0)

    height, width = image.shape[:2]
    block_width = width // 5
    for index in range(1, 5):
        x = index * block_width
        cv2.line(overlay, (x, 0), (x, height), (255, 255, 255), 2)

    block_ratios = result["red_block_ratios"]
    for index, ratio in enumerate(block_ratios):
        start_x = index * block_width
        end_x = (index + 1) * block_width if index < 4 else width
        label = f"{index + 1}:{ratio * 100:.1f}%"
        cv2.putText(
            overlay,
            label,
            (start_x + 12, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        if ratio >= args.block_threshold:
            cv2.rectangle(overlay, (start_x, 0), (end_x - 1, height - 1), (0, 255, 255), 3)

    summary = (
        f"total={result['total_red_ratio'] * 100:.2f}% "
        f"detected={result['is_red_detected']} "
        f"direction={result['red_direction']}"
    )
    cv2.putText(
        overlay,
        summary,
        (12, height - 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return overlay


def print_result(result: dict[str, object], image_path: Path | None, args: argparse.Namespace) -> None:
    print("===== 赤検知テスト =====")
    if image_path is not None:
        print(f"画像: {image_path}")
    print(
        "HSV範囲: "
        f"H={args.h1_min}-{args.h1_max} / {args.h2_min}-{args.h2_max}, "
        f"S={args.s_min}-{args.s_max}, V={args.v_min}-{args.v_max}"
    )
    print(f"全体赤割合: {result['total_red_ratio'] * 100:.2f} %")
    print(f"全体赤検知しきい値: {args.red_threshold * 100:.2f} %")
    print(f"赤検知: {result['is_red_detected']}")
    print(f"5分割方向判定しきい値: {args.block_threshold * 100:.2f} %")
    for index, ratio in enumerate(result["red_block_ratios"], start=1):
        mark = "OK" if ratio >= args.block_threshold else "--"
        print(f"  block {index}: {ratio * 100:.2f} % [{mark}]")
    print(f"赤方向: {result['red_direction']}")
    print(f"赤ブロック番号: {result['red_block_number']}")


def save_result_images(
    result: dict[str, object],
    overlay: np.ndarray,
    args: argparse.Namespace,
) -> None:
    args.save_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    mask_path = args.save_dir / f"red_mask_{stamp}.png"
    overlay_path = args.save_dir / f"red_overlay_{stamp}.jpg"

    if not cv2.imwrite(str(mask_path), result["red_mask"]):
        raise IOError(f"マスク画像の保存に失敗しました: {mask_path}")
    if not cv2.imwrite(str(overlay_path), overlay):
        raise IOError(f"重ね合わせ画像の保存に失敗しました: {overlay_path}")

    print(f"マスク画像: {mask_path}")
    print(f"重ね合わせ画像: {overlay_path}")


def main() -> int:
    args = parse_args()
    if CV_IMPORT_ERROR is not None:
        raise SystemExit(
            "OpenCVまたはNumPyを読み込めませんでした。"
            "実行環境に opencv-python と numpy を入れてください: "
            f"{CV_IMPORT_ERROR}"
        )
    validate_args(args)

    sensors = None if args.image is not None else SensorManager()
    try:
        for count in range(args.repeat):
            if count > 0:
                time.sleep(args.interval)

            image, image_path = load_or_capture_image(args, sensors)
            result = detect_red(image, args)
            overlay = make_overlay(image, result["red_mask"], result, args)
            print_result(result, image_path, args)

            if not args.no_save:
                save_result_images(result, overlay, args)

            if args.show:
                cv2.imshow("red mask", result["red_mask"])
                cv2.imshow("red overlay", overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        if sensors is not None:
            sensors.close()
        if args.show:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
