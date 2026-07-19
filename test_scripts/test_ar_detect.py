import sys
from pathlib import Path

# ==============================
# image_processor.py を読み込むためのパス設定
# ==============================
CURRENT_FILE = Path(__file__).resolve()
PROJECT_ROOT = CURRENT_FILE.parents[1]

sys.path.append(str(PROJECT_ROOT))

from image_processor import ImageProcessor


def resolve_image_path(input_path):
    """
    入力された画像パスを解決する。

    対応例:
        images/test.JPG
        ../images/test.JPG
        /home/pi/project/images/test.JPG
    """

    path = Path(input_path).expanduser()

    # 絶対パスならそのまま
    if path.is_absolute():
        return path

    # 実行した場所からの相対パス
    cwd_path = Path.cwd() / path
    if cwd_path.exists():
        return cwd_path

    # projectルートからの相対パス
    project_path = PROJECT_ROOT / path
    if project_path.exists():
        return project_path

    return project_path


def main():
    processor = ImageProcessor()

    print("===== ArUco マーカー検出テスト =====")
    print("検出したい画像パスを入力してください。")
    print("例: images/test.JPG")
    print("例: /home/pi/project/images/test.JPG")
    print()

    input_path = input("画像パス: ").strip().strip('"').strip("'")

    image_path = resolve_image_path(input_path)

    if not image_path.exists():
        print(f"画像ファイルが見つかりません: {image_path}")
        return

    # 画像読み込み
    image = processor.load_image(image_path)

    # 左右反転はしない

    # ArUcoマーカー検出
    result = processor.detect_single_aruco_marker_for_capture_check(
        image=image,
        target_center_x=1135,
        target_center_y=1220,
        position_tolerance_x=160,
        position_tolerance_y=120,
        min_area_ratio=0.0015,
        max_area_ratio=0.10
    )

    print()
    print("===== 検出結果 =====")
    print(f"画像パス: {image_path}")
    print(f"検出: {result['is_detected']}")
    print(f"撮影OK: {result['is_capture_ok']}")
    print(f"理由: {result['reason']}")

    if result["is_detected"]:
        print()
        print("----- マーカー情報 -----")
        print(f"マーカーID: {result['marker_id']}")
        print(f"中心座標: ({result['center_x']:.1f}, {result['center_y']:.1f})")
        print(f"目標中心: ({result['target_center_x']:.1f}, {result['target_center_y']:.1f})")
        print(f"中心ずれX: {result['center_error_x']:.1f}")
        print(f"中心ずれY: {result['center_error_y']:.1f}")
        print(f"位置OK: {result['is_position_ok']}")
        print(f"傾き: {result['tilt_deg']:.1f} deg")
        print(f"面積px: {result['marker_area_px']:.1f}")
        print(f"面積比: {result['marker_area_ratio']:.6f}")
        print(f"面積下限OK: {result['is_area_large_enough']}")
        print(f"面積上限OK: {result['is_area_small_enough']}")
        print(f"面積OK: {result['is_area_ok']}")


if __name__ == "__main__":
    main()