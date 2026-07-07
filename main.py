from image_processor import ImageProcessor


def main():
    processor = ImageProcessor()

    # 画像読み込み
    image = processor.load_image("images/test13.JPG")

    # ARマーカー検出
    result = processor.detect_single_aruco_marker_for_capture_check(
        image=image
    )

    print("===== 検出結果 =====")
    print(f"検出: {result['is_detected']}")

    if result["is_detected"]:
        print(f"ID: {result['marker_id']}")
        print(f"中心座標: ({result['center_x']:.1f}, {result['center_y']:.1f})")
        print(f"傾き: {result['tilt_deg']:.1f}°")
        print(f"面積比: {result['marker_area_ratio']:.4f}")
        print(f"撮影OK: {result['is_capture_ok']}")

    print(f"理由: {result['reason']}")


if __name__ == "__main__":
    main()