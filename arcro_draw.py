from arcro import Arcro


def main():
    arcro = Arcro()

    # 画像読み込み
    image = arcro.load_image("images/selfie_20260814_000947.jpg")

    # 撮影画像が左右反転している場合は反転
    image = arcro.flip_horizontal(image)

    # しきい値領域を描画
    threshold_image = arcro.draw_aruco_capture_check_threshold_area(
        image=image,
        target_center_x=623,
        target_center_y=516,
        position_tolerance_x=150,
        position_tolerance_y=150,
        min_area_ratio=0.0008,
        max_area_ratio=0.25
    )

    # ArUcoマーカー検出
    result = arcro.detect_single_aruco_marker_for_capture_check(
        image=image,
        target_center_x=623,
        target_center_y=516,
        position_tolerance_x=150,
        position_tolerance_y=150,
        min_area_ratio=0.0008,
        max_area_ratio=0.25,
        tilt_tolerance_deg=30
    )

    # 検出結果を描画
    result_image = arcro.draw_aruco_capture_check_result(
        threshold_image,
        result
    )

    # 保存
    arcro.save_image(
        result_image,
        "images/aruco_check_result.JPG"
    )

    # コンソール出力
    print("===== ArUco 検出結果 =====")
    print(f"検出: {result['is_detected']}")
    print(f"撮影OK: {result['is_capture_ok']}")
    print(f"理由: {result['reason']}")

    if result["is_detected"]:
        print(f"ID: {result['marker_id']}")
        print(f"中心座標: ({result['center_x']:.1f}, {result['center_y']:.1f})")
        print(f"目標中心: ({result['target_center_x']:.1f}, {result['target_center_y']:.1f})")
        print(f"中心ずれX: {result['center_error_x']:.1f}")
        print(f"中心ずれY: {result['center_error_y']:.1f}")
        print(f"位置OK: {result['is_position_ok']}")
        print(f"面積比: {result['marker_area_ratio']:.4f}")
        print(f"面積OK: {result['is_area_ok']}")
        print(f"傾き: {result['tilt_deg']:.1f} deg")
        print(f"傾きOK: {result['is_tilt_ok']}")


if __name__ == "__main__":
    main()