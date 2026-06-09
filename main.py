from image_processor import ImageProcessor


def main():
    # 処理したい元画像
    # image_path = "images/sample1.jpeg"
    image_path = "images/sample1.jpeg"
    # image_path = "aruco/aruco_marker_1.png"

    
    processor = ImageProcessor()

    image = processor.load_image(image_path)

    # result = processor.detect_single_aruco_marker_for_capture_check(
    #     image=image,
    #     min_area_ratio=0.005,
    #     center_tolerance_ratio=0.30,
    #     edge_margin_ratio=0.05
    # )

    # print("===== ArUco撮影判定結果 =====")

    # if not result["is_detected"]:
    #     print("マーカーは検出されませんでした")
    #     print(f"理由: {result['reason']}")
    #     return

    # print(f"マーカーID: {result['marker_id']}")

    # print(f"中心座標: ({result['center_x']:.1f}, {result['center_y']:.1f})")
    # print(f"画像中心: ({result['image_center_x']:.1f}, {result['image_center_y']:.1f})")

    # print(f"x方向の中心ずれ: {result['center_error_x']:.1f} px")
    # print(f"y方向の中心ずれ: {result['center_error_y']:.1f} px")

    # print(f"画像上の傾き: {result['tilt_deg']:.1f} deg")

    # print(f"マーカー面積: {result['marker_area_px']:.1f} px")
    # print(f"マーカー面積割合: {result['marker_area_ratio'] * 100:.2f} %")

    # print(f"中心付近にあるか: {result['is_near_center']}")
    # print(f"十分な大きさか: {result['is_large_enough']}")
    # print(f"画像端から離れているか: {result['is_inside_margin']}")

    # print(f"撮影正常判定: {result['is_capture_ok']}")
    # print(f"理由: {result['reason']}")


    # 出力先
    # compressed_output_path = "output/compressed_sample1.jpeg"
    # red_mask_output_path = "output/red_mask_sample1.jpeg"

    # # JPEG圧縮品質
    # quality = 50

    # # ImageProcessorを作成
    # processor = ImageProcessor()

    # # 1. 画像を取得・読み込み
    # image = processor.load_image(image_path)

    # print("画像を読み込みました")
    # print(f"画像パス: {image_path}")
    # print(f"画像サイズ: {image.shape}")

    # # 2. 赤色検出
    # red_ratio, red_mask = processor.detect_red_ratio(image)

    # print("赤色検出結果")
    # print(f"赤色占有率: {red_ratio:.4f}")
    # print(f"赤色占有率: {red_ratio * 100:.2f} %")

    # # 赤色マスク画像を保存
    # processor.save_image(
    #     image=red_mask,
    #     output_path=red_mask_output_path
    # )

    # # 3. 元画像を圧縮して保存
    # processor.compress_image(
    #     image=image,
    #     output_path=compressed_output_path,
    #     quality=quality
    # )

    # output_image = processor.draw_aruco_capture_check_result(
    #         image=image,
    #         result=result
    #     )

    # processor.save_image(
    #         image=output_image,
    #         output_path="output/aruco_capture_check.jpeg"
    #     )
    
    # result = processor.detect_red_regions(
    #     image=image,
    #     red_threshold=0.05,
    #     center_width_ratio=0.4
    # )
    
    # print("===== 赤色領域検出結果 =====")
    # print(f"赤色検出: {result['is_red_detected']}")
    # print(f"全体の赤色割合: {result['total_red_ratio'] * 100:.2f} %")

    # print(f"左の赤色割合: {result['left_red_ratio'] * 100:.2f} %")
    # print(f"中央の赤色割合: {result['center_red_ratio'] * 100:.2f} %")
    # print(f"右の赤色割合: {result['right_red_ratio'] * 100:.2f} %")

    # print(f"左に赤色あり: {result['is_red_left']}")
    # print(f"正面に赤色あり: {result['is_red_center']}")
    # print(f"右に赤色あり: {result['is_red_right']}")

    # print(f"赤色が最も多い方向: {result['red_direction']}")

    # if result["is_red_in_front"]:
    #     print("正面に赤色があります")

    # if result["red_direction"] == "left":
    #     print("赤色は左側に多いです")
    # elif result["red_direction"] == "center":
    #     print("赤色は正面に多いです")
    # elif result["red_direction"] == "right":
    #     print("赤色は右側に多いです")
    # else:
    #     print("赤色は検出されませんでした")

    
    red_result = processor.detect_red(
        image=image,
        red_threshold=0.05,
        center_width_ratio=0.4
    )

    print("===== 赤色検出結果 =====")
    print(f"赤色検出: {red_result['is_red_detected']}")
    print(f"全体赤色割合: {red_result['total_red_ratio'] * 100:.2f} %")

    print(f"左赤色割合: {red_result['left_red_ratio'] * 100:.2f} %")
    print(f"中央赤色割合: {red_result['center_red_ratio'] * 100:.2f} %")
    print(f"右赤色割合: {red_result['right_red_ratio'] * 100:.2f} %")

    print(f"赤色方向: {red_result['red_direction']}")
    print(f"正面に赤色あり: {red_result['is_red_in_front']}")
    print(f"理由: {red_result['reason']}")

    processor.save_image(
        image=red_result["red_mask"],
        output_path="output/red_mask.jpeg"
    )



if __name__ == "__main__":
    main()