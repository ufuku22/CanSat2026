from image_processor import ImageProcessor


def main():
    # 処理したい元画像
    image_path = "images/sample1.jpeg"

    # 出力先
    compressed_output_path = "output/compressed_sample1.jpeg"
    red_mask_output_path = "output/red_mask_sample1.jpeg"

    # JPEG圧縮品質
    quality = 50

    # ImageProcessorを作成
    processor = ImageProcessor()

    # 1. 画像を取得・読み込み
    image = processor.load_image(image_path)

    print("画像を読み込みました")
    print(f"画像パス: {image_path}")
    print(f"画像サイズ: {image.shape}")

    # 2. 赤色検出
    red_ratio, red_mask = processor.detect_red_ratio(image)

    print("赤色検出結果")
    print(f"赤色占有率: {red_ratio:.4f}")
    print(f"赤色占有率: {red_ratio * 100:.2f} %")

    # 赤色マスク画像を保存
    processor.save_image(
        image=red_mask,
        output_path=red_mask_output_path
    )

    # 3. 元画像を圧縮して保存
    processor.compress_image(
        image=image,
        output_path=compressed_output_path,
        quality=quality
    )


if __name__ == "__main__":
    main()