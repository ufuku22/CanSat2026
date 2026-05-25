import cv2
import numpy as np
from pathlib import Path


class ImageProcessor:
    """
    画像処理用クラス

    機能:
        - 画像を読み込む
        - 赤色を検出して占有率を計算する
        - 画像を圧縮して保存する
        - 画像を保存する
    """

    def __init__(self):
        pass

    def load_image(self, image_path):
        """
        指定された画像ファイルを読み込む
        """

        image_path = Path(image_path)

        if not image_path.exists():
            raise FileNotFoundError(f"画像ファイルが存在しません: {image_path}")

        image = cv2.imread(str(image_path))

        if image is None:
            raise ValueError(f"画像を読み込めませんでした: {image_path}")

        return image

    def detect_red_ratio(self, image):
        """
        画像中の赤色領域を検出し、赤色の占有率を返す

        Returns
        -------
        red_ratio : float
            赤色の占有率
            例: 0.25なら25%

        red_mask : numpy.ndarray
            赤色部分を白、それ以外を黒にした画像
        """

        # OpenCVの画像はBGR形式なので、HSV形式に変換する
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 赤色の範囲その1
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])

        # 赤色の範囲その2
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # 赤色範囲に入っている画素を白、それ以外を黒にする
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        # 2つの赤色範囲を合成する
        red_mask = cv2.bitwise_or(mask1, mask2)

        # 画像全体のピクセル数
        total_pixels = image.shape[0] * image.shape[1]

        # 赤色として検出されたピクセル数
        red_pixels = cv2.countNonZero(red_mask)

        # 赤色占有率
        red_ratio = red_pixels / total_pixels

        return red_ratio, red_mask

    def save_image(self, image, output_path):
        """
        画像をそのまま保存する
        """

        output_path = Path(output_path)

        # 保存先フォルダがなければ作成する
        output_path.parent.mkdir(parents=True, exist_ok=True)

        success = cv2.imwrite(str(output_path), image)

        if not success:
            raise IOError(f"画像の保存に失敗しました: {output_path}")

        print(f"画像を保存しました: {output_path}")

    def compress_image(self, image, output_path, quality=70):
        """
        画像をJPEG形式で圧縮して保存する

        Parameters
        ----------
        image : numpy.ndarray
            OpenCVで読み込んだ画像データ

        output_path : str
            圧縮後の画像を保存するパス

        quality : int
            JPEG品質
            1〜100で指定する
            大きいほど高画質
            小さいほど高圧縮
        """

        output_path = Path(output_path)

        # 保存先フォルダがなければ作成する
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # JPEG品質を指定
        encode_param = [
            int(cv2.IMWRITE_JPEG_QUALITY),
            int(quality)
        ]

        success = cv2.imwrite(
            str(output_path),
            image,
            encode_param
        )

        if not success:
            raise IOError(f"圧縮画像の保存に失敗しました: {output_path}")

        print(f"圧縮画像を保存しました: {output_path}")