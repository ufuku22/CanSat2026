import cv2
import numpy as np
from pathlib import Path
import math


class ImageProcessor:
    """
    画像処理用クラス

    機能:
        - 画像を読み込む
        - 赤色を検出して占有率を計算する
        - 画像を圧縮して保存する
        - 画像を保存する
        - ARマーカーを検出する 
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


    def detect_red(
        self,
        image,
        red_threshold=0.05,
        center_width_ratio=0.4
    ):
        """
        画像中の赤色を検出し、
        全体・左・中央・右それぞれの赤色割合を返す。

        用途:
            - 赤色占有率の取得
            - 赤色パラシュート回避
            - 赤色ゴール検出
            - 赤色が左・中央・右のどこに多いかの判定

        Parameters
        ----------
        image : numpy.ndarray
            OpenCVで読み込んだ画像データ

        red_threshold : float
            赤色を検出したと判断するしきい値
            例:
                0.05 = 画像領域の5%以上が赤なら検出あり

        center_width_ratio : float
            中央領域の幅の割合
            例:
                0.4 = 画像幅の中央40%を正面領域とする

        Returns
        -------
        result : dict
            赤色検出結果
        """

        height, width = image.shape[:2]
        total_pixels = height * width

        if total_pixels == 0:
            return {
                "is_red_detected": False,
                "total_red_ratio": 0.0,

                "left_red_ratio": 0.0,
                "center_red_ratio": 0.0,
                "right_red_ratio": 0.0,

                "is_red_left": False,
                "is_red_center": False,
                "is_red_right": False,
                "is_red_in_front": False,

                "red_direction": "none",

                "center_start_x": 0,
                "center_end_x": 0,

                "red_mask": None,
                "reason": "画像サイズが不正です"
            }

        # BGR画像をHSV画像に変換
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 赤色の範囲1
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])

        # 赤色の範囲2
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # 赤色マスクを作成
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        red_mask = cv2.bitwise_or(mask1, mask2)

        # ==============================
        # 画像全体の赤色割合
        # ==============================
        total_red_pixels = cv2.countNonZero(red_mask)
        total_red_ratio = total_red_pixels / total_pixels

        # ==============================
        # 左・中央・右に分割
        # ==============================
        center_width = int(width * center_width_ratio)

        center_start_x = int((width - center_width) / 2)
        center_end_x = center_start_x + center_width

        left_mask = red_mask[:, 0:center_start_x]
        center_mask = red_mask[:, center_start_x:center_end_x]
        right_mask = red_mask[:, center_end_x:width]

        def calculate_ratio(mask):
            area = mask.shape[0] * mask.shape[1]

            if area == 0:
                return 0.0

            red_pixels = cv2.countNonZero(mask)
            return red_pixels / area

        left_red_ratio = calculate_ratio(left_mask)
        center_red_ratio = calculate_ratio(center_mask)
        right_red_ratio = calculate_ratio(right_mask)

        # ==============================
        # 各領域の赤色判定
        # ==============================
        is_red_detected = total_red_ratio >= red_threshold

        is_red_left = left_red_ratio >= red_threshold
        is_red_center = center_red_ratio >= red_threshold
        is_red_right = right_red_ratio >= red_threshold

        is_red_in_front = is_red_center

        # ==============================
        # 赤色が最も多い方向
        # ==============================
        region_ratios = {
            "left": left_red_ratio,
            "center": center_red_ratio,
            "right": right_red_ratio
        }

        max_region = max(region_ratios, key=region_ratios.get)
        max_ratio = region_ratios[max_region]

        if max_ratio < red_threshold:
            red_direction = "none"
        else:
            red_direction = max_region

        # ==============================
        # 理由
        # ==============================
        if not is_red_detected:
            reason = "赤色は検出されませんでした"
        elif red_direction == "left":
            reason = "赤色は左側に多く検出されました"
        elif red_direction == "center":
            reason = "赤色は正面に多く検出されました"
        elif red_direction == "right":
            reason = "赤色は右側に多く検出されました"
        else:
            reason = "赤色方向を判定できませんでした"

        result = {
            # 全体情報
            "is_red_detected": bool(is_red_detected),
            "total_red_ratio": float(total_red_ratio),

            # 分割領域ごとの赤色割合
            "left_red_ratio": float(left_red_ratio),
            "center_red_ratio": float(center_red_ratio),
            "right_red_ratio": float(right_red_ratio),

            # 分割領域ごとの赤色有無
            "is_red_left": bool(is_red_left),
            "is_red_center": bool(is_red_center),
            "is_red_right": bool(is_red_right),

            # 正面判定
            "is_red_in_front": bool(is_red_in_front),

            # 赤色が最も多い方向
            "red_direction": red_direction,

            # 分割位置
            "center_start_x": int(center_start_x),
            "center_end_x": int(center_end_x),

            # 赤色マスク画像
            "red_mask": red_mask,

            # 判定理由
            "reason": reason
        }

        return result

        
    def detect_single_aruco_marker_for_capture_check(
        self,
        image,
        min_area_ratio=0.005,
        center_tolerance_ratio=0.30,
        edge_margin_ratio=0.05
    ):
        """
        画像からArUcoマーカーを1つ検出し、
        撮影が正常に行われたかを判断するための情報を取得する。

        使用するArUco辞書:
            cv2.aruco.DICT_4X4_50

        Parameters
        ----------
        image : numpy.ndarray
            OpenCVで読み込んだ画像データ

        min_area_ratio : float
            マーカー面積が画像全体の何割以上あればOKとするか
            例:
                0.005 = 0.5%

        center_tolerance_ratio : float
            画像中心からどれくらいズレてもOKとするか
            画像幅・高さに対する割合で指定
            例:
                0.30 = 画像中心から幅・高さの30%以内ならOK

        edge_margin_ratio : float
            画像端からどれくらい離れていればOKとするか
            例:
                0.05 = 画像端から5%以上離れていればOK

        Returns
        -------
        result : dict
            検出結果と撮影正常判定に必要な情報
        """

        height, width = image.shape[:2]
        image_area = width * height

        # 初期結果
        result = {
            "is_detected": False,
            "is_capture_ok": False,

            "marker_id": None,

            "center_x": None,
            "center_y": None,

            "image_center_x": width / 2,
            "image_center_y": height / 2,

            "center_error_x": None,
            "center_error_y": None,
            "center_error_ratio_x": None,
            "center_error_ratio_y": None,

            "corners": None,

            "tilt_deg": None,

            "marker_area_px": None,
            "marker_area_ratio": None,

            "bbox_x": None,
            "bbox_y": None,
            "bbox_w": None,
            "bbox_h": None,

            "is_near_center": False,
            "is_large_enough": False,
            "is_inside_margin": False,

            "reason": "マーカーが検出されませんでした"
        }

        # グレースケール化
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # ArUco辞書を指定
        aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        # OpenCVのバージョン差を吸収して検出
        parameters = cv2.aruco.DetectorParameters()

        if hasattr(cv2.aruco, "ArucoDetector"):
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, rejected = detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                aruco_dict,
                parameters=parameters
            )

        # マーカー未検出
        if ids is None or len(ids) == 0:
            return result

        # 今回は「1つのArUcoマーカー」を使う想定
        # 複数見つかった場合は、一番面積が大きいものを使う
        largest_index = 0
        largest_area = 0

        for i, marker_corners in enumerate(corners):
            points = marker_corners[0]
            area = cv2.contourArea(points)

            if area > largest_area:
                largest_area = area
                largest_index = i

        marker_corners = corners[largest_index]
        marker_id = int(ids[largest_index][0])
        points = marker_corners[0]

        # 四隅座標
        # 通常、順番は 左上・右上・右下・左下
        top_left = points[0]
        top_right = points[1]
        bottom_right = points[2]
        bottom_left = points[3]

        # 中心座標
        center_x = float(np.mean(points[:, 0]))
        center_y = float(np.mean(points[:, 1]))

        # 画像中心からのズレ
        image_center_x = width / 2
        image_center_y = height / 2

        center_error_x = center_x - image_center_x
        center_error_y = center_y - image_center_y

        center_error_ratio_x = abs(center_error_x) / width
        center_error_ratio_y = abs(center_error_y) / height

        # マーカーの画像上の傾き
        dx = top_right[0] - top_left[0]
        dy = top_right[1] - top_left[1]

        tilt_deg = math.degrees(math.atan2(dy, dx))

        # マーカー面積
        marker_area_px = float(cv2.contourArea(points))
        marker_area_ratio = marker_area_px / image_area

        # 外接矩形
        x, y, w, h = cv2.boundingRect(points.astype(np.float32))

        # 中心付近にあるか
        is_near_center = (
            center_error_ratio_x <= center_tolerance_ratio
            and center_error_ratio_y <= center_tolerance_ratio
        )

        # 十分な大きさか
        is_large_enough = marker_area_ratio >= min_area_ratio

        # 画像端に近すぎないか
        margin_x = width * edge_margin_ratio
        margin_y = height * edge_margin_ratio

        min_x = np.min(points[:, 0])
        max_x = np.max(points[:, 0])
        min_y = np.min(points[:, 1])
        max_y = np.max(points[:, 1])

        is_inside_margin = (
            min_x > margin_x
            and max_x < width - margin_x
            and min_y > margin_y
            and max_y < height - margin_y
        )

        # 撮影正常判定
        is_capture_ok = (
            is_near_center
            and is_large_enough
            and is_inside_margin
        )

        # 理由を作成
        reasons = []

        if not is_near_center:
            reasons.append("マーカーが画像中心から大きくずれています")

        if not is_large_enough:
            reasons.append("マーカーが小さすぎます")

        if not is_inside_margin:
            reasons.append("マーカーが画像端に近すぎます")

        if is_capture_ok:
            reason = "撮影は正常と判断されます"
        else:
            reason = " / ".join(reasons)

        # 結果をまとめる
        result = {
            "is_detected": True,
            "is_capture_ok": is_capture_ok,

            "marker_id": marker_id,

            "center_x": center_x,
            "center_y": center_y,

            "image_center_x": image_center_x,
            "image_center_y": image_center_y,

            "center_error_x": float(center_error_x),
            "center_error_y": float(center_error_y),
            "center_error_ratio_x": float(center_error_ratio_x),
            "center_error_ratio_y": float(center_error_ratio_y),

            "corners": points,

            "tilt_deg": float(tilt_deg),

            "marker_area_px": marker_area_px,
            "marker_area_ratio": float(marker_area_ratio),

            "bbox_x": int(x),
            "bbox_y": int(y),
            "bbox_w": int(w),
            "bbox_h": int(h),

            "is_near_center": is_near_center,
            "is_large_enough": is_large_enough,
            "is_inside_margin": is_inside_margin,

            "reason": reason
        }

        return result


    def draw_aruco_capture_check_result(self, image, result):
        """
        ArUcoマーカーの検出結果を画像に描画する
        """

        output_image = image.copy()

        if not result["is_detected"]:
            cv2.putText(
                output_image,
                "ArUco marker not detected",
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2
            )
            return output_image

        corners = result["corners"].astype(np.int32)

        # マーカーの外枠を描画
        cv2.polylines(
            output_image,
            [corners],
            isClosed=True,
            color=(0, 255, 0),
            thickness=2
        )

        # 中心点を描画
        center_x = int(result["center_x"])
        center_y = int(result["center_y"])

        cv2.circle(
            output_image,
            (center_x, center_y),
            5,
            (0, 0, 255),
            -1
        )

        # 画像中心を描画
        image_center_x = int(result["image_center_x"])
        image_center_y = int(result["image_center_y"])

        cv2.circle(
            output_image,
            (image_center_x, image_center_y),
            5,
            (255, 0, 0),
            -1
        )

        # 情報を文字で描画
        text_lines = [
            f"ID: {result['marker_id']}",
            f"Capture OK: {result['is_capture_ok']}",
            f"Center: ({result['center_x']:.1f}, {result['center_y']:.1f})",
            f"Tilt: {result['tilt_deg']:.1f} deg",
            f"Area: {result['marker_area_ratio'] * 100:.2f} %",
            f"Reason: {result['reason']}"
        ]

        x0 = 30
        y0 = 40
        line_height = 30

        for i, text in enumerate(text_lines):
            cv2.putText(
                output_image,
                text,
                (x0, y0 + i * line_height),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        return output_image
