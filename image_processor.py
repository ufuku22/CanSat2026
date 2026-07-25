import cv2
import numpy as np
from pathlib import Path
import math


class ImageProcessor:
    """
    画像処理用クラス

    機能:
        - 画像を読み込む
        - 複数のHSV範囲から任意色を検出して占有率を計算する
        - 画像を圧縮して保存する
        - 画像を保存する
        - ARマーカーを検出する 
    """

    RED_HSV_RANGES = [
        ((0, 100, 100), (10, 255, 255)),
        ((160, 100, 100), (179, 255, 255)),
    ]
    ORANGE_HSV_RANGES = [
        ((0, 150, 120), (12, 255, 255)),
        ((170, 150, 120), (179, 255, 255)),
    ]
    PURPLE_HSV_RANGES = [
        ((125, 80, 50), (160, 255, 255)),
    ]

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
    
    def flip_horizontal(self, image):
        """
        画像を左右反転する
        """
        
        filipped_image = cv2.flip(image, 1)
        
        return filipped_image

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

    def compress_image(
        self,
        image,
        output_path,
        max_width=320,
        max_height=240,
        quality=35,
        target_bytes=5000,
        max_bytes=6500,
        min_width=160,
        min_height=120,
        min_quality=15
    ):
        """
        画像をJPEG形式で圧縮して保存する

        Parameters
        ----------
        image : numpy.ndarray
            OpenCVで読み込んだ画像データ

        output_path : str
            圧縮後の画像を保存するパス

        max_width : int
            圧縮後の最大幅

        max_height : int
            圧縮後の最大高さ

        quality : int
            最初に試すJPEG品質

        target_bytes : int
            目標ファイルサイズ

        max_bytes : int
            許容する最大ファイルサイズ

        min_width : int
            圧縮時の最小幅

        min_height : int
            圧縮時の最小高さ

        min_quality : int
            JPEG品質の下限
        """

        output_path = Path(output_path)

        if image is None:
            raise ValueError("画像データが不正です")

        # 保存先フォルダがなければ作成する
        output_path.parent.mkdir(parents=True, exist_ok=True)

        source_height, source_width = image.shape[:2]
        best_size = None
        current_width = max_width
        current_height = max_height
        current_quality = quality

        for attempt in range(1, 25):
            scale = min(
                current_width / source_width,
                current_height / source_height,
                1.0
            )
            new_width = max(1, int(round(source_width * scale)))
            new_height = max(1, int(round(source_height * scale)))

            resized = image
            if (new_width, new_height) != (source_width, source_height):
                resized = cv2.resize(
                    image,
                    (new_width, new_height),
                    interpolation=cv2.INTER_AREA
                )

            success = cv2.imwrite(
                str(output_path),
                resized,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(current_quality)]
            )

            if not success:
                raise IOError(f"圧縮画像の保存に失敗しました: {output_path}")

            size = output_path.stat().st_size
            best_size = size
            print(
                "Image compression attempt "
                f"{attempt}: {new_width}x{new_height}, "
                f"quality={current_quality}, size={size} bytes"
            )

            if size <= max_bytes:
                if size > target_bytes:
                    print(f"Compressed image is within limit and near target: {size} bytes")
                print(f"圧縮画像を保存しました: {output_path}")
                return output_path

            if current_quality > min_quality:
                current_quality = max(min_quality, current_quality - 5)
                continue

            next_width = max(min_width, int(current_width * 0.85))
            next_height = max(min_height, int(current_height * 0.85))
            if (next_width, next_height) == (current_width, current_height):
                break
            current_width = next_width
            current_height = next_height
            current_quality = quality

        raise RuntimeError(
            f"Could not compress image under {max_bytes} bytes. "
            f"Best size was {best_size} bytes at minimum settings."
        )


    def detect_color(
        self,
        image,
        hsv_ranges,
        color_threshold=0.05,
        block_threshold=None,
        column_average_width=15,
    ):
        """指定された複数のHSV範囲に含まれる色の量と方向を返す。"""
        height, width = image.shape[:2]
        total_pixels = height * width

        if total_pixels == 0:
            return {
                "is_color_detected": False,
                "total_color_ratio": 0.0,
                "left_color_ratio": 0.0,
                "center_color_ratio": 0.0,
                "right_color_ratio": 0.0,
                "left_far_color_ratio": 0.0,
                "left_near_color_ratio": 0.0,
                "right_near_color_ratio": 0.0,
                "right_far_color_ratio": 0.0,
                "color_block_ratios": [0.0, 0.0, 0.0, 0.0, 0.0],
                "is_color_left": False,
                "is_color_center": False,
                "is_color_right": False,
                "is_color_left_far": False,
                "is_color_left_near": False,
                "is_color_right_near": False,
                "is_color_right_far": False,
                "is_color_in_front": False,
                "color_direction": "none",
                "color_block_number": None,
                "color_peak_column_x": None,
                "color_peak_column_ratio": 0.0,
                "color_peak_center_offset_ratio": None,
                "color_column_average_width": 0,
                "is_color_column_detected": False,
                "color_column_ratios": [],
                "smoothed_color_column_ratios": [],
                "center_start_x": 0,
                "center_end_x": 0,
                "color_mask": None,
                "reason": "画像サイズが不正です",
            }

        if block_threshold is None:
            block_threshold = color_threshold

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        color_mask = np.zeros((height, width), dtype=np.uint8)
        for lower_hsv, upper_hsv in hsv_ranges:
            range_mask = cv2.inRange(
                hsv_image,
                np.array(lower_hsv, dtype=np.uint8),
                np.array(upper_hsv, dtype=np.uint8),
            )
            color_mask = cv2.bitwise_or(color_mask, range_mask)
        total_color_ratio = cv2.countNonZero(color_mask) / total_pixels

        column_average_width = int(column_average_width)
        if column_average_width <= 0:
            column_average_width = 1
        column_average_width = min(column_average_width, width)

        color_column_counts = np.count_nonzero(color_mask, axis=0)
        color_column_ratios = color_column_counts.astype(np.float64) / height
        if column_average_width > 1:
            kernel = np.ones(column_average_width, dtype=np.float64)
            kernel /= column_average_width
            smoothed_color_column_ratios = np.convolve(
                color_column_ratios,
                kernel,
                mode="same",
            )
        else:
            smoothed_color_column_ratios = color_column_ratios

        peak_column_ratio = float(np.max(smoothed_color_column_ratios))
        peak_column_indices = np.flatnonzero(
            np.isclose(smoothed_color_column_ratios, peak_column_ratio)
        )
        peak_column_index = float(np.mean(peak_column_indices))
        is_color_column_detected = peak_column_ratio >= block_threshold
        if is_color_column_detected:
            peak_column_x = peak_column_index
            peak_center_offset_ratio = ((peak_column_index + 0.5) / width) - 0.5
        else:
            peak_column_x = None
            peak_center_offset_ratio = None

        block_count = 5
        block_width = width // block_count
        color_block_ratios = []
        for index in range(block_count):
            start_x = index * block_width
            end_x = (index + 1) * block_width if index < block_count - 1 else width
            block_mask = color_mask[:, start_x:end_x]
            block_area = block_mask.shape[0] * block_mask.shape[1]
            ratio = cv2.countNonZero(block_mask) / block_area if block_area else 0.0
            color_block_ratios.append(ratio)

        left_far_color_ratio = color_block_ratios[0]
        left_near_color_ratio = color_block_ratios[1]
        center_color_ratio = color_block_ratios[2]
        right_near_color_ratio = color_block_ratios[3]
        right_far_color_ratio = color_block_ratios[4]
        left_color_ratio = max(left_far_color_ratio, left_near_color_ratio)
        right_color_ratio = max(right_near_color_ratio, right_far_color_ratio)

        region_ratios = {
            "left_far": left_far_color_ratio,
            "left": left_near_color_ratio,
            "center": center_color_ratio,
            "right": right_near_color_ratio,
            "right_far": right_far_color_ratio,
        }
        color_direction = max(region_ratios, key=region_ratios.get)
        if region_ratios[color_direction] < block_threshold:
            color_direction = "none"
            color_block_number = None
        else:
            color_block_number = list(region_ratios).index(color_direction) + 1

        is_color_detected = total_color_ratio >= color_threshold
        direction_text = {
            "left_far": "一番左側",
            "left": "左側",
            "center": "正面",
            "right": "右側",
            "right_far": "一番右側",
        }
        if not is_color_detected:
            reason = "指定色は検出されませんでした"
        elif color_direction in direction_text:
            reason = f"指定色は{direction_text[color_direction]}に多く検出されました"
        else:
            reason = "指定色の方向を判定できませんでした"

        return {
            "is_color_detected": bool(is_color_detected),
            "total_color_ratio": float(total_color_ratio),
            "left_color_ratio": float(left_color_ratio),
            "center_color_ratio": float(center_color_ratio),
            "right_color_ratio": float(right_color_ratio),
            "left_far_color_ratio": float(left_far_color_ratio),
            "left_near_color_ratio": float(left_near_color_ratio),
            "right_near_color_ratio": float(right_near_color_ratio),
            "right_far_color_ratio": float(right_far_color_ratio),
            "color_block_ratios": [float(ratio) for ratio in color_block_ratios],
            "is_color_left": bool(left_color_ratio >= color_threshold),
            "is_color_center": bool(center_color_ratio >= color_threshold),
            "is_color_right": bool(right_color_ratio >= color_threshold),
            "is_color_left_far": bool(left_far_color_ratio >= color_threshold),
            "is_color_left_near": bool(left_near_color_ratio >= color_threshold),
            "is_color_right_near": bool(right_near_color_ratio >= color_threshold),
            "is_color_right_far": bool(right_far_color_ratio >= color_threshold),
            "is_color_in_front": bool(center_color_ratio >= color_threshold),
            "color_direction": color_direction,
            "color_block_number": color_block_number,
            "color_peak_column_x": (
                None
                if peak_column_x is None
                else float(peak_column_x)
            ),
            "color_peak_column_ratio": float(peak_column_ratio),
            "color_peak_center_offset_ratio": (
                None
                if peak_center_offset_ratio is None
                else float(peak_center_offset_ratio)
            ),
            "color_column_average_width": int(column_average_width),
            "is_color_column_detected": bool(is_color_column_detected),
            "color_column_ratios": [
                float(ratio) for ratio in color_column_ratios.tolist()
            ],
            "smoothed_color_column_ratios": [
                float(ratio) for ratio in smoothed_color_column_ratios.tolist()
            ],
            "center_start_x": int(block_width * 2),
            "center_end_x": int(block_width * 3),
            "color_mask": color_mask,
            "reason": reason,
        }

    
    def judge_red_goal_reached(
        self,
        image,
        red_threshold=0.15,
        goal_center_threshold=0.90,
        goal_total_threshold=0.90,
        center_width_ratio=0.4
    ):
        """
        赤色パイロンをゴールとして検出し、ゴールしたかを判定する。

        判定条件:
            1. 5分割した画像の中央ブロックの赤色割合がしきい値以上

        Parameters
        ----------
        image : numpy.ndarray
            OpenCVで読み込んだ画像データ

        red_threshold : float
            赤色検出の基本しきい値
            例: 0.15 = 15%

        goal_center_threshold : float
            5分割した画像の中央ブロックにおける赤色割合のゴール判定しきい値
            例: 0.90 = 中央ブロックの90%以上が赤ならゴール

        goal_total_threshold : float
            互換性のため残している引数。現在のゴール判定には使用しない。

        center_width_ratio : float
            画像中央をどれくらいの幅で見るか
            例: 0.4 = 画像中央40%を正面領域とする

        Returns
        -------
        result : dict
            ゴール判定結果
        """

        color_result = self.detect_color(
            image=image,
            hsv_ranges=self.RED_HSV_RANGES,
            color_threshold=red_threshold,
        )

        total_red_ratio = color_result["total_color_ratio"]
        center_block_red_ratio = color_result["color_block_ratios"][2]
        red_direction = color_result["color_direction"]

        # ゴールが正面にあるか
        is_goal_in_front = (
            red_direction == "center"
            or color_result["is_color_center"]
        )

        # 5分割の中央ブロックに十分な赤色があるか
        is_center_large_enough = center_block_red_ratio >= goal_center_threshold

        # 画像全体としても十分な赤色があるか
        is_total_large_enough = total_red_ratio >= goal_total_threshold

        # 最終的なゴール判定
        goal_reached = is_center_large_enough

        if goal_reached:
            reason = "中央ブロックの赤色割合がしきい値以上のため、ゴールしたと判定します"
        else:
            reason = "中央ブロックの赤色割合が小さいため、ゴールとは判定できません"

        result = color_result.copy()

        result["goal_reached"] = bool(goal_reached)
        result["is_goal_in_front"] = bool(is_goal_in_front)
        result["is_center_large_enough"] = bool(is_center_large_enough)
        result["is_total_large_enough"] = bool(is_total_large_enough)

        result["center_block_color_ratio"] = float(center_block_red_ratio)
        result["goal_center_threshold"] = float(goal_center_threshold)
        result["goal_total_threshold"] = float(goal_total_threshold)

        result["goal_reason"] = reason

        return result

        
    def detect_single_aruco_marker_for_capture_check(
        self,
        image,
        target_center_x=1135,
        target_center_y=1220,
        position_tolerance_x=160,
        position_tolerance_y=120,
        min_area_ratio=0.0015,
        max_area_ratio=0.10
    ):
        """
        画像からArUcoマーカーを1つ検出し、
        マーカーの位置と大きさから撮影が正常か判定する。

        使用するArUco辞書:
            cv2.aruco.DICT_4X4_50

        Parameters
        ----------
        image : numpy.ndarray
            OpenCVで読み込んだ画像データ

        target_center_x : float
            想定しているマーカー中心x座標
            例: 画像中心が640x480なら 320

        target_center_y : float
            想定しているマーカー中心y座標
            例: 画像中心が640x480なら 240

        position_tolerance_x : float
            x方向の許容誤差[pixel]
            例: 80なら target_center_x ± 80 px をOK範囲とする

        position_tolerance_y : float
            y方向の許容誤差[pixel]
            例: 60なら target_center_y ± 60 px をOK範囲とする

        min_area_ratio : float
            マーカー面積割合の下限
            例: 0.005 = 画像全体の0.5%以上ならOK

        max_area_ratio : float
            マーカー面積割合の上限
            例: 0.20 = 画像全体の20%以下ならOK

        Returns
        -------
        result : dict
            ArUcoマーカー検出結果と撮影判定結果
        """

        height, width = image.shape[:2]
        image_area = width * height

        result = {
            "is_detected": False,
            "is_capture_ok": False,

            "marker_id": None,

            "center_x": None,
            "center_y": None,

            "target_center_x": target_center_x,
            "target_center_y": target_center_y,

            "center_error_x": None,
            "center_error_y": None,

            "position_tolerance_x": position_tolerance_x,
            "position_tolerance_y": position_tolerance_y,

            "is_position_ok": False,

            "corners": None,

            "tilt_deg": None,

            "marker_area_px": None,
            "marker_area_ratio": None,

            "min_area_ratio": min_area_ratio,
            "max_area_ratio": max_area_ratio,

            "is_area_large_enough": False,
            "is_area_small_enough": False,
            "is_area_ok": False,

            "bbox_x": None,
            "bbox_y": None,
            "bbox_w": None,
            "bbox_h": None,

            "reason": "マーカーが検出されませんでした"
        }

        if image_area == 0:
            result["reason"] = "画像サイズが不正です"
            return result

        # グレースケール化
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # ArUco辞書を指定
        aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        parameters = cv2.aruco.DetectorParameters()

        # OpenCVのバージョン差に対応
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

        # 複数検出された場合は、一番面積が大きいマーカーを採用
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
        top_left = points[0]
        top_right = points[1]

        # 中心座標
        center_x = float(np.mean(points[:, 0]))
        center_y = float(np.mean(points[:, 1]))

        # 想定位置からのズレ
        center_error_x = center_x - target_center_x
        center_error_y = center_y - target_center_y

        # 位置判定
        is_position_ok = (
            abs(center_error_x) <= position_tolerance_x
            and abs(center_error_y) <= position_tolerance_y
        )

        # 画像上の傾き
        dx = top_right[0] - top_left[0]
        dy = top_right[1] - top_left[1]
        tilt_deg = math.degrees(math.atan2(dy, dx))

        # 面積
        marker_area_px = float(cv2.contourArea(points))
        marker_area_ratio = marker_area_px / image_area

        # 面積判定
        is_area_large_enough = marker_area_ratio >= min_area_ratio
        is_area_small_enough = marker_area_ratio <= max_area_ratio

        is_area_ok = (
            is_area_large_enough
            and is_area_small_enough
        )

        # 外接矩形
        x, y, w, h = cv2.boundingRect(points.astype(np.float32))

        # 撮影正常判定
        is_capture_ok = (
            is_position_ok
            and is_area_ok
        )

        # 理由作成
        reasons = []

        if not is_position_ok:
            reasons.append("マーカーが想定位置から外れています")

        if not is_area_large_enough:
            reasons.append("マーカーが小さすぎます")

        if not is_area_small_enough:
            reasons.append("マーカーが大きすぎます")

        if is_capture_ok:
            reason = "撮影は正常と判断されます"
        else:
            reason = " / ".join(reasons)

        result = {
            "is_detected": True,
            "is_capture_ok": bool(is_capture_ok),

            "marker_id": marker_id,

            "center_x": center_x,
            "center_y": center_y,

            "target_center_x": float(target_center_x),
            "target_center_y": float(target_center_y),

            "center_error_x": float(center_error_x),
            "center_error_y": float(center_error_y),

            "position_tolerance_x": float(position_tolerance_x),
            "position_tolerance_y": float(position_tolerance_y),

            "is_position_ok": bool(is_position_ok),

            "corners": points,

            "tilt_deg": float(tilt_deg),

            "marker_area_px": marker_area_px,
            "marker_area_ratio": float(marker_area_ratio),

            "min_area_ratio": float(min_area_ratio),
            "max_area_ratio": float(max_area_ratio),

            "is_area_large_enough": bool(is_area_large_enough),
            "is_area_small_enough": bool(is_area_small_enough),
            "is_area_ok": bool(is_area_ok),

            "bbox_x": int(x),
            "bbox_y": int(y),
            "bbox_w": int(w),
            "bbox_h": int(h),

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
        image_center_x = int(result["target_center_x"])
        image_center_y = int(result["target_center_y"])

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
