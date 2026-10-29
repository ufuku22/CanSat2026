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

    def __init__(self, logger=None):
        self.logger = logger

    def _log(self, message):
        if self.logger is not None:
            self.logger.console(message)
        else:
            print(message, flush=True)

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

    def select_best_selfie_image(self, image_paths, **judge_options):
        """複数の自撮り画像からPCへ送信する1枚を選ぶ。"""
        from image_judge import ImageJudge

        return ImageJudge(self, **judge_options).select_best_image(image_paths)
    
    def flip_horizontal(self, image):
        """
        画像を左右反転する
        """
        
        filipped_image = cv2.flip(image, 1)
        
        return filipped_image

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
            self._log(
                "Image compression attempt "
                f"{attempt}: {new_width}x{new_height}, "
                f"quality={current_quality}, size={size} bytes"
            )

            if size <= max_bytes:
                if size > target_bytes:
                    self._log(f"Compressed image is within limit and near target: {size} bytes")
                self._log(f"圧縮画像を保存しました: {output_path}")
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
        column_threshold=None,
        column_average_width=15,
    ):
        """指定された複数のHSV範囲に含まれる色の量と方向を返す。"""
        height, width = image.shape[:2]
        total_pixels = height * width

        if total_pixels == 0:
            return {
                "is_color_detected": False,
                "total_color_ratio": 0.0,
                "color_peak_column_x": None,
                "color_peak_center_offset_ratio": None,
                "color_peak_columns": [],
                "color_peak_count": 0,
                "color_mask": None,
                "image_width": int(width),
                "image_height": int(height),
                "reason": "画像サイズが不正です",
            }

        if column_threshold is None:
            column_threshold = color_threshold

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

        peak_columns = []
        above_column_threshold = (
            smoothed_color_column_ratios >= column_threshold
        )
        segment_start = None
        for column_index, is_above_threshold in enumerate(
            above_column_threshold
        ):
            if is_above_threshold and segment_start is None:
                segment_start = column_index
            if (
                segment_start is not None
                and (
                    not is_above_threshold
                    or column_index == width - 1
                )
            ):
                segment_end = (
                    column_index + 1
                    if is_above_threshold and column_index == width - 1
                    else column_index
                )
                segment_ratios = smoothed_color_column_ratios[
                    segment_start:segment_end
                ]
                segment_peak_ratio = float(np.max(segment_ratios))
                segment_peak_indices = np.flatnonzero(
                    np.isclose(segment_ratios, segment_peak_ratio)
                ) + segment_start
                segment_peak_index = float(np.mean(segment_peak_indices))
                peak_columns.append({
                    "x": segment_peak_index,
                    "center_offset_ratio": (
                        (segment_peak_index + 0.5) / width
                    ) - 0.5,
                    "column_ratio": segment_peak_ratio,
                    "start_x": float(segment_start),
                    "end_x": float(segment_end - 1),
                })
                segment_start = None

        peak_column_ratio = float(np.max(smoothed_color_column_ratios))
        peak_column_indices = np.flatnonzero(
            np.isclose(smoothed_color_column_ratios, peak_column_ratio)
        )
        peak_column_index = float(np.mean(peak_column_indices))
        is_color_column_detected = peak_column_ratio >= column_threshold
        if is_color_column_detected:
            peak_column_x = peak_column_index
            peak_center_offset_ratio = ((peak_column_index + 0.5) / width) - 0.5
        else:
            peak_column_x = None
            peak_center_offset_ratio = None

        is_color_detected = total_color_ratio >= color_threshold
        if not is_color_detected:
            reason = "指定色は検出されませんでした"
        elif peak_column_x is not None:
            reason = f"指定色はx={peak_column_x:.1f}の列に多く検出されました"
        else:
            reason = "指定色の方向を判定できませんでした"

        return {
            "is_color_detected": bool(is_color_detected),
            "total_color_ratio": float(total_color_ratio),
            "color_peak_column_x": (
                None
                if peak_column_x is None
                else float(peak_column_x)
            ),
            "color_peak_center_offset_ratio": (
                None
                if peak_center_offset_ratio is None
                else float(peak_center_offset_ratio)
            ),
            "color_peak_columns": peak_columns,
            "color_peak_count": len(peak_columns),
            "color_mask": color_mask,
            "image_width": int(width),
            "image_height": int(height),
            "reason": reason,
        }

    def detect_red_ball_candidates(
        self,
        image,
        *,
        min_area_ratio=0.00025,
        min_center_y_ratio=0.35,
        max_center_y_ratio=0.80,
        peak_expand_px=20,
        color_result=None,
    ):
        """列ピークごとの見かけサイズから赤ボール候補を返す。

        Raspberry Pi Zeroでも使いやすいよう、距離変換やwatershedは使わず、
        detect_color()が作った赤マスクと列ピークセグメントを再利用する。
        """
        height, width = image.shape[:2]
        total_pixels = height * width
        if total_pixels == 0:
            return []

        if color_result is None:
            color_result = self.detect_color(
                image,
                hsv_ranges=self.RED_HSV_RANGES,
                color_threshold=0.0,
                column_threshold=0.005,
                column_average_width=31,
            )
        color_mask = color_result.get("color_mask")
        if color_mask is None:
            return []

        y_min = int(np.clip(min_center_y_ratio, 0.0, 1.0) * height)
        y_max = int(np.clip(max_center_y_ratio, 0.0, 1.0) * height)
        min_area_px = float(min_area_ratio) * total_pixels
        peak_expand_px = max(0, int(peak_expand_px))
        candidates = []
        for peak in color_result.get("color_peak_columns", []):
            peak_x = float(peak.get("x", 0.0))
            start_x = int(round(peak.get("start_x", peak_x))) - peak_expand_px
            end_x = int(round(peak.get("end_x", peak_x))) + peak_expand_px + 1
            start_x = max(0, start_x)
            end_x = min(width, end_x)
            if start_x >= end_x:
                continue

            roi = color_mask[y_min:y_max, start_x:end_x]
            # 離れた赤領域を1つの巨大なbboxへまとめない。
            component_count, _, stats, _ = cv2.connectedComponentsWithStats(
                roi,
                connectivity=8,
            )
            if component_count <= 1:
                continue

            component_index = 1 + int(
                np.argmax(stats[1:, cv2.CC_STAT_AREA])
            )
            area = float(stats[component_index, cv2.CC_STAT_AREA])
            if area < min_area_px:
                continue

            x = int(stats[component_index, cv2.CC_STAT_LEFT])
            y = int(stats[component_index, cv2.CC_STAT_TOP])
            w = int(stats[component_index, cv2.CC_STAT_WIDTH])
            h = int(stats[component_index, cv2.CC_STAT_HEIGHT])
            x += start_x
            y += y_min
            if w <= 0 or h <= 0:
                continue

            center_x = x + w / 2.0
            center_y = y + h / 2.0
            center_y_ratio = center_y / height
            if (
                center_y_ratio < min_center_y_ratio
                or center_y_ratio > max_center_y_ratio
            ):
                continue

            aspect_ratio = w / h
            if aspect_ratio < 0.25 or aspect_ratio > 3.50:
                continue

            fill_ratio = area / float(w * h)
            visible_diameter_px = max(float(w), float(h))
            score = visible_diameter_px * visible_diameter_px * max(
                0.2,
                min(fill_ratio, 1.0),
            )
            candidates.append({
                "x": float(center_x),
                "y": float(center_y),
                "center_offset_ratio": ((center_x + 0.5) / width) - 0.5,
                "area_px": area,
                "area_ratio": area / total_pixels,
                "bbox": {
                    "x": float(x),
                    "y": float(y),
                    "width": float(w),
                    "height": float(h),
                },
                "aspect_ratio": float(aspect_ratio),
                "fill_ratio": float(fill_ratio),
                "visible_diameter_px": visible_diameter_px,
                "score": float(score),
                "source_peak": peak.copy(),
            })

        deduped = []
        for candidate in sorted(
            candidates,
            key=lambda item: item["score"],
            reverse=True,
        ):
            if any(
                abs(candidate["x"] - kept["x"]) < 20.0
                and abs(candidate["y"] - kept["y"]) < 20.0
                for kept in deduped
            ):
                continue
            deduped.append(candidate)

        return deduped

    def detect_red_ball_circle_candidates(
        self,
        image,
        *,
        hsv_ranges=None,
        scale=0.5,
        min_red_fill_ratio=0.70,
        min_score=1000.0,
        min_score_ratio_to_best=0.18,
        contained_center_ratio=0.75,
        contained_radius_ratio=0.70,
    ):
        """縮小画像の全域でHough円検出し、赤ボール中心候補を返す。"""
        height, width = image.shape[:2]
        if height * width == 0:
            return []

        scale = float(scale)
        if scale <= 0.0 or scale > 1.0:
            scale = 0.5
        small_width = max(1, int(round(width * scale)))
        small_height = max(1, int(round(height * scale)))
        small = cv2.resize(
            image,
            (small_width, small_height),
            interpolation=cv2.INTER_AREA,
        )

        hsv_ranges = hsv_ranges or self.RED_HSV_RANGES
        hsv_image = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        red_mask = np.zeros((small_height, small_width), dtype=np.uint8)
        for lower_hsv, upper_hsv in hsv_ranges:
            range_mask = cv2.inRange(
                hsv_image,
                np.array(lower_hsv, dtype=np.uint8),
                np.array(upper_hsv, dtype=np.uint8),
            )
            red_mask = cv2.bitwise_or(red_mask, range_mask)

        boundary_red_mask = cv2.medianBlur(red_mask, 5)
        red_mask = cv2.medianBlur(red_mask, 5)

        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        gray = cv2.bitwise_and(gray, gray, mask=red_mask)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        min_radius = max(8, int(round(min(width, height) * scale * 0.025)))
        max_radius = max(min_radius + 1, int(round(min(width, height) * scale * 0.18)))
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=max(20, int(round(width * scale * 0.06))),
            param1=80,
            param2=12,
            minRadius=min_radius,
            maxRadius=max_radius,
        )
        if circles is None:
            return []

        candidates = []
        boundary_angles = np.linspace(
            0.0,
            2.0 * math.pi,
            72,
            endpoint=False,
        )
        boundary_cos = np.cos(boundary_angles)
        boundary_sin = np.sin(boundary_angles)
        for small_x, small_y, small_radius in np.round(circles[0]).astype(int):
            circle_mask = np.zeros((small_height, small_width), dtype=np.uint8)
            cv2.circle(
                circle_mask,
                (int(small_x), int(small_y)),
                int(small_radius),
                255,
                -1,
            )
            red_inside = cv2.countNonZero(cv2.bitwise_and(red_mask, circle_mask))
            circle_area = math.pi * float(small_radius) * float(small_radius)
            red_fill_ratio = red_inside / circle_area if circle_area else 0.0
            if red_fill_ratio < min_red_fill_ratio:
                continue

            inner_x = np.rint(
                small_x + boundary_cos * small_radius * 0.85
            ).astype(int)
            inner_y = np.rint(
                small_y + boundary_sin * small_radius * 0.85
            ).astype(int)
            outer_x = np.rint(
                small_x + boundary_cos * small_radius * 1.10
            ).astype(int)
            outer_y = np.rint(
                small_y + boundary_sin * small_radius * 1.10
            ).astype(int)
            valid_boundary = (
                (inner_x >= 0)
                & (inner_x < small_width)
                & (inner_y >= 0)
                & (inner_y < small_height)
                & (outer_x >= 0)
                & (outer_x < small_width)
                & (outer_y >= 0)
                & (outer_y < small_height)
            )
            boundary_support = np.zeros(
                len(boundary_angles),
                dtype=bool,
            )
            boundary_support[valid_boundary] = (
                (
                    boundary_red_mask[
                        inner_y[valid_boundary],
                        inner_x[valid_boundary],
                    ]
                    > 0
                )
                & (
                    boundary_red_mask[
                        outer_y[valid_boundary],
                        outer_x[valid_boundary],
                    ]
                    == 0
                )
            )
            circle_boundary_confidence = float(
                np.mean(boundary_support)
            )

            center_x = float(small_x) / scale
            center_y = float(small_y) / scale
            radius = float(small_radius) / scale
            score = (
                radius
                * radius
                * min(red_fill_ratio, 1.0)
                * circle_boundary_confidence
            )
            if score < float(min_score):
                continue
            candidates.append({
                "x": center_x,
                "y": center_y,
                "center_offset_ratio": ((center_x + 0.5) / width) - 0.5,
                "radius_px": radius,
                "visible_diameter_px": radius * 2.0,
                "red_fill_ratio": float(red_fill_ratio),
                "circle_boundary_confidence": (
                    circle_boundary_confidence
                ),
                "score": float(score),
                "bbox": {
                    "x": center_x - radius,
                    "y": center_y - radius,
                    "width": radius * 2.0,
                    "height": radius * 2.0,
                },
            })

        if candidates:
            best_score = max(float(candidate["score"]) for candidate in candidates)
            score_threshold = max(
                float(min_score),
                best_score * float(min_score_ratio_to_best),
            )
            candidates = [
                candidate
                for candidate in candidates
                if float(candidate["score"]) >= score_threshold
            ]

        deduped = []
        for candidate in sorted(
            candidates,
            key=lambda item: item["score"],
            reverse=True,
        ):
            is_contained = False
            for kept in deduped:
                center_distance = math.hypot(
                    float(candidate["x"]) - float(kept["x"]),
                    float(candidate["y"]) - float(kept["y"]),
                )
                if (
                    center_distance
                    <= float(kept["radius_px"]) * float(contained_center_ratio)
                    and float(candidate["radius_px"])
                    <= float(kept["radius_px"]) * float(contained_radius_ratio)
                ):
                    is_contained = True
                    break
            if is_contained:
                continue
            if any(
                abs(candidate["x"] - kept["x"]) < 25.0
                and abs(candidate["y"] - kept["y"]) < 25.0
                for kept in deduped
            ):
                continue
            deduped.append(candidate)

        return deduped

    def judge_red_goal_reached(
        self,
        image,
        red_threshold=0.15,
        goal_angle_red_threshold=0.90,
        horizontal_fov_deg=66.0,
        goal_angle_min_deg=-6.6,
        goal_angle_max_deg=6.6,
    ):
        """
        赤色パイロンをゴールとして検出し、ゴールしたかを判定する。

        判定条件:
            1. 指定した水平角度範囲の赤色割合がしきい値以上

        Parameters
        ----------
        image : numpy.ndarray
            OpenCVで読み込んだ画像データ

        red_threshold : float
            赤色検出の基本しきい値
            例: 0.15 = 15%

        goal_angle_red_threshold : float
            指定した水平角度範囲における赤色割合のゴール判定しきい値
            例: 0.90 = 指定角度範囲の90%以上が赤ならゴール

        horizontal_fov_deg : float
            カメラの水平視野角

        goal_angle_min_deg, goal_angle_max_deg : float
            ゴール判定に使う水平角度範囲

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

        min_angle_deg = min(float(goal_angle_min_deg), float(goal_angle_max_deg))
        max_angle_deg = max(float(goal_angle_min_deg), float(goal_angle_max_deg))
        height, width = color_result["color_mask"].shape[:2]
        half_fov_deg = float(horizontal_fov_deg) / 2.0
        start_ratio = (min_angle_deg + half_fov_deg) / float(horizontal_fov_deg)
        end_ratio = (max_angle_deg + half_fov_deg) / float(horizontal_fov_deg)
        start_x = int(np.floor(np.clip(start_ratio, 0.0, 1.0) * width))
        end_x = int(np.ceil(np.clip(end_ratio, 0.0, 1.0) * width))
        end_x = max(start_x + 1, min(width, end_x))

        angle_mask = color_result["color_mask"][:, start_x:end_x]
        angle_area = angle_mask.shape[0] * angle_mask.shape[1]
        angle_red_ratio = (
            cv2.countNonZero(angle_mask) / angle_area
            if height * width and angle_area
            else 0.0
        )

        # 最終的なゴール判定
        goal_reached = angle_red_ratio >= goal_angle_red_threshold

        if goal_reached:
            reason = "指定角度範囲の赤色割合がしきい値以上のため、ゴールしたと判定します"
        else:
            reason = "指定角度範囲の赤色割合が小さいため、ゴールとは判定できません"

        result = color_result.copy()

        result["goal_reached"] = bool(goal_reached)
        result["goal_angle_color_ratio"] = float(angle_red_ratio)
        result["goal_angle_red_threshold"] = float(goal_angle_red_threshold)
        result["horizontal_fov_deg"] = float(horizontal_fov_deg)
        result["goal_angle_min_deg"] = float(min_angle_deg)
        result["goal_angle_max_deg"] = float(max_angle_deg)

        result["goal_reason"] = reason

        return result

        
    def detect_largest_aruco_marker(self, image):
        """
        画像から最大のArUcoマーカーを1つ検出する。

        使用するArUco辞書:
            cv2.aruco.DICT_4X4_50

        Parameters
        ----------
        image : numpy.ndarray
            OpenCVで読み込んだ画像データ

        Returns
        -------
        result : dict
            ArUcoマーカー検出結果
        """

        height, width = image.shape[:2]
        image_area = width * height

        result = {
            "is_detected": False,
            "marker_id": None,
            "center_x": None,
            "center_y": None,
            "corners": None,
            "tilt_deg": None,
            "marker_area_px": None,
            "marker_area_ratio": None,
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

        # OpenCVのバージョン差に対応
        if hasattr(cv2.aruco, "ArucoDetector"):
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, rejected = detector.detectMarkers(gray)
        else:
            # 旧OpenCVではコンストラクタを直接呼ぶと、環境によって
            # detectMarkers内でsegmentation faultになるため専用factoryを使う。
            parameters = cv2.aruco.DetectorParameters_create()
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

        # 画像上の傾き
        dx = top_right[0] - top_left[0]
        dy = top_right[1] - top_left[1]
        tilt_deg = math.degrees(math.atan2(dy, dx))

        # 面積
        marker_area_px = float(cv2.contourArea(points))
        marker_area_ratio = marker_area_px / image_area

        # 外接矩形
        x, y, w, h = cv2.boundingRect(points.astype(np.float32))

        result = {
            "is_detected": True,
            "marker_id": marker_id,
            "center_x": center_x,
            "center_y": center_y,
            "corners": points,
            "tilt_deg": float(tilt_deg),
            "marker_area_px": marker_area_px,
            "marker_area_ratio": float(marker_area_ratio),
            "bbox_x": int(x),
            "bbox_y": int(y),
            "bbox_w": int(w),
            "bbox_h": int(h),

            "reason": "ARマーカーを検出しました"
        }

        return result

    def draw_aruco_detection_result(self, image, result):
        """ArUcoマーカーの検出結果を画像に描画する。"""

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

        # 情報を文字で描画
        text_lines = [
            f"ID: {result['marker_id']}",
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
