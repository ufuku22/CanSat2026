import cv2
import numpy as np
from pathlib import Path
import math

"""
赤十字：目標中心座標
黄緑：検出したARマーカー
赤点：検出したマーカーの中心

黄緑：中心の成功領域
黄：マーカーの最小サイズ
青：マーカーの最大サイズ

"""


class Arcro:
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
    
    def flip_horizontal(self,image):
        """
        画像を左右反転する
        """
        
        filipped_image = cv2.flip(image, -1)
        
        return filipped_image
    
    def draw_aruco_capture_check_threshold_area(
        self,
        image,
        target_center_x=1135,
        target_center_y=1220,
        position_tolerance_x=256,
        position_tolerance_y=192,
        min_area_ratio=0.002,
        max_area_ratio=0.20
    ):
        """
        ArUcoマーカー撮影判定で使用するしきい値を画像上に描画する。

        描画内容:
            - 想定中心座標
            - 位置許容範囲
            - 最小面積の目安
            - 最大面積の目安

        Parameters
        ----------
        image : numpy.ndarray
            OpenCVで読み込んだ画像データ

        target_center_x : float
            想定しているマーカー中心x座標

        target_center_y : float
            想定しているマーカー中心y座標

        position_tolerance_x : float
            x方向の許容誤差[pixel]

        position_tolerance_y : float
            y方向の許容誤差[pixel]

        min_area_ratio : float
            マーカー面積割合の下限

        max_area_ratio : float
            マーカー面積割合の上限

        Returns
        -------
        output_image : numpy.ndarray
            しきい値の意味を描画した画像
        """

        if image is None:
            raise ValueError("画像データが不正です")

        output_image = image.copy()

        height, width = output_image.shape[:2]
        image_area = width * height

        target_center_x = int(target_center_x)
        target_center_y = int(target_center_y)

        # ==============================
        # 1. 位置許容範囲を描画
        # ==============================
        left = int(target_center_x - position_tolerance_x)
        right = int(target_center_x + position_tolerance_x)
        top = int(target_center_y - position_tolerance_y)
        bottom = int(target_center_y + position_tolerance_y)

        # 画像外にはみ出さないように補正
        left = max(0, left)
        right = min(width - 1, right)
        top = max(0, top)
        bottom = min(height - 1, bottom)

        # 位置OK範囲: 緑の四角
        cv2.rectangle(
            output_image,
            (left, top),
            (right, bottom),
            (0, 255, 0),
            3
        )

        # ==============================
        # 2. 想定中心点を描画
        # ==============================
        # 中心点: 赤い丸
        cv2.circle(
            output_image,
            (target_center_x, target_center_y),
            10,
            (0, 0, 255),
            -1
        )

        # 中心十字線
        cross_size = 40

        cv2.line(
            output_image,
            (target_center_x - cross_size, target_center_y),
            (target_center_x + cross_size, target_center_y),
            (0, 0, 255),
            3
        )

        cv2.line(
            output_image,
            (target_center_x, target_center_y - cross_size),
            (target_center_x, target_center_y + cross_size),
            (0, 0, 255),
            3
        )

        # ==============================
        # 3. 面積しきい値を正方形で描画
        # ==============================
        # 面積ratioからピクセル面積に変換
        min_area_px = image_area * min_area_ratio
        max_area_px = image_area * max_area_ratio

        # 正方形マーカーと仮定した場合の一辺
        min_side = int(round(math.sqrt(min_area_px)))
        max_side = int(round(math.sqrt(max_area_px)))

        def draw_centered_square(side_length, color, thickness):
            half = side_length // 2

            x1 = target_center_x - half
            y1 = target_center_y - half
            x2 = target_center_x + half
            y2 = target_center_y + half

            # 画像外にはみ出さないように補正
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(width - 1, x2)
            y2 = min(height - 1, y2)

            cv2.rectangle(
                output_image,
                (x1, y1),
                (x2, y2),
                color,
                thickness
            )

        # 最小面積の目安: 黄色
        draw_centered_square(
            min_side,
            (0, 255, 255),
            2
        )

        # 最大面積の目安: 青
        draw_centered_square(
            max_side,
            (255, 0, 0),
            2
        )

        # ==============================
        # 4. 説明テキストを描画
        # ==============================
        text_lines = [
            "Target center: red cross",
            "Position OK area: green rectangle",
            f"Target center = ({target_center_x}, {target_center_y})",
            f"Tolerance X = +/- {position_tolerance_x}px",
            f"Tolerance Y = +/- {position_tolerance_y}px",
            f"Min area ratio = {min_area_ratio}",
            f"Max area ratio = {max_area_ratio}",
            f"Min square side ~= {min_side}px",
            f"Max square side ~= {max_side}px"
        ]

        x0 = 30
        y0 = 50
        line_height = 34

        for i, text in enumerate(text_lines):
            cv2.putText(
                output_image,
                text,
                (x0, y0 + i * line_height),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),  # 赤色
                2
            )

        return output_image
    

    
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
        
    def detect_single_aruco_marker_for_capture_check(
        self,
        image,
        target_center_x=1135,
        target_center_y=1220,
        position_tolerance_x=256,
        position_tolerance_y=192,
        min_area_ratio=0.002,
        max_area_ratio=0.20,
        tilt_tolerance_deg=15.0
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
            "tilt_tolerance_deg": tilt_tolerance_deg,
            "is_tilt_ok": False,

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
        marker_id = int(np.array(ids).flatten()[largest_index])
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
        
        # 傾き判定
        is_tilt_ok = abs(tilt_deg) <= tilt_tolerance_deg

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
            and is_tilt_ok
        )

        # 理由作成
        reasons = []

        if not is_position_ok:
            reasons.append("マーカーが想定位置から外れています")

        if not is_area_large_enough:
            reasons.append("マーカーが小さすぎます")

        if not is_area_small_enough:
            reasons.append("マーカーが大きすぎます")
            
        if not is_tilt_ok:
            reasons.append("マーカーの傾きが大きすぎます")

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
            "tilt_tolerance_deg": float(tilt_tolerance_deg),
            "is_tilt_ok": bool(is_tilt_ok),

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

        x0 = 600
        y0 = 40
        line_height = 30
        
        # 画像外にはみ出さないようにする
        x0 = max(30, x0)


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
