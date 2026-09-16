"""実画像で赤コーン探索を検証する（モーター・カメラは使用しない）。"""
import sys
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
import numpy as np

from config import RedBallConfig, RedConeConfig
from image_processor import ImageProcessor
from navigation_goal import _is_red_cone_detected, guide_to_red_cone, _scan_red_target_360


def detect(image):
    config = RedConeConfig()
    processor = ImageProcessor()
    result = processor.detect_color(
        image, processor.RED_HSV_RANGES, config.RED_THRESHOLD,
        config.RED_COLUMN_THRESHOLD, config.RED_COLUMN_AVERAGE_WIDTH,
    )
    result['is_color_detected'] = _is_red_cone_detected(
        result, config.MIN_RED_COMPONENT_AREA_RATIO,
    )
    return result


if __name__ == '__main__':
    if not sys.argv[1:]:
        raise SystemExit('Usage: python -m test_scripts.red_cone_image_regression_test IMAGE ...')
    for filename in sys.argv[1:]:
        image = cv2.imdecode(np.fromfile(filename, dtype=np.uint8), cv2.IMREAD_COLOR)
        assert image is not None, filename
        result = detect(image)
        assert result['is_color_detected'], filename
        height, width = image.shape[:2]
        x = result['color_peak_column_x']
        assert 0.48 * width < x < 0.56 * width, x
        assert result['total_color_ratio'] < RedBallConfig.SWITCH_RED_RATIO

        # 左右の背景だけでは検出しない（中央の球群を含めない）。
        assert not detect(image[:, :width // 3])['is_color_detected']
        assert not detect(image[:, 2 * width // 3:])['is_color_detected']
        blank = np.zeros_like(image)
        assert not detect(blank)['is_color_detected']
        noise = blank.copy()
        noise[::20, ::20] = (0, 0, 255)
        assert not detect(noise)['is_color_detected']

        navigator, driver, sensors = Mock(), Mock(), Mock()
        navigator.rotate_by_angle.return_value = {'reached': True}
        sensors.capture_front_frame.return_value = image
        logger = Mock()
        # 同じ検出を使うGNSS周辺探索も、その場で対象を発見できる。
        scan = _scan_red_target_360(
            navigator, driver, sensors, ImageProcessor(logger),
            RedConeConfig(), RedConeConfig.RED_THRESHOLD, 60, logger,
        )
        assert scan['red_detected']
        navigator.rotate_by_angle.assert_not_called()

        # 初回画像→前進後画像→接近済み画像で粗誘導からの切替まで通す。
        close = blank.copy()
        close[height // 2:height // 2 + height // 5,
              width // 2:width // 2 + width // 5] = (0, 0, 255)
        sensors.capture_front_frame.side_effect = [image, image, close]
        with patch.object(RedConeConfig, 'MAX_GUIDANCE_STEPS', 2):
            guidance = guide_to_red_cone(
                navigator, driver, sensors,
                stop_red_ratio_threshold=RedBallConfig.SWITCH_RED_RATIO,
                forward_duration_by_red_ratio=RedBallConfig.CONE_FORWARD_DURATION_BY_RED_RATIO,
                logger=logger,
            )
        assert guidance['red_ratio_threshold_reached']
        navigator.pd_forward.assert_called_once()
        navigator.rotate_by_angle.assert_called_once()
        angle = navigator.rotate_by_angle.call_args.args[2]
        assert 0 < angle < 4, angle
        print(f'{Path(filename).name}: PASS, red={result["total_color_ratio"]:.8f}, '
              f'x={x:.2f}, turn={angle:.2f}deg')
