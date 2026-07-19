from sensor_manager import CAMERA_FULL_HD_HEIGHT, CAMERA_FULL_HD_WIDTH, SensorManager
from image_processor import ImageProcessor


def main():
    processor = ImageProcessor()

    with SensorManager() as sensors:
        sensors.setup()

        # 前方カメラで撮影
        image_path = sensors.capture_front_image(
            width=CAMERA_FULL_HD_WIDTH,
            height=CAMERA_FULL_HD_HEIGHT,
            hdr=True,
            timeout_ms=2000,
        )

        print(f"撮影画像: {image_path}")

    # 撮影画像を読み込む
    image = processor.load_image(image_path)

    # 赤色パイロンによるゴール判定
    goal_result = processor.judge_color_goal_reached(
        image=image,
        hsv_ranges=processor.RED_HSV_RANGES,
        color_threshold=0.15,
        goal_center_threshold=0.10,
        goal_total_threshold=0.90,
    )

    print("===== 赤色パイロン ゴール判定 =====")
    print(f"ゴール判定: {goal_result['goal_reached']}")
    print(f"理由: {goal_result['goal_reason']}")

    print(f"全体赤色割合: {goal_result['total_color_ratio'] * 100:.2f} %")
    print(f"左赤色割合: {goal_result['left_color_ratio'] * 100:.2f} %")
    print(f"中央赤色割合: {goal_result['center_color_ratio'] * 100:.2f} %")
    print(f"右赤色割合: {goal_result['right_color_ratio'] * 100:.2f} %")

    print(f"赤色方向: {goal_result['color_direction']}")
    print(f"正面にゴールあり: {goal_result['is_goal_in_front']}")

    if goal_result["goal_reached"]:
        print("ゴールしました")
    else:
        print("まだゴールしていません")


if __name__ == "__main__":
    main()
