"""
turn_test.py

CanSat の右旋回・左旋回を確認するテストプログラムです。
"""

import time

from drive_controller import DriveController


# ===== テスト条件 =====
TURN_SPEED = 60       # 旋回速度 [%]
TURN_TIME_S = 2.0     # 旋回時間 [秒]
STOP_TIME_S = 1.0     # 停止時間 [秒]


def main():
    driver = DriveController()

    try:
        print("=== 旋回テスト開始 ===")

        # 右旋回
        print("右旋回します")
        driver.turn_right(TURN_SPEED)
        time.sleep(TURN_TIME_S)

        driver.stop()
        time.sleep(STOP_TIME_S)

        # 左旋回
        print("左旋回します")
        driver.turn_left(TURN_SPEED)
        time.sleep(TURN_TIME_S)

        driver.stop()
        time.sleep(STOP_TIME_S)

        print("=== 旋回テスト終了 ===")

    except KeyboardInterrupt:
        print("\n旋回テストを中断しました")

    finally:
        driver.cleanup()


if __name__ == "__main__":
    main()