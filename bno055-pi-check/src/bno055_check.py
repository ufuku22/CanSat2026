#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Raspberry Pi Zero WH + AE-BNO055-BO 動作確認用コード

取得する値:
- 温度
- オイラー角 heading, roll, pitch
- クォータニオン
- 加速度
- ジャイロ
- 磁気
- 線形加速度
- 重力
- キャリブレーション状態

終了:
Ctrl + C
"""

import argparse
import time
from datetime import datetime

import board
import adafruit_bno055


def fmt_tuple(value, digits=3):
    """None または tuple を見やすく整形する"""
    if value is None:
        return "None"
    return "(" + ", ".join(f"{v:.{digits}f}" if isinstance(v, float) else str(v) for v in value) + ")"


def main():
    parser = argparse.ArgumentParser(description="AE-BNO055-BO / BNO055 動作確認")
    parser.add_argument(
        "--address",
        type=lambda x: int(x, 0),
        default=0x28,
        help="I2C address. default: 0x28",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="読み取り間隔 秒. default: 1.0",
    )
    args = parser.parse_args()

    print("=== AE-BNO055-BO / BNO055 動作確認 ===")
    print(f"I2C address : 0x{args.address:02X}")
    print(f"interval    : {args.interval} sec")
    print("Ctrl + C で終了します")
    print()

    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c, address=args.address)
    except Exception as e:
        print("BNO055 の初期化に失敗しました。")
        print("確認してください:")
        print("  1. 配線 VIN/GND/SDA/SCL")
        print("  2. Raspberry Pi の I2C が有効か")
        print("  3. i2cdetect で 0x28 が見えるか")
        print("  4. requirements.txt のライブラリをインストール済みか")
        print()
        print(f"error: {e}")
        return

    print("初期化成功")
    print("キャリブレーション状態は 0〜3 で、3 が最良です。")
    print("表示順: system, gyro, accel, mag")
    print()

    while True:
        try:
            now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            temperature = sensor.temperature
            euler = sensor.euler
            quaternion = sensor.quaternion
            acceleration = sensor.acceleration
            gyro = sensor.gyro
            magnetic = sensor.magnetic
            linear_acceleration = sensor.linear_acceleration
            gravity = sensor.gravity
            calibration = sensor.calibration_status

            print(f"[{now}]")
            print(f"  temperature         : {temperature} °C")
            print(f"  euler heading/roll/pitch [deg] : {fmt_tuple(euler)}")
            print(f"  quaternion          : {fmt_tuple(quaternion, digits=5)}")
            print(f"  acceleration [m/s^2]: {fmt_tuple(acceleration)}")
            print(f"  gyro [rad/s]        : {fmt_tuple(gyro)}")
            print(f"  magnetic [uT]       : {fmt_tuple(magnetic)}")
            print(f"  linear accel [m/s^2]: {fmt_tuple(linear_acceleration)}")
            print(f"  gravity [m/s^2]     : {fmt_tuple(gravity)}")
            print(f"  calibration sys/gyro/accel/mag : {calibration}")
            print("-" * 60)

            time.sleep(args.interval)

        except KeyboardInterrupt:
            print("\n終了します。")
            break
        except Exception as e:
            print(f"読み取りエラー: {e}")
            time.sleep(args.interval)


if __name__ == "__main__":
    main()