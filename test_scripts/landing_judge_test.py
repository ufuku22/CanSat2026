#!/usr/bin/env python3
"""実機センサで着地判定だけを実行する簡易テスト。"""

from pathlib import Path
import sys
import time


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import LandingJudgeConfig
from judge import judge_landing
from logger import Logger
from sensor_manager import SensorManager


def main() -> None:
    logger = Logger(log_to_file=False)
    print(
        "着地判定条件: "
        f"加速度={LandingJudgeConfig.TARGET_ACCEL_MPS2:.1f}"
        f"±{LandingJudgeConfig.TOLERANCE_MPS2:.1f} m/s^2, "
        f"連続時間={LandingJudgeConfig.CONTINUOUS_DURATION_S:.1f}秒, "
        "気圧中央値差="
        f"{LandingJudgeConfig.PRESSURE_CHANGE_TOLERANCE_HPA:.1f} hPa以下"
    )
    print("BME280とBNO055を初期化します。Ctrl+Cで終了できます。")

    try:
        with SensorManager() as sensors:
            # 着地判定に必要な2センサだけを初期化する。
            sensors.environment.setup()
            sensors.imu.setup()
            print("着地判定を開始します。機体を静止させてください。")
            started_at = time.monotonic()
            landed = judge_landing(sensors, logger=logger, timeout_s=None)
            elapsed_s = time.monotonic() - started_at
            print(f"着地判定成功（開始から{elapsed_s:.1f}秒）" if landed else "着地判定失敗")
    except KeyboardInterrupt:
        print("\n着地判定テストを終了しました。")


if __name__ == "__main__":
    main()
