#!/usr/bin/env python3
"""前進開始からPD減速停止後まで、線形加速度とジャークを連続表示する。"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import threading
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import DriveControllerConfig, StuckAvoidanceConfig
from drive_controller import DriveController
from navigation_controller import NavigationController
from sensor_manager import SensorManager


SAMPLE_INTERVAL_S = 0.02
MOTOR_OUTPUT_PERCENT = 100.0
FORWARD_DURATION_S = 2.0
PRE_START_LOG_DURATION_S = 0.2
POST_STOP_LOG_DURATION_S = 0.5


def wait_until(deadline: float, logger_failed: threading.Event) -> None:
    while True:
        if logger_failed.is_set():
            raise RuntimeError("加速度ログの取得に失敗しました")
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return
        time.sleep(min(remaining, SAMPLE_INTERVAL_S))


def log_acceleration_and_jerk(
    sensors: SensorManager,
    phase: dict[str, str],
    stop_logging: threading.Event,
    logger_failed: threading.Event,
    log_started_at: float,
) -> None:
    axis = str(StuckAvoidanceConfig.SENSOR_FORWARD_AXIS).lower()
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    axis_sign = float(StuckAvoidanceConfig.SENSOR_FORWARD_SIGN)
    previous_time: float | None = None
    previous_forward_accel: float | None = None
    next_sample_at = log_started_at

    print(
        "elapsed_s,phase,dt_s,"
        "linear_accel_x_mps2,linear_accel_y_mps2,linear_accel_z_mps2,"
        "forward_jerk_mps3",
        flush=True,
    )

    try:
        while not stop_logging.is_set():
            now = time.monotonic()
            if now < next_sample_at:
                stop_logging.wait(next_sample_at - now)
                if stop_logging.is_set():
                    break

            sample_time = time.monotonic()
            accel = tuple(
                float(value) for value in sensors.get_linear_acceleration()
            )
            forward_accel = accel[axis_index] * axis_sign

            if previous_time is None or previous_forward_accel is None:
                sample_interval = math.nan
                jerk = math.nan
            else:
                sample_interval = sample_time - previous_time
                jerk = (
                    (forward_accel - previous_forward_accel) / sample_interval
                    if sample_interval > 0.0
                    else math.nan
                )

            print(
                f"{sample_time - log_started_at:.6f},"
                f"{phase['value']},"
                f"{sample_interval:.6f},"
                f"{accel[0]:+.3f},{accel[1]:+.3f},{accel[2]:+.3f},"
                f"{jerk:+.3f}",
                flush=True,
            )

            previous_time = sample_time
            previous_forward_accel = forward_accel
            next_sample_at += SAMPLE_INTERVAL_S
            if next_sample_at < time.monotonic():
                next_sample_at = time.monotonic()
    except Exception as exc:
        logger_failed.set()
        print(f"LOGGER_ERROR,{type(exc).__name__},{exc}", flush=True)


def main() -> int:
    speed = MOTOR_OUTPUT_PERCENT
    print(
        f"前進{FORWARD_DURATION_S:g}秒後にPD減速停止します。"
        f"出力={speed:g}%, サンプリング間隔={SAMPLE_INTERVAL_S:g}秒"
    )
    input("安全を確認し、準備できたらEnterを押してください")

    driver: DriveController | None = None
    sensors: SensorManager | None = None
    logger_thread: threading.Thread | None = None
    stop_logging = threading.Event()
    logger_failed = threading.Event()
    phase = {"value": "PRE_START"}

    try:
        driver = DriveController()
        sensors = SensorManager()
        sensors.imu.setup()
        navigator = NavigationController()
        target_heading = float(sensors.get_heading_deg())

        log_started_at = time.monotonic()
        logger_thread = threading.Thread(
            target=log_acceleration_and_jerk,
            args=(
                sensors,
                phase,
                stop_logging,
                logger_failed,
                log_started_at,
            ),
            name="brake-jerk-logger",
        )
        logger_thread.start()

        wait_until(
            log_started_at + PRE_START_LOG_DURATION_S,
            logger_failed,
        )

        phase["value"] = "FORWARD"
        forward_started_at = time.monotonic()
        stop_at = forward_started_at + FORWARD_DURATION_S
        driver.drive(speed)
        wait_until(stop_at, logger_failed)

        phase["value"] = "PD_RAMP_STOP"
        ramp_started_at = time.monotonic()
        navigator._pd_ramp_stop_forward(
            driver,
            sensors,
            speed,
            speed,
            target_heading=target_heading,
            prev_error=0.0,
            steps=DriveControllerConfig.RAMP_STOP_STEPS,
            interval=DriveControllerConfig.RAMP_STOP_INTERVAL_S,
        )
        stopped_at = time.monotonic()
        wait_until(
            stopped_at + POST_STOP_LOG_DURATION_S,
            logger_failed,
        )

        phase["value"] = "FINISHED"
        print(
            f"テスト終了: PD減速開始="
            f"{ramp_started_at - log_started_at:.6f}秒, "
            f"停止完了={stopped_at - log_started_at:.6f}秒",
            flush=True,
        )
        return 0
    except KeyboardInterrupt:
        if driver is not None:
            driver.stop()
        print("\nテストを中断してモーター出力を停止しました", flush=True)
        return 130
    finally:
        stop_logging.set()
        if logger_thread is not None:
            logger_thread.join(timeout=1.0)
        if driver is not None:
            driver.cleanup()
        if sensors is not None:
            sensors.close()


if __name__ == "__main__":
    raise SystemExit(main())
