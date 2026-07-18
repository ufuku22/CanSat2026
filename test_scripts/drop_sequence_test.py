#!/usr/bin/env python3
"""投下試験用の一連動作を実行するテスト。"""

from collections import deque
from datetime import datetime
import math
from pathlib import Path
from statistics import median
import sys
import threading
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from communication_manager import CommunicationManager
from drive_controller import DriveController
from fusing import fuse_and_kick
from judge import judge_landing
from logger import CsvLogger, Logger
from navigation_controller import NavigationController
from selfie_manager import SelfieManager
from sensor_manager import SensorManager
from test_scripts.pressure_imu_altitude_test import (
    ALTITUDE_CORRECTION_GAIN,
    IMU_INTERVAL_S as ALTITUDE_IMU_INTERVAL_SECONDS,
    IMU_VELOCITY_DECAY_TIME_S as IMU_VELOCITY_DECAY_TIME_SECONDS,
    PRESSURE_INTERVAL_S as ALTITUDE_PRESSURE_INTERVAL_SECONDS,
    PRESSURE_MEDIAN_SAMPLES,
    calibrate as calibrate_altitude,
    configure_bme280_for_logging as configure_bme280_for_altitude,
    input_air_temperature_c,
    relative_altitude_m,
    vertical_acceleration_mps2,
)


# CSV記録と放出判定に使うセンサ値の測定間隔
SENSOR_INTERVAL_SECONDS = 0.1
VELOCITY_CORRECTION_GAIN = 0.0

# 放出判定用の気圧しきい値。投下高度に合わせて試験前に調整する。
RELEASE_ABOVE_THRESHOLD_OFFSETS_HPA = (1, 2)
RELEASE_BELOW_THRESHOLD_OFFSETS_HPA = (2, 1)

# 着地判定後、自動的に溶断を始めるまでの待機時間
LANDING_TO_FUSING_DELAY_SECONDS = 3.0

# 試験終了後のモジュール確認に使う設定
RADIO_TEST_MESSAGE = "DROP_TEST_COMPLETE"
GNSS_READ_WAIT_SECONDS = 1.0


def judge_release_and_send_pressure(
    sensors: SensorManager,
    logger: Logger,
    *,
    ground_pressure_hpa: float,
    above_threshold_offsets_hpa: tuple[float, float],
    below_threshold_offsets_hpa: tuple[float, float],
    measurement_interval_s: float,
) -> bool:
    """放出判定の3つ目のしきい値到達時に、その気圧を無線送信する。"""
    checks = (
        (above_threshold_offsets_hpa[0], "above"),
        (above_threshold_offsets_hpa[1], "above"),
        (below_threshold_offsets_hpa[0], "below"),
        (below_threshold_offsets_hpa[1], "below"),
    )
    logger.event("放出判定開始")

    for check_number, (threshold_offset_hpa, expected_state) in enumerate(
        checks,
        start=1,
    ):
        threshold_pressure_hpa = ground_pressure_hpa - threshold_offset_hpa
        while True:
            pressure_hpa = float(sensors.get_environment()["pressure_hpa"])
            pressure_is_above = pressure_hpa >= threshold_pressure_hpa
            threshold_reached = (
                pressure_is_above
                if expected_state == "above"
                else not pressure_is_above
            )
            if threshold_reached:
                logger.event(
                    f"放出気圧判定 {check_number}/4: {expected_state}, "
                    f"閾値={threshold_pressure_hpa:.2f} hPa, "
                    f"気圧={pressure_hpa:.2f} hPa"
                )
                if check_number == 3:
                    try:
                        with CommunicationManager(logger=logger) as communication:
                            response = communication.send_telemetry(
                                {
                                    "environment": {
                                        "pressure_hpa": pressure_hpa,
                                    }
                                }
                            )
                        radio_ok = "radio_tx_ok" in response
                        logger.event(
                            "Third-threshold pressure transmission: "
                            f"{'OK' if radio_ok else 'NG'} "
                            f"(pressure={pressure_hpa:.2f} hPa, "
                            f"response={response.strip()!r})"
                        )
                    except Exception as exc:
                        logger.event(
                            "Third-threshold pressure transmission: NG "
                            f"(pressure={pressure_hpa:.2f} hPa, "
                            f"{type(exc).__name__}: {exc})"
                        )
                break

            time.sleep(measurement_interval_s)

    logger.event("放出成功")
    return True


def log_sensors(
    sensors: SensorManager,
    output_path: Path,
    stop_event: threading.Event,
    display_event: threading.Event,
    air_temperature_c: float,
    reference_pressure_hpa: float,
    accel_bias_mps2: float,
) -> None:
    """融合高度を更新しながらセンサ値をCSVへ記録する。"""
    start_time = time.monotonic()
    previous_imu_time = start_time
    next_imu_time = start_time
    next_pressure_time = start_time
    next_sample_time = start_time
    fused_altitude_m = 0.0
    vertical_velocity_mps = 0.0
    pressure_samples: deque[float] = deque(maxlen=PRESSURE_MEDIAN_SAMPLES)

    with CsvLogger(
        sensors,
        output_path,
        extra_fields=("fused_altitude_m",),
    ) as csv_logger:
        while not stop_event.is_set():
            now = time.monotonic()
            if now < next_imu_time and stop_event.wait(next_imu_time - now):
                break

            loop_time = time.monotonic()
            dt = loop_time - previous_imu_time
            previous_imu_time = loop_time
            vertical_accel_mps2 = 0.0
            try:
                motion = sensors.get_altitude_motion()
                vertical_accel_mps2 = (
                    vertical_acceleration_mps2(motion) - accel_bias_mps2
                )
            except Exception:
                pass

            vertical_velocity_mps *= math.exp(
                -dt / IMU_VELOCITY_DECAY_TIME_SECONDS
            )
            fused_altitude_m += (
                vertical_velocity_mps * dt
                + 0.5 * vertical_accel_mps2 * dt * dt
            )
            vertical_velocity_mps += vertical_accel_mps2 * dt

            if loop_time >= next_pressure_time:
                try:
                    pressure_hpa = float(
                        sensors.get_environment()["pressure_hpa"]
                    )
                    pressure_samples.append(pressure_hpa)
                    filtered_pressure_hpa = median(pressure_samples)
                    baro_altitude_m = relative_altitude_m(
                        reference_pressure_hpa,
                        filtered_pressure_hpa,
                        air_temperature_c,
                    )
                    fused_altitude_m += ALTITUDE_CORRECTION_GAIN * (
                        baro_altitude_m - fused_altitude_m
                    )
                except Exception:
                    pass
                next_pressure_time += ALTITUDE_PRESSURE_INTERVAL_SECONDS
                if next_pressure_time < time.monotonic():
                    next_pressure_time = time.monotonic()

            if loop_time >= next_sample_time:
                row = csv_logger.write_row(
                    {"fused_altitude_m": f"{fused_altitude_m:.4f}"}
                )
                if display_event.is_set():
                    print(
                        "Sensor: "
                        f"temp={row['temperature_c']}C, "
                        f"pressure={row['pressure_hpa']}hPa, "
                        f"fused_altitude={row['fused_altitude_m']}m, "
                        f"humidity={row['humidity_percent']}%, "
                        f"heading={row['heading_deg']}deg, "
                        f"roll={row['roll_deg']}deg, "
                        f"pitch={row['pitch_deg']}deg, "
                        f"accel=({row['accel_x_mps2']}, "
                        f"{row['accel_y_mps2']}, {row['accel_z_mps2']})m/s^2, "
                        f"gyro=({row['gyro_x_dps']}, {row['gyro_y_dps']}, "
                        f"{row['gyro_z_dps']})dps, "
                        f"calibration={row['calibration']}, "
                        f"distance={row['distance_m']}m, error={row['error']}",
                        flush=True,
                    )
                next_sample_time += SENSOR_INTERVAL_SECONDS
                if next_sample_time < time.monotonic():
                    next_sample_time = time.monotonic()

            next_imu_time += ALTITUDE_IMU_INTERVAL_SECONDS
            if next_imu_time < time.monotonic():
                next_imu_time = time.monotonic()


def check_modules(
    sensors: SensorManager,
    logger: Logger,
) -> None:
    """無線とGNSSを順番に確認し、結果をイベントログへ残す。"""
    logger.event("Post-test module checks started")

    try:
        with CommunicationManager(logger=logger) as communication:
            response = communication.send_text(RADIO_TEST_MESSAGE)
        radio_ok = "radio_tx_ok" in response
        logger.event(
            f"Radio transmission check: {'OK' if radio_ok else 'NG'} "
            f"(response={response.strip()!r})"
        )
    except Exception as exc:
        logger.event(f"Radio transmission check: NG ({type(exc).__name__}: {exc})")

    try:
        sensors.gnss.setup()
        time.sleep(GNSS_READ_WAIT_SECONDS)
        gnss = sensors.get_gnss()
        gnss_ok = bool(gnss.get("raw"))
        logger.event(
            f"GNSS read check: {'OK' if gnss_ok else 'NG'} "
            f"(connected={gnss.get('connected')}, has_fix={gnss.get('has_fix')}, "
            f"latitude={gnss.get('latitude_deg')}, longitude={gnss.get('longitude_deg')}, "
            f"satellites={gnss.get('satellites')})"
        )
    except Exception as exc:
        logger.event(f"GNSS read check: NG ({type(exc).__name__}: {exc})")


def main() -> None:
    air_temperature_c = input_air_temperature_c()
    sensors = SensorManager()
    driver = None
    stop_event = threading.Event()
    display_event = threading.Event()
    display_event.set()
    sensor_thread = None
    interrupted = False
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    logger = Logger(
        log_dir=PROJECT_ROOT / "logs",
        filename=f"drop_{timestamp}.txt",
    )
    sensor_log_path = PROJECT_ROOT / "sensor_csv_logs" / f"drop_{timestamp}.csv"

    try:
        # 1. 環境、9軸、GNSS、距離の各センサを初期化する。
        try:
            sensors.setup()
            logger.event("All sensors initialized")

            configure_bme280_for_altitude(sensors)
            ground_pressure_hpa, accel_bias_mps2, calibration = (
                calibrate_altitude(sensors)
            )
            logger.event(
                f"Ground pressure initialized: {ground_pressure_hpa:.2f} hPa; "
                f"air temperature={air_temperature_c:.1f} C; "
                f"vertical acceleration offset={accel_bias_mps2:+.4f} m/s^2; "
                f"BNO055 calibration=0x{calibration:02X}; "
                f"velocity correction gain={VELOCITY_CORRECTION_GAIN:.1f}"
            )

            # 2. 融合高度を含むセンサ値を0.1秒間隔で記録・表示する。
            logger.event(
                f"Sensor measurement started: interval={SENSOR_INTERVAL_SECONDS:.1f} s"
            )
            sensor_thread = threading.Thread(
                target=log_sensors,
                args=(
                    sensors,
                    sensor_log_path,
                    stop_event,
                    display_event,
                    air_temperature_c,
                    ground_pressure_hpa,
                    accel_bias_mps2,
                ),
                daemon=True,
            )
            sensor_thread.start()
        except Exception as exc:
            logger.event(f"Sensor preparation failed ({type(exc).__name__}: {exc})")
            logger.event("Sequence aborted because sensor preparation did not complete")
            return

        # 3. 気圧変化による放出判定が成功するまで待機する。
        released = judge_release_and_send_pressure(
            sensors,
            logger,
            ground_pressure_hpa=ground_pressure_hpa,
            above_threshold_offsets_hpa=RELEASE_ABOVE_THRESHOLD_OFFSETS_HPA,
            below_threshold_offsets_hpa=RELEASE_BELOW_THRESHOLD_OFFSETS_HPA,
            measurement_interval_s=SENSOR_INTERVAL_SECONDS,
        )
        if not released:
            logger.event("Sequence aborted because release was not detected")
            return

        # 4. 放出成功後、加速度が安定して着地と判定されるまで待機する。
        landed = judge_landing(
            sensors,
            logger=logger,
            timeout_s=None,
        )
        if not landed:
            logger.event("Sequence aborted because landing was not detected")
            return

        # 5. 着地判定後、設定時間が経過したら入力待ちなしで自動溶断する。
        logger.event(
            "Landing detected; automatic fusing starts in "
            f"{LANDING_TO_FUSING_DELAY_SECONDS:.1f} s"
        )
        time.sleep(LANDING_TO_FUSING_DELAY_SECONDS)

        # 以降は vibration_sequence_test.py の溶断以降と同じ流れ。
        display_event.clear()
        try:
            logger.event("Fusing circuit started")
            driver = DriveController()
            fuse_and_kick(driver, pulse_time=0.5)

            # 姿勢の正常化
            NavigationController().restore_posture(driver, sensors)

            # 前方カメラの赤色検知結果に応じてパラシュートを回避する。
            avoidance_result = NavigationController().avoid_parachute(driver, sensors)
            logger.event(f"Parachute avoidance: action={avoidance_result['action']}")
        except Exception as exc:
            logger.event(
                f"Fusing or parachute avoidance failed ({type(exc).__name__}: {exc})"
            )
        finally:
            display_event.set()

        arm = SelfieManager()
        arm.expand()
        arm.retract()

        check_modules(sensors, logger)

        logger.event("Drop test sequence completed")
        print(f"Event log: {logger.log_path}")
        print(f"Sensor CSV: {sensor_log_path}")

    except KeyboardInterrupt:
        interrupted = True
        logger.event("Drop test sequence interrupted")
    except Exception as exc:
        logger.event(f"Unexpected sequence error ({type(exc).__name__}: {exc})")
    finally:
        stop_event.set()
        if sensor_thread is not None:
            sensor_thread.join()
        if driver is not None:
            try:
                driver.cleanup()
            except Exception as exc:
                logger.event(f"Drive cleanup failed ({type(exc).__name__}: {exc})")
        try:
            sensors.close()
        except Exception as exc:
            logger.event(f"Sensor cleanup failed ({type(exc).__name__}: {exc})")

        if interrupted:
            logger.event("Interrupted cleanup completed; logs saved")
            print(f"Event log: {logger.log_path}")
            if sensor_log_path.exists():
                print(f"Sensor CSV: {sensor_log_path}")


if __name__ == "__main__":
    main()
