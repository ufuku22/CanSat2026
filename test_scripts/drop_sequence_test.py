#!/usr/bin/env python3
"""投下試験用の一連動作を実行するテスト。"""

from datetime import datetime
from pathlib import Path
import sys
import threading
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from altitude_estimator import (
    AltitudeEstimator,
    IMU_INTERVAL_S as ALTITUDE_IMU_INTERVAL_SECONDS,
    calibrate_altitude,
    configure_bme280_for_altitude,
)
from drive_controller import DriveController
from fusing import fuse_and_kick
from judge import judge_landing
from logger import CsvLogger, Logger
from navigation_controller import NavigationController
from selfie_manager import SelfieManager
from sensor_manager import SensorManager


# CSV記録と放出判定に使うセンサ値の測定間隔
SENSOR_INTERVAL_SECONDS = 0.1

# 放出判定用の気圧しきい値。投下高度に合わせて試験前に調整する。
RELEASE_ABOVE_THRESHOLD_OFFSETS_HPA = (2, 0.5)
RELEASE_BELOW_THRESHOLD_OFFSETS_HPA = (1, 2.5)

# 着地判定後、自動的に溶断を始めるまでの待機時間
LANDING_TO_FUSING_DELAY_SECONDS = 3.0

def input_air_temperature_c() -> float:
    """高度計算に使う外気温を入力する。"""
    while True:
        try:
            temperature_c = float(input("外気温 [°C] を入力してください: "))
        except ValueError:
            print("数値で入力してください。")
            continue
        if temperature_c <= -273.15:
            print("-273.15°Cより高い値を入力してください。")
            continue
        return temperature_c


def judge_release_and_send_pressure(
    sensors: SensorManager,
    logger: Logger,
    *,
    ground_pressure_hpa: float,
    above_threshold_offsets_hpa: tuple[float, float],
    below_threshold_offsets_hpa: tuple[float, float],
    measurement_interval_s: float,
) -> bool:
    """放出判定の3つ目の閾値到達時に、その気圧を無線送信する。"""
    checks = (
        (below_threshold_offsets_hpa[0], "below"),
        (below_threshold_offsets_hpa[1], "below"),
        (above_threshold_offsets_hpa[0], "above"),
        (above_threshold_offsets_hpa[1], "above"),
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
    next_imu_time = start_time
    next_sample_time = start_time
    estimator = AltitudeEstimator(
        sensors,
        air_temperature_c,
        reference_pressure_hpa,
        accel_bias_mps2,
        tolerate_read_errors=True,
    )

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
            estimate = estimator.update(loop_time)

            if loop_time >= next_sample_time:
                row = csv_logger.write_row(
                    {"fused_altitude_m": f"{estimate.fused_altitude_m:.4f}"}
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
                f"BNO055 calibration=0x{calibration:02X}"
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
