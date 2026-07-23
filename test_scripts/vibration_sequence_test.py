#!/usr/bin/env python3
"""振動試験用の一連動作を実行するテスト。"""

from datetime import datetime
from pathlib import Path
import sys
import threading
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from communication_manager import CommunicationManager
from drive_controller import DriveController
from fusing import fuse_and_kick
from logger import CsvLogger, Logger
from navigation_controller import NavigationController
from selfie_manager import SelfieManager
from sensor_manager import SensorManager


# 測定間隔
SENSOR_INTERVAL_SECONDS = 0.1

# 試験終了後のモジュール確認に使う設定
RADIO_TEST_MESSAGE = "VIBRATION_TEST_COMPLETE"
GNSS_READ_WAIT_SECONDS = 1.0


def log_sensors(
    sensors: SensorManager,
    output_path: Path,
    stop_event: threading.Event,
    display_event: threading.Event,
) -> None:
    """センサ値をCSVへ記録し、表示が有効な間はコンソールにも表示する。"""
    next_sample_time = time.monotonic()

    with CsvLogger(sensors, output_path) as csv_logger:
        while not stop_event.is_set():
            # 指定した測定時刻まで待つ。停止指示が来た場合はすぐに終了する。
            now = time.monotonic()
            if now < next_sample_time and stop_event.wait(next_sample_time - now):
                break

            row = csv_logger.write_row()
            if display_event.is_set():
                print(
                    "Sensor: "
                    f"temp={row['temperature_c']}C, pressure={row['pressure_hpa']}hPa, "
                    f"humidity={row['humidity_percent']}%, "
                    f"heading={row['heading_deg']}deg, roll={row['roll_deg']}deg, "
                    f"pitch={row['pitch_deg']}deg, "
                    f"accel=({row['accel_x_mps2']}, {row['accel_y_mps2']}, "
                    f"{row['accel_z_mps2']})m/s^2, "
                    f"gyro=({row['gyro_x_dps']}, {row['gyro_y_dps']}, "
                    f"{row['gyro_z_dps']})dps, calibration={row['calibration']}, "
                    f"distance={row['distance_m']}m, "
                    f"error={row['error']}",
                    flush=True,
                )
            next_sample_time += SENSOR_INTERVAL_SECONDS

            # 読み取りに時間がかかった場合は、遅れた分を連続測定しない。
            if next_sample_time < time.monotonic():
                next_sample_time = time.monotonic()


def check_modules(
    sensors: SensorManager,
    logger: Logger,
) -> None:
    """無線とGNSSを順番に確認し、結果をイベントログへ残す。"""
    logger.event("Post-test module checks started")

    # 無線モジュールから短い文字列を送信し、送信完了応答を確認する。
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

    # GNSSモジュールを初期化し、NMEAデータを読み出せるか確認する。
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
    # 使用する機器と、バックグラウンド測定の停止指示を準備する。
    sensors = SensorManager()
    driver = None
    stop_event = threading.Event()
    display_event = threading.Event()
    display_event.set()
    sensor_thread = None
    interrupted = False
    fusing_authorized = False
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    logger = Logger(
        log_dir=PROJECT_ROOT / "logs",
        filename=f"vibration_{timestamp}.txt",
    )
    sensor_log_path = (
        PROJECT_ROOT / "sensor_csv_logs" / f"vibration_{timestamp}.csv"
    )

    try:
        # 1. 環境センサ、9軸センサ、距離センサを初期化する。
        try:
            CsvLogger.setup_sensors(sensors)

            # 2. CSV記録と画面表示を別スレッドで開始し、Enter入力まで待機する。
            logger.event("Sensor measurement started")
            sensor_thread = threading.Thread(
                target=log_sensors,
                args=(sensors, sensor_log_path, stop_event, display_event),
                daemon=True,
            )
            sensor_thread.start()
            if not sys.stdin.isatty():
                logger.event(
                    "Fusing aborted: interactive terminal input is not available"
                )
                return
            try:
                input("溶断を開始するにはEnterキーを押してください: ")
            except EOFError:
                logger.event("Fusing aborted: terminal input was closed")
                return
            fusing_authorized = True
            logger.event("Fusing authorized by Enter key")
        except Exception as exc:
            logger.event(f"Sensor measurement failed ({type(exc).__name__}: {exc})")
            logger.event("Fusing aborted because preparation did not complete")
            return

        # 明示的なEnter入力が成功しなければ、以降の駆動処理には絶対に進まない。
        if not fusing_authorized:
            logger.event("Fusing aborted: authorization was not received")
            return

        # 3～4. 溶断とパラシュート回避の間だけセンサ値の画面表示を止める。
        display_event.clear()
        try:
            # 3. 溶断回路を作動させ、直後にモーターを一瞬だけ後転させる。
            logger.event("Fusing circuit started")
            driver = DriveController()
            fuse_and_kick(driver, pulse_time=0.5)

            # スタビを機体の下側から出す
            #time.sleep(3)
            #driver.flip()
            #driver.reverse_stabilizer()

            # 姿勢の正常化
            NavigationController().restore_posture(driver, sensors)

            # 4. 前方カメラの紫色検知結果に応じてパラシュートを回避する。
            avoidance_result = NavigationController().avoid_parachute(driver, sensors)
            logger.event(f"Parachute avoidance: action={avoidance_result['action']}")
        except Exception as exc:
            logger.event(f"Fusing or parachute avoidance failed ({type(exc).__name__}: {exc})")
        finally:
            display_event.set()

        # 5. アームを展開して写真を撮影し、必ずアームを収納する。
        #try:
        #    arm = SelfieManager()
        #    try:
        #        arm.expand()
        #        selfie_path = arm.capture()
        #        logger.event(f"Selfie image: {selfie_path}")
        #    except Exception as exc:
        #        logger.event(f"Selfie sequence failed ({type(exc).__name__}: {exc})")
        #    finally:
        #        try:
        #            arm.retract()
        #        except Exception as exc:
        #            logger.event(f"Arm retraction failed ({type(exc).__name__}: {exc})")
        #except Exception as exc:
        #    logger.event(f"Selfie manager initialization failed ({type(exc).__name__}: {exc})")
        arm = SelfieManager()
        arm.expand()
        arm.retract()

        # 6. 無線とGNSSが振動試験後も動作するか確認する。
        check_modules(sensors, logger)

        logger.event("Vibration test sequence completed")
        print(f"Event log: {logger.log_path}")
        print(f"Sensor CSV: {sensor_log_path}")

    except KeyboardInterrupt:
        interrupted = True
        logger.event("Vibration test sequence interrupted")
    except Exception as exc:
        logger.event(f"Unexpected sequence error ({type(exc).__name__}: {exc})")
    finally:
        # CSV記録を止めてから、モーターとセンサのリソースを解放する。
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
