#!/usr/bin/env python3
"""能代ミッションの自撮り処理だけを実機でテストする。"""

from __future__ import annotations

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from config import NoshiroMissionConfig  # noqa: E402
from drive_controller import DriveController  # noqa: E402
from logger import Logger  # noqa: E402
from mission_controller import MissionController  # noqa: E402
from mission_noshiro import (  # noqa: E402
    TARGET_LATITUDE_DEG,
    TARGET_LONGITUDE_DEG,
    USE_SIMPLE_SELFIE_MISSION,
)
from navigation_controller import NavigationController  # noqa: E402
from selfie_manager import SelfieManager  # noqa: E402
from sensor_manager import SensorManager  # noqa: E402
from telemetry_service import TelemetryService  # noqa: E402


LOG_FILE = "noshiro_selfie_mission_test.log"


def main() -> int:
    """IMU・モーター・自撮り機能だけを初期化して1回実行する。"""
    logger = Logger(filename=LOG_FILE)
    selfie = SelfieManager(logger=logger)
    sensors: SensorManager | None = None
    driver: DriveController | None = None
    mission: MissionController | None = None
    exit_code = 0

    logger.event("能代自撮りミッションテスト開始")
    try:
        sensors = SensorManager(status_callback=logger.event)
        sensors.imu.setup()
        driver = DriveController()
        navigator = NavigationController(
            TARGET_LATITUDE_DEG,
            TARGET_LONGITUDE_DEG,
            logger=logger,
        )
        telemetry = TelemetryService(
            sensors,
            logger,
            interval_s=NoshiroMissionConfig.TELEMETRY_INTERVAL_S,
        )
        mission = MissionController(
            config=NoshiroMissionConfig,
            logger=logger,
            sensors=sensors,
            driver=driver,
            navigator=navigator,
            telemetry=telemetry,
            selfie=selfie,
        )
        mission.run_selfie_mission(simple=USE_SIMPLE_SELFIE_MISSION)
    except KeyboardInterrupt:
        logger.event("能代自撮りミッションテスト中断")
        exit_code = 130
    except (Exception, SystemExit) as exc:
        logger.event(
            f"能代自撮りミッションテスト失敗 "
            f"({type(exc).__name__}: {exc})"
        )
        exit_code = 1
    finally:
        try:
            if mission is not None:
                mission.close()
            else:
                try:
                    if driver is not None:
                        driver.cleanup()
                finally:
                    if sensors is not None:
                        sensors.close()
        finally:
            # MissionController.close()でも実行されるが、単独テストでは
            # 初期化途中の失敗時も含めてWi-Fi復元を必ず試みる。
            try:
                selfie.close_server()
            finally:
                selfie.restore_wifi()
        logger.event("自撮り用APを停止し、元のWi-Fiへの復元処理を完了")

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
