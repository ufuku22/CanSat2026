#!/usr/bin/env python3
"""ミッションと同じ構成で定期テレメトリをバックグラウンド送信する。"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from communication_manager import CommunicationManager
from config import MissionConfig
from logger import Logger
from sensor_manager import SensorManager
from telemetry_service import TelemetryService


class CommunicationResultLogger:
    """通信結果を画面表示し、ファイルを作らず成功回数だけ数える。"""

    def __init__(self) -> None:
        self.attempts = 0
        self.successes = 0

    def event(self, message: str) -> None:
        print(message, flush=True)
        if "telemetry seq=" in message or "telemetry error (" in message:
            self.attempts += 1
        if "radio_tx_ok=True" in message:
            self.successes += 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test mission-style background telemetry transmission."
    )
    parser.add_argument(
        "--interval",
        type=float,
        help="telemetry interval in seconds; default uses config.py",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    interval_s = (
        float(args.interval)
        if args.interval is not None
        else float(MissionConfig.TELEMETRY_INTERVAL_S)
    )
    if interval_s <= 0.0:
        raise SystemExit("--interval must be greater than 0")
    event_logger = Logger(log_to_file=False)
    communication_logger = CommunicationResultLogger()

    sensors: SensorManager | None = None
    communication: CommunicationManager | None = None
    telemetry: TelemetryService | None = None
    exit_code = 1

    try:
        event_logger.event(
            f"バックグラウンドテレメトリ試験開始 "
            f"(interval={interval_s:g}s)"
        )
        sensors = SensorManager(status_callback=event_logger.event)
        sensors.setup(enable_distance_sensor=False)
        sensors.set_gnss_cache_max_age_s(MissionConfig.GNSS_CACHE_MAX_AGE_S)

        communication = CommunicationManager(logger=communication_logger)
        communication.setup()
        telemetry = TelemetryService(
            sensors,
            event_logger,
            interval_s=interval_s,
            communication=communication,
            communication_logger=communication_logger,
        )
        telemetry.set_phase("telemetry_test")
        telemetry.start()

        event_logger.event("メイン処理を継続中。Ctrl+Cで終了します")

        started_at = time.monotonic()
        next_heartbeat_at = started_at
        while True:
            now = time.monotonic()
            if now >= next_heartbeat_at:
                event_logger.event(
                    f"メイン処理稼働中 (elapsed={now - started_at:.1f}s)"
                )
                next_heartbeat_at = now + 5.0
            time.sleep(0.1)
        exit_code = 0
    except KeyboardInterrupt:
        event_logger.event("試験をCtrl+Cで終了します")
        exit_code = 0
    except Exception as exc:
        event_logger.event(f"試験失敗 ({type(exc).__name__}: {exc})")
    finally:
        if telemetry is not None:
            telemetry.stop()
        elif communication is not None:
            communication.close()
        if sensors is not None:
            sensors.close()

    event_logger.event(
        f"試験結果: telemetry_attempts={communication_logger.attempts}, "
        f"radio_tx_ok={communication_logger.successes}"
    )
    if (
        exit_code == 0
        and communication_logger.attempts > 0
        and communication_logger.successes > 0
    ):
        event_logger.event("バックグラウンドテレメトリ試験成功")
        return 0

    event_logger.event("バックグラウンドテレメトリ試験失敗")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
