#!/usr/bin/env python3
"""End-to-end BBM integration test for CanSat2026 hardware."""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from communication_manager import (
    DEFAULT_MAX_RADIO_PAYLOAD,
    CommunicationManager,
)
from drive_controller import DriveController
from logger import Logger
from selfie_manager import SelfieManager
from sensor_manager import SensorManager


PROJECT_DIR = Path(__file__).resolve().parent
RAW_IMAGE_DIR = PROJECT_DIR / "raw_images"
COMPRESSED_IMAGE_DIR = PROJECT_DIR / "images"
LOG_DIR = PROJECT_DIR / "logs"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the BBM integration test: Wi-Fi, sensors, motors, selfie, compression, and radio send."
    )
    parser.add_argument("--drive-speed", type=float, default=100.0, help="drive motor speed percent")
    parser.add_argument("--drive-seconds", type=float, default=3.0, help="seconds for forward and reverse drive")
    parser.add_argument("--image-width", type=int, default=640, help="maximum compressed image width")
    parser.add_argument("--image-height", type=int, default=480, help="maximum compressed image height")
    parser.add_argument("--jpeg-quality", type=int, default=70, help="JPEG quality from 1 to 100")
    parser.add_argument("--port", default="/dev/serial0", help="TLM922S UART port")
    parser.add_argument("--baudrate", type=int, default=115200, help="TLM922S UART baudrate")
    parser.add_argument("--radio-timeout", type=float, default=4.0, help="seconds to wait for radio responses")
    parser.add_argument("--packet-delay", type=float, default=0.2, help="seconds between image packets")
    parser.add_argument("--max-radio-payload", type=int, default=DEFAULT_MAX_RADIO_PAYLOAD)
    return parser.parse_args()


def event(logger: Logger, message: str) -> None:
    print(message, flush=True)
    logger.write_event(message)


def read_non_gnss_sensors(sensors: SensorManager) -> dict[str, Any]:
    return {
        "environment": sensors.get_environment(),
        "imu": sensors.get_imu(),
        "distance_m": sensors.get_distance_m(),
    }


def drive_forward_and_reverse(logger: Logger, speed: float, seconds: float) -> None:
    driver: DriveController | None = None
    try:
        event(logger, f"Drive test start: speed={speed:g}%, seconds={seconds:g}")
        driver = DriveController()

        event(logger, "Drive forward")
        driver.drive(speed)
        time.sleep(seconds)
        driver.stop()
        time.sleep(0.5)

        event(logger, "Drive reverse")
        driver.drive(-speed)
        time.sleep(seconds)
        driver.stop()
        event(logger, "Drive test complete")
    finally:
        if driver is not None:
            driver.cleanup()
            event(logger, "Drive GPIO cleaned up")


def compressed_image_path(raw_path: Path) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return COMPRESSED_IMAGE_DIR / f"{raw_path.stem}_vga_{timestamp}.jpg"


def compress_image_keep_aspect(
    raw_path: Path,
    *,
    max_width: int,
    max_height: int,
    quality: int,
) -> Path:
    import cv2

    if not 1 <= quality <= 100:
        raise ValueError("--jpeg-quality must be between 1 and 100")
    if max_width <= 0 or max_height <= 0:
        raise ValueError("--image-width and --image-height must be positive")

    image = cv2.imread(str(raw_path))
    if image is None:
        raise RuntimeError(f"Could not read image: {raw_path}")

    height, width = image.shape[:2]
    scale = min(max_width / width, max_height / height, 1.0)
    new_width = max(1, int(round(width * scale)))
    new_height = max(1, int(round(height * scale)))

    if (new_width, new_height) != (width, height):
        image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)

    output_path = compressed_image_path(raw_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(
        str(output_path),
        image,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)],
    )
    if not ok:
        raise RuntimeError(f"Could not write compressed image: {output_path}")

    return output_path


def send_image_by_radio(args: argparse.Namespace, image_path: Path, logger: Logger) -> None:
    event(logger, f"Radio image send start: {image_path}")
    with CommunicationManager(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.radio_timeout,
    ) as comm:
        result = comm.send_image(
            image_path,
            max_radio_payload=args.max_radio_payload,
            inter_packet_delay=args.packet_delay,
        )

    event(
        logger,
        (
            "Radio image send complete: "
            f"size={result.file_size} bytes, k={result.k}, m={result.m}, "
            f"block={result.block_size}, file_id={result.file_id:08x}, "
            f"radio_tx_ok={result.radio_tx_ok_count}/{len(result.responses)}"
        ),
    )
    if not result.all_radio_tx_ok:
        raise RuntimeError("Some radio image packets did not report radio_tx_ok")


def main() -> int:
    args = parse_args()
    log_name = f"test_BBM_{datetime.now():%Y%m%d_%H%M%S}.txt"
    logger = Logger(log_dir=LOG_DIR, filename=log_name)
    logger.reset_timer()

    selfie = SelfieManager(image_dir=RAW_IMAGE_DIR)
    sensors: SensorManager | None = None
    arm_expanded = False

    try:
        event(logger, "BBM integration test started")
        event(logger, f"Options: {json.dumps(vars(args), ensure_ascii=False)}")

        event(logger, "Starting Wi-Fi AP")
        selfie.start_ap()
        event(logger, "Waiting for ESP32S3 connection")
        selfie.wait_connection()
        event(logger, "ESP32S3 connection established")

        event(logger, "Sensor setup start")
        sensors = SensorManager()
        sensors.setup()
        sensor_data = read_non_gnss_sensors(sensors)
        logger.write_sensor(sensor_data)
        event(logger, f"Sensor data: {json.dumps(sensor_data, ensure_ascii=False, default=str)}")

        drive_forward_and_reverse(logger, args.drive_speed, args.drive_seconds)

        event(logger, "Selfie arm expand start")
        selfie.expand()
        arm_expanded = True
        event(logger, "Selfie arm expand complete")

        event(logger, "ESP32S3 capture start")
        raw_path = selfie.capture_connected()
        event(logger, f"Raw image saved: {raw_path} ({raw_path.stat().st_size} bytes)")

        event(logger, "Selfie arm retract start")
        selfie.retract()
        arm_expanded = False
        event(logger, "Selfie arm retract complete")

        event(
            logger,
            (
                "Image compression start: "
                f"max={args.image_width}x{args.image_height}, quality={args.jpeg_quality}"
            ),
        )
        compressed_path = compress_image_keep_aspect(
            raw_path,
            max_width=args.image_width,
            max_height=args.image_height,
            quality=args.jpeg_quality,
        )
        compressed_size = compressed_path.stat().st_size
        event(logger, f"Compressed image saved: {compressed_path} ({compressed_size} bytes)")
        print(f"Compressed image size: {compressed_size} bytes", flush=True)

        send_image_by_radio(args, compressed_path, logger)

        event(logger, "BBM integration test completed successfully")
        return 0
    except KeyboardInterrupt:
        event(logger, "BBM integration test interrupted")
        return 130
    except Exception as exc:
        event(logger, f"BBM integration test failed: {type(exc).__name__}: {exc}")
        return 1
    finally:
        if arm_expanded:
            try:
                event(logger, "Selfie arm retract start during cleanup")
                selfie.retract()
                event(logger, "Selfie arm retract complete during cleanup")
            except Exception as exc:
                event(logger, f"Selfie arm cleanup failed: {type(exc).__name__}: {exc}")

        if sensors is not None:
            try:
                sensors.close()
                event(logger, "Sensor manager closed")
            except Exception as exc:
                event(logger, f"Sensor cleanup failed: {type(exc).__name__}: {exc}")

        try:
            selfie.close_connection()
            selfie.restore_wifi()
            event(logger, "Wi-Fi restored and ESP32S3 connection closed")
        except Exception as exc:
            event(logger, f"Wi-Fi cleanup failed: {type(exc).__name__}: {exc}")

        event(logger, f"Log file: {logger.log_path}")


if __name__ == "__main__":
    sys.exit(main())
