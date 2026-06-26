#!/usr/bin/env python3
"""End-to-end EM integration test for CanSat2026 hardware."""

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
        description="Run the EM integration test: Wi-Fi, sensors, motors, selfie, compression, and radio send."
    )
    parser.add_argument("--drive-speed", type=float, default=100.0, help="drive motor speed percent")
    parser.add_argument("--drive-seconds", type=float, default=3.0, help="seconds for forward and reverse drive")
    parser.add_argument("--image-width", type=int, default=320, help="maximum compressed image width")
    parser.add_argument("--image-height", type=int, default=240, help="maximum compressed image height")
    parser.add_argument("--jpeg-quality", type=int, default=35, help="initial JPEG quality from 1 to 100")
    parser.add_argument("--target-image-bytes", type=int, default=5000, help="target compressed image size")
    parser.add_argument("--max-image-bytes", type=int, default=6500, help="maximum accepted compressed image size")
    parser.add_argument("--min-image-width", type=int, default=160, help="minimum compressed image width")
    parser.add_argument("--min-image-height", type=int, default=120, help="minimum compressed image height")
    parser.add_argument("--min-jpeg-quality", type=int, default=15, help="minimum JPEG quality")
    parser.add_argument("--port", default="/dev/serial0", help="TLM922S UART port")
    parser.add_argument("--baudrate", type=int, default=115200, help="TLM922S UART baudrate")
    parser.add_argument("--radio-timeout", type=float, default=10.0, help="seconds to wait for radio responses")
    parser.add_argument("--packet-delay", type=float, default=0.2, help="seconds between image packets")
    parser.add_argument("--max-radio-payload", type=int, default=DEFAULT_MAX_RADIO_PAYLOAD)
    parser.add_argument(
        "--sensor-settle-seconds",
        type=float,
        default=1.0,
        help="seconds to wait before I2C sensor setup after ESP32S3 connects",
    )
    parser.add_argument(
        "--sensor-setup-retries",
        type=int,
        default=100,
        help="I2C sensor setup retry count",
    )
    parser.add_argument(
        "--sensor-read-retries",
        type=int,
        default=5,
        help="I2C sensor read retry count after setup succeeds",
    )
    parser.add_argument(
        "--sensor-retry-delay",
        type=float,
        default=0.5,
        help="seconds between I2C sensor retries",
    )
    return parser.parse_args()


def event(logger: Logger, message: str) -> None:
    logger.event(message)


def run_logged_step(
    logger: Logger,
    name: str,
    func: Any,
    *,
    retries: int | None = 1,
    retry_delay: float = 0.0,
) -> Any:
    event(logger, f"{name} start")
    last_exc: Exception | None = None
    attempt = 1
    while retries is None or attempt <= max(1, retries):
        try:
            result = func()
        except Exception as exc:
            last_exc = exc
            if retries is None:
                event(logger, f"{name} attempt {attempt} failed: {type(exc).__name__}: {exc}")
            else:
                event(logger, f"{name} attempt {attempt}/{max(1, retries)} failed: {type(exc).__name__}: {exc}")
            if (retries is None or attempt < max(1, retries)) and retry_delay > 0:
                time.sleep(retry_delay)
            attempt += 1
            continue
        event(logger, f"{name} complete")
        return result

    if last_exc is None:
        raise RuntimeError(f"{name} failed without an exception")
    event(logger, f"{name} failed after {max(1, retries)} attempts")
    raise last_exc


def setup_non_gnss_sensors(
    sensors: SensorManager,
    logger: Logger,
    *,
    retries: int,
    retry_delay: float,
) -> None:
    run_logged_step(logger, "BME280 setup", sensors.environment.setup, retries=retries, retry_delay=retry_delay)
    run_logged_step(logger, "BNO055 setup", sensors.imu.setup, retries=retries, retry_delay=retry_delay)
    run_logged_step(logger, "TSD20 setup", sensors.distance.setup, retries=retries, retry_delay=retry_delay)


def read_non_gnss_sensors_logged(
    sensors: SensorManager,
    logger: Logger,
    *,
    retries: int,
    retry_delay: float,
) -> dict[str, Any]:
    return {
        "environment": run_logged_step(
            logger,
            "BME280 read",
            sensors.get_environment,
            retries=retries,
            retry_delay=retry_delay,
        ),
        "imu": run_logged_step(
            logger,
            "BNO055 read",
            sensors.get_imu,
            retries=retries,
            retry_delay=retry_delay,
        ),
        "distance_m": run_logged_step(
            logger,
            "TSD20 read",
            sensors.get_distance_m,
            retries=retries,
            retry_delay=retry_delay,
        ),
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
    target_bytes: int,
    max_bytes: int,
    min_width: int,
    min_height: int,
    min_quality: int,
    logger: Logger,
) -> Path:
    import cv2

    if not 1 <= quality <= 100:
        raise ValueError("--jpeg-quality must be between 1 and 100")
    if not 1 <= min_quality <= quality:
        raise ValueError("--min-jpeg-quality must be between 1 and --jpeg-quality")
    if max_width <= 0 or max_height <= 0:
        raise ValueError("--image-width and --image-height must be positive")
    if min_width <= 0 or min_height <= 0:
        raise ValueError("--min-image-width and --min-image-height must be positive")
    if min_width > max_width or min_height > max_height:
        raise ValueError("minimum image size must not exceed maximum image size")
    if target_bytes <= 0 or max_bytes <= 0:
        raise ValueError("--target-image-bytes and --max-image-bytes must be positive")

    image = cv2.imread(str(raw_path))
    if image is None:
        raise RuntimeError(f"Could not read image: {raw_path}")

    source_height, source_width = image.shape[:2]
    output_path = compressed_image_path(raw_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    best_path = output_path
    best_size: int | None = None
    current_width = max_width
    current_height = max_height
    current_quality = quality

    for attempt in range(1, 25):
        scale = min(current_width / source_width, current_height / source_height, 1.0)
        new_width = max(1, int(round(source_width * scale)))
        new_height = max(1, int(round(source_height * scale)))
        resized = image
        if (new_width, new_height) != (source_width, source_height):
            resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)

        ok = cv2.imwrite(
            str(output_path),
            resized,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(current_quality)],
        )
        if not ok:
            raise RuntimeError(f"Could not write compressed image: {output_path}")

        size = output_path.stat().st_size
        best_size = size
        best_path = output_path
        event(
            logger,
            (
                "Image compression attempt "
                f"{attempt}: {new_width}x{new_height}, quality={current_quality}, size={size} bytes"
            ),
        )
        if size <= max_bytes:
            if size > target_bytes:
                event(logger, f"Compressed image is within limit and near target: {size} bytes")
            return output_path

        if current_quality > min_quality:
            current_quality = max(min_quality, current_quality - 5)
            continue

        next_width = max(min_width, int(current_width * 0.85))
        next_height = max(min_height, int(current_height * 0.85))
        if (next_width, next_height) == (current_width, current_height):
            break
        current_width = next_width
        current_height = next_height
        current_quality = quality

    if best_size is not None and best_size <= max_bytes:
        return best_path
    raise RuntimeError(
        f"Could not compress image under {max_bytes} bytes. "
        f"Best size was {best_size} bytes at minimum settings."
    )


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
    log_name = f"test_EM_{datetime.now():%Y%m%d_%H%M%S}.txt"
    logger = Logger(log_dir=LOG_DIR, filename=log_name)
    logger.reset_timer()

    selfie = SelfieManager(image_dir=RAW_IMAGE_DIR)
    sensors: SensorManager | None = None
    arm_expanded = False

    try:
        event(logger, "EM integration test started")
        event(logger, f"Options: {json.dumps(vars(args), ensure_ascii=False)}")

        event(logger, "Starting Wi-Fi AP")
        selfie.start_ap()
        event(logger, "Waiting for ESP32S3 connection")
        selfie.wait_connection()
        event(logger, "ESP32S3 connection established")

        if args.sensor_settle_seconds > 0:
            event(logger, f"Waiting before sensor setup: {args.sensor_settle_seconds:g} seconds")
            time.sleep(args.sensor_settle_seconds)

        event(logger, "Sensor setup start")
        sensors = SensorManager()
        setup_non_gnss_sensors(
            sensors,
            logger,
            retries=args.sensor_setup_retries,
            retry_delay=args.sensor_retry_delay,
        )
        event(logger, "Sensor setup complete")
        sensor_data = read_non_gnss_sensors_logged(
            sensors,
            logger,
            retries=args.sensor_read_retries,
            retry_delay=args.sensor_retry_delay,
        )
        logger.sensor(sensor_data)
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
                f"max={args.image_width}x{args.image_height}, quality={args.jpeg_quality}, "
                f"target={args.target_image_bytes} bytes, limit={args.max_image_bytes} bytes"
            ),
        )
        compressed_path = compress_image_keep_aspect(
            raw_path,
            max_width=args.image_width,
            max_height=args.image_height,
            quality=args.jpeg_quality,
            target_bytes=args.target_image_bytes,
            max_bytes=args.max_image_bytes,
            min_width=args.min_image_width,
            min_height=args.min_image_height,
            min_quality=args.min_jpeg_quality,
            logger=logger,
        )
        compressed_size = compressed_path.stat().st_size
        event(logger, f"Compressed image saved: {compressed_path} ({compressed_size} bytes)")
        print(f"Compressed image size: {compressed_size} bytes", flush=True)

        send_image_by_radio(args, compressed_path, logger)

        event(logger, "EM integration test completed successfully")
        return 0
    except KeyboardInterrupt:
        event(logger, "EM integration test interrupted")
        return 130
    except Exception as exc:
        event(logger, f"EM integration test failed: {type(exc).__name__}: {exc}")
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
