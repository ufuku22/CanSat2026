#!/usr/bin/env python3
"""ESP32S3 SenseからJPEGを1枚受け取るラズパイ側テスト。"""

from pathlib import Path
import sys

sys.path.append(str(Path(__file__).resolve().parents[1]))

from esp32s3_camera_receiver import (  # noqa: E402
    CameraServerConfig,
    Esp32S3CameraReceiver,
    WifiApConfig,
    log,
)


MANAGE_WIFI = True
AP_SSID = "CanSat-Camera"
AP_PASSWORD = "cansat2026"
ORIGINAL_CONNECTION = "netplan-wlan0-KimuraLab_StudentRoom"
TCP_PORT = 5000

# ESP32S3は最大60秒sleepしてからAPを探すので、テスト側は長めに待つ。
TIMEOUT_SEC = 120.0

# 受信した画像は raw_images 直下へ保存する。
SAVE_DIR = Path("raw_images")


def main() -> None:
    # 起動したら、AP起動から画像保存、Wi-Fi復帰まで自動で行う。
    log("ESP32S3 camera receive test started")
    log("USB SSHなら、この表示を見ながらWi-Fi切替中も監視できる")
    receiver = Esp32S3CameraReceiver(
        wifi=WifiApConfig(
            ap_ssid=AP_SSID,
            ap_password=AP_PASSWORD,
            original_connection=ORIGINAL_CONNECTION,
        ),
        server=CameraServerConfig(
            port=TCP_PORT,
            timeout_sec=TIMEOUT_SEC,
            image_dir=SAVE_DIR,
        ),
        manage_wifi=MANAGE_WIFI,
    )
    if MANAGE_WIFI and not receiver.has_wifi_permission():
        log("Wi-Fiを切り替えるため、sudoで実行してください")
        log("例: sudo python3 test_scripts/test_esp32s3_camera_receiver.py")
        raise SystemExit(1)

    try:
        saved_path = receiver.run_capture_sequence()
    except Exception as exc:
        log(f"test failed: {exc}")
        raise SystemExit(1) from exc

    log(f"test saved: {saved_path}")


if __name__ == "__main__":
    main()
