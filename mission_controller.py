#!/usr/bin/env python3
"""既存の各機能をARLISSミッションの順番につなぐ。"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import time
from typing import Any

from config import MissionConfig, NavigationTargetConfig
from drive_controller import DriveController
from fusing import fuse_and_kick
from image_processor import ImageProcessor
from judge import judge_landing, judge_release, read_median_pressure_hpa
from logger import Logger
from navigation_controller import NavigationController
from navigation_goal import guide_to_red_cone, search_around_gnss_goal
from selfie_manager import SelfieManager
from sensor_manager import SensorManager
from telemetry_service import TelemetryService


PROJECT_ROOT = Path(__file__).resolve().parent


class MissionController:
    """ミッションの順番、再試行、機器の終了処理を管理する。"""

    def __init__(
        self,
        *,
        config: Any = MissionConfig,
        logger: Logger | None = None,
        sensors: SensorManager | None = None,
        driver: DriveController | None = None,
        navigator: NavigationController | None = None,
        telemetry: TelemetryService | None = None,
        selfie: SelfieManager | None = None,
    ) -> None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.config = config
        self.logger = logger or Logger(
            log_dir=PROJECT_ROOT / "logs",
            filename=f"mission_{timestamp}.txt",
        )
        self.sensors = sensors
        self.driver = driver
        self.navigator = navigator
        self.telemetry = telemetry
        self.selfie = selfie
        self.selfie_wifi_started = False
        self.phase = "startup"
        self.ground_pressure_hpa: float | None = None
        self.landing_reference_position: dict[str, Any] | None = None

    def __enter__(self) -> "MissionController":
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        if exc is not None:
            self.logger.event(
                f"ミッション異常終了 ({type(exc).__name__}: {exc})"
            )
        self.close()

    def prepare(self) -> None:
        """センサと基準気圧を準備する。"""
        if float(self.config.LANDING_CLEARANCE_DISTANCE_M) <= 0.0:
            raise ValueError("LANDING_CLEARANCE_DISTANCE_M must be greater than 0")

        self._set_phase("preparing")
        if self.sensors is None:
            self.sensors = SensorManager()
        self.sensors.setup()

        if self.navigator is None:
            self.navigator = NavigationController(
                target_latitude_deg=NavigationTargetConfig.TARGET_LATITUDE_DEG,
                target_longitude_deg=NavigationTargetConfig.TARGET_LONGITUDE_DEG,
            )

        self.ground_pressure_hpa = read_median_pressure_hpa(self.sensors)
        if self.telemetry is None:
            self.telemetry = TelemetryService(
                self.sensors,
                self.logger,
                interval_s=self.config.TELEMETRY_INTERVAL_S,
            )
        self._set_phase("waiting_for_release")
        self.logger.event(
            f"ミッション準備完了 (基準気圧={self.ground_pressure_hpa:.2f} hPa)"
        )

    def wait_for_release(self) -> None:
        """気圧変化から放出を判定する。"""
        sensors = self._sensors()
        if self.ground_pressure_hpa is None:
            raise RuntimeError("prepare() must be called before wait_for_release()")

        released = judge_release(
            sensors,
            logger=self.logger,
            ground_pressure_hpa=self.ground_pressure_hpa,
            above_threshold_offsets_hpa=(
                self.config.RELEASE_ABOVE_THRESHOLD_OFFSETS_HPA
            ),
            below_threshold_offsets_hpa=(
                self.config.RELEASE_BELOW_THRESHOLD_OFFSETS_HPA
            ),
            timeout_s=None,
        )
        if not released:
            raise RuntimeError("放出を判定できませんでした")
        self._set_phase("descending")

    def start_telemetry(self) -> None:
        """放出後の定期テレメトリ送信を開始する。"""
        if self.telemetry is None:
            raise RuntimeError("prepare() must be called before start_telemetry()")
        self.telemetry.set_phase(self.phase)
        self.telemetry.start()

    def wait_for_landing(self) -> None:
        """加速度が安定するまで着地を待つ。"""
        self._set_phase("waiting_for_landing")
        while not judge_landing(
            self._sensors(),
            logger=self.logger,
            timeout_s=None,
        ):
            pass
        self._set_phase("landed")
        self._send_event("着地成功")

    def deploy(self) -> None:
        """溶断、姿勢復帰、着地点基準GNSS取得を行う。"""
        self._set_phase("deploying")
        time.sleep(float(self.config.LANDING_TO_FUSING_DELAY_S))

        if self.driver is None:
            self.driver = DriveController()
        driver = self._driver()
        navigator = self._navigator()

        fuse_and_kick(driver)
        navigator.restore_posture(driver, self._sensors())
        driver.stop()
        self._send_event("溶断・姿勢復帰成功")

        self.landing_reference_position = self._wait_for_gnss_fix("deploying")
        self.logger.event(
            "着地点基準GNSS取得 "
            f"(lat={self.landing_reference_position['latitude_deg']:.7f}, "
            f"lon={self.landing_reference_position['longitude_deg']:.7f})"
        )

    def start_wifi_ap(self) -> None:
        """自撮り用APと常時待受TCPサーバーを起動する。"""
        if self.selfie is None:
            self.selfie = SelfieManager(logger=self.logger)
        if self.selfie_wifi_started:
            return

        try:
            self.selfie.start_ap()
            self.selfie.start_server()
            self.selfie_wifi_started = True
            self.logger.event("自撮り用Wi-Fi AP起動完了")
        except Exception as exc:
            self.logger.event(
                f"自撮り用Wi-Fi AP起動失敗・ミッション続行 "
                f"({type(exc).__name__}: {exc})"
            )

    def avoid_parachute(self) -> None:
        """紫色パラシュートが前方から消えるまで回避する。"""
        self._set_phase("avoiding_parachute")
        while True:
            try:
                result = self._navigator().avoid_parachute(
                    self._driver(),
                    self._sensors(),
                )
            except Exception as exc:
                self._stop_driver()
                self.logger.event(
                    f"パラシュート回避エラー・再試行 "
                    f"({type(exc).__name__}: {exc})"
                )
                result = {}
            if result.get("completed"):
                self._send_event("パラシュート回避成功")
                return
            self.logger.event("パラシュート回避を再試行します")
            time.sleep(float(self.config.PARACHUTE_RETRY_DELAY_S))

    def clear_landing_area(self) -> None:
        """着地点基準から設定距離以上離れる。"""
        if self.landing_reference_position is None:
            raise RuntimeError("着地点基準GNSSがありません")

        self._set_phase("clearing_landing_area")
        landing_navigator = NavigationController(
            target_latitude_deg=self.landing_reference_position["latitude_deg"],
            target_longitude_deg=self.landing_reference_position["longitude_deg"],
        )

        while True:
            gnss = self._wait_for_gnss_fix("clearing_landing_area")
            latitude_deg = float(gnss["latitude_deg"])
            longitude_deg = float(gnss["longitude_deg"])
            distance_m = landing_navigator.distance_to_target_m(
                latitude_deg,
                longitude_deg,
            )
            self.logger.event(f"着地点基準からの距離: {distance_m:.1f} m")
            if distance_m >= float(self.config.LANDING_CLEARANCE_DISTANCE_M):
                self._send_event("着地点離脱成功")
                return

            navigator = self._navigator()
            target_bearing_deg = navigator.bearing_to_target(
                latitude_deg,
                longitude_deg,
            )
            current_heading_deg = float(self._sensors().get_heading_deg())
            turn_angle_deg = navigator.heading_error(
                target_bearing_deg,
                current_heading_deg,
            )
            navigator.rotate_by_angle(
                self._driver(),
                self._sensors(),
                turn_angle_deg,
            )
            navigator.follow_forward(
                self._driver(),
                self._sensors(),
                float(self.config.LANDING_CLEARANCE_MOVE_DURATION_S),
            )

    def run_selfie_mission(self) -> None:
        """自撮り、画像選択、無線送信を行い、失敗しても先へ進む。"""
        self._set_phase("selfie")
        captured_paths: list[Path] = []
        arm_expanded = False

        try:
            if not self.selfie_wifi_started:
                self.start_wifi_ap()
            if self.selfie is None:
                raise RuntimeError("自撮りカメラを開始できませんでした")

            self.selfie.ensure_connection()
            self.selfie.expand()
            arm_expanded = True
            try:
                captured_paths = self.selfie.capture_exposure_series()
            finally:
                if arm_expanded:
                    self.selfie.retract()
                    arm_expanded = False

            processor = ImageProcessor()
            selection = processor.select_best_selfie_image(captured_paths)
            selected_path = Path(selection["selected_path"])
            compressed_path = processor.compress_image(
                processor.load_image(selected_path),
                PROJECT_ROOT
                / "compressed_images"
                / f"{selected_path.stem}_compressed.jpg",
            )
            if self.telemetry is not None:
                self.telemetry.send_image(compressed_path)
            self.logger.event(
                f"自撮りミッション完了 ({compressed_path})"
            )
        except Exception as exc:
            self.logger.event(
                f"自撮りミッション失敗・GPS誘導へ続行 "
                f"({type(exc).__name__}: {exc})"
            )

    def navigate_to_goal_area(self) -> None:
        """GNSSゴール範囲へ到達するまで誘導を繰り返す。"""
        while True:
            self._set_phase("gnss_navigation")
            self._wait_for_gnss_fix("gnss_navigation")
            try:
                reached = self._navigator().follow_target(
                    self._driver(),
                    self._sensors(),
                    status_callback=self.logger.event,
                )
            except Exception as exc:
                self.logger.event(
                    f"GNSS誘導エラー・再試行 "
                    f"({type(exc).__name__}: {exc})"
                )
                reached = False
            finally:
                self._stop_driver()

            if reached:
                self._send_event("GNSS誘導成功")
                return
            self.logger.event("GNSS誘導を再試行します")
            time.sleep(float(self.config.GNSS_RETRY_INTERVAL_S))

    def search_and_guide_to_goal(self) -> None:
        """赤コーンを探索し、ゴール判定まで探索と誘導を繰り返す。"""
        while True:
            self._set_phase("searching_goal")
            self._wait_for_gnss_fix("searching_goal")
            try:
                search_result = search_around_gnss_goal(
                    self._navigator(),
                    self._driver(),
                    self._sensors(),
                    float(self.config.GOAL_SEARCH_DISTANCE_M),
                    float(self.config.GOAL_SEARCH_RED_RATIO_THRESHOLD),
                    status_callback=self.logger.event,
                )
            except Exception as exc:
                self._stop_driver()
                self.logger.event(
                    f"ゴール探索エラー・再試行 "
                    f"({type(exc).__name__}: {exc})"
                )
                search_result = {}

            if search_result.get("red_detected"):
                self._set_phase("guiding_to_goal")
                try:
                    guidance_result = guide_to_red_cone(
                        self._navigator(),
                        self._driver(),
                        self._sensors(),
                    )
                except Exception as exc:
                    self._stop_driver()
                    self.logger.event(
                        f"ゴール誘導エラー・再探索 "
                        f"({type(exc).__name__}: {exc})"
                    )
                    guidance_result = {}
                if guidance_result.get("goal_reached"):
                    self._send_event("ゴール判定成功")
                    return

            self.logger.event("ゴール探索をやり直します")
            self.navigate_to_goal_area()

    def complete(self) -> None:
        """モーターを停止し、ミッション完了を通知する。"""
        self._stop_driver()
        self._set_phase("completed")
        self.logger.event("Mission Complete")
        if self.telemetry is not None:
            self.telemetry.send_text("Mission Complete")

    def close(self) -> None:
        """使用した機器を終了する。"""
        self._stop_driver()
        if self.telemetry is not None:
            try:
                self.telemetry.stop()
            except Exception as exc:
                self.logger.event(
                    f"テレメトリ終了失敗 ({type(exc).__name__}: {exc})"
                )
        if self.selfie is not None:
            try:
                self.selfie.close_server()
                self.selfie.restore_wifi()
            except Exception as exc:
                self.logger.event(
                    f"自撮りカメラ終了失敗 ({type(exc).__name__}: {exc})"
                )
        if self.driver is not None:
            try:
                self.driver.cleanup()
            except Exception as exc:
                self.logger.event(
                    f"モーター終了失敗 ({type(exc).__name__}: {exc})"
                )
        if self.sensors is not None:
            try:
                self.sensors.close()
            except Exception as exc:
                self.logger.event(
                    f"センサ終了失敗 ({type(exc).__name__}: {exc})"
                )

    def _wait_for_gnss_fix(self, resume_phase: str) -> dict[str, Any]:
        """モーターを止め、GNSSが取得できるまで再試行する。"""
        self._stop_driver()
        failure_count = 0

        while True:
            try:
                gnss = self._sensors().get_gnss()
                latitude_deg = gnss.get("latitude_deg")
                longitude_deg = gnss.get("longitude_deg")
                if (
                    gnss.get("has_fix")
                    and latitude_deg is not None
                    and longitude_deg is not None
                ):
                    self._set_phase(resume_phase)
                    return gnss

                if gnss.get("raw"):
                    failure_count = 0
                    self._set_phase("waiting_for_gnss_fix")
                else:
                    failure_count += 1
            except Exception as exc:
                failure_count += 1
                self.logger.event(
                    f"GNSS取得失敗 ({type(exc).__name__}: {exc})"
                )

            if failure_count >= int(
                self.config.GNSS_REINITIALIZE_AFTER_FAILURES
            ):
                self._set_phase("recovering_gnss")
                try:
                    self._sensors().setup_gnss()
                except Exception as exc:
                    self.logger.event(
                        f"GNSS再初期化失敗 ({type(exc).__name__}: {exc})"
                    )
                failure_count = 0

            time.sleep(float(self.config.GNSS_RETRY_INTERVAL_S))

    def _set_phase(self, phase: str) -> None:
        if self.phase == phase:
            return
        self.phase = phase
        self.logger.event(f"Mission phase: {phase}")
        if self.telemetry is not None:
            self.telemetry.set_phase(phase)

    def _send_event(self, message: str) -> None:
        self.logger.event(message)

    def _stop_driver(self) -> None:
        if self.driver is not None:
            try:
                self.driver.stop()
            except Exception as exc:
                self.logger.event(
                    f"モーター停止失敗 ({type(exc).__name__}: {exc})"
                )

    def _sensors(self) -> SensorManager:
        if self.sensors is None:
            raise RuntimeError("SensorManager is not initialized")
        return self.sensors

    def _driver(self) -> DriveController:
        if self.driver is None:
            raise RuntimeError("DriveController is not initialized")
        return self.driver

    def _navigator(self) -> NavigationController:
        if self.navigator is None:
            raise RuntimeError("NavigationController is not initialized")
        return self.navigator
