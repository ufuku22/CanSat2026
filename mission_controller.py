#!/usr/bin/env python3
"""既存の各機能を能代・ARLISSミッションの順番につなぐ。"""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any

from config import MissionConfig
from communication_manager import CommunicationManager
from drive_controller import DriveController
from fusing import fuse_and_kick
from image_processor import ImageProcessor
from judge import judge_landing, judge_release, read_median_pressure_hpa
from logger import CsvLogger, get_mission_timestamp, Logger, PeriodicCsvLogger
from navigation_controller import NavigationController
from navigation_goal import (
    guide_to_center_of_zone,
    guide_to_red_ball,
    guide_to_red_cone,
    guide_to_square_zone,
    search_around_gnss_goal,
)
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
        history: PeriodicCsvLogger | None = None,
    ) -> None:
        timestamp = get_mission_timestamp()
        self.config = config
        self.logger = logger or Logger(
            log_dir=PROJECT_ROOT / "logs",
            filename=f"mission_{timestamp}_events.txt",
        )
        self.communication_log_path = (
            PROJECT_ROOT / "logs" / f"mission_{timestamp}_communication.txt"
        )
        self.communication_logger = Logger(
            log_dir=self.communication_log_path.parent,
            filename=self.communication_log_path.name,
        )
        self.history_path = PROJECT_ROOT / "logs" / f"mission_{timestamp}_history.csv"
        self.sensors = sensors
        self.driver = driver
        self.navigator = navigator
        if self.navigator is not None:
            self.navigator.logger = self.logger
        self.telemetry = telemetry
        self.selfie = selfie
        self.history = history
        self._owns_sensors = sensors is None
        self.selfie_wifi_started = False
        self.phase = "startup"
        self.ground_pressure_hpa: float | None = None

    def __enter__(self) -> "MissionController":
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        try:
            if exc is not None:
                self.logger.event(
                    f"ミッション異常終了 ({type(exc).__name__}: {exc})"
                )
        finally:
            self.close()

    def prepare(self) -> None:
        """センサと基準気圧を準備する。"""
        self._set_phase("preparing")
        use_distance_sensor = bool(
            getattr(self.config, "USE_DISTANCE_SENSOR", False)
        )
        while True:
            try:
                if self.sensors is None:
                    self.sensors = SensorManager(
                        status_callback=self.logger.event,
                    )
                self.sensors.setup(enable_distance_sensor=use_distance_sensor)
                self.sensors.set_gnss_cache_max_age_s(
                    self.config.GNSS_CACHE_MAX_AGE_S
                )
                self.ground_pressure_hpa = read_median_pressure_hpa(
                    self.sensors,
                    logger=self.logger,
                )
                break
            except Exception as exc:
                self.logger.event(
                    f"センサ初期化・基準気圧取得失敗、再試行 "
                    f"({type(exc).__name__}: {exc})"
                )
                if self._owns_sensors and self.sensors is not None:
                    try:
                        self.sensors.close()
                    except Exception as close_exc:
                        self.logger.event(
                            "センサ再生成前の終了処理失敗 "
                            f"({type(close_exc).__name__}: {close_exc})"
                        )
                    self.sensors = None
                time.sleep(float(self.config.GNSS_RETRY_INTERVAL_S))

        if self.navigator is None:
            self.navigator = NavigationController(logger=self.logger)

        if self.telemetry is None:
            try:
                communication = CommunicationManager(logger=self.communication_logger)
                communication.setup()
                self.telemetry = TelemetryService(
                    self.sensors,
                    self.logger,
                    interval_s=self.config.TELEMETRY_INTERVAL_S,
                    communication=communication,
                    communication_logger=self.communication_logger,
                )
                self.telemetry.set_phase(self.phase)
                self.telemetry.send_once()
            except (Exception, SystemExit) as exc:
                self.logger.event(
                    f"テレメトリ準備失敗・ミッション続行 "
                    f"({type(exc).__name__}: {exc})"
                )
        if self.history is None:
            csv_logger = CsvLogger(
                self.sensors,
                self.history_path,
                record_fields=(
                    "phase",
                    *CsvLogger.GNSS_FIELDS,
                    "target_latitude_deg",
                    "target_longitude_deg",
                    "distance_to_target_m",
                    "target_heading_deg",
                    *CsvLogger.ENVIRONMENT_FIELDS,
                    *CsvLogger.IMU_FIELDS,
                    "heading_error_deg",
                    "control_mode",
                    "left_motor_command_percent",
                    "right_motor_command_percent",
                    *(
                        CsvLogger.DISTANCE_FIELDS
                        if use_distance_sensor
                        else ()
                    ),
                ),
                start_time=self.logger.start_time,
            )
            self.history = PeriodicCsvLogger(
                csv_logger,
                interval_s=float(self.config.CONTROL_LOG_INTERVAL_S),
                values_provider=self._control_history_values,
                status_callback=self.logger.event,
            )
        self.history.start()
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
            self.logger.event("テレメトリを利用できないため、送信なしで続行します")
            return
        try:
            self.telemetry.set_phase(self.phase)
            self.telemetry.start()
        except (Exception, SystemExit) as exc:
            self._disable_telemetry("テレメトリ開始失敗・ミッション続行", exc)

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
        """着地点基準GNSS取得、溶断、直進、姿勢復帰を行う。"""
        self._set_phase("deploying")

        landing_position = self._wait_for_gnss_fix("deploying")
        self.logger.event(
            "着地点基準GNSS取得 "
            f"(lat={landing_position['latitude_deg']:.7f}, "
            f"lon={landing_position['longitude_deg']:.7f})"
        )
        time.sleep(float(self.config.LANDING_TO_FUSING_DELAY_S))

        if self.driver is None:
            self.driver = DriveController()
        driver = self._driver()
        navigator = self._navigator()

        fuse_and_kick(driver)
        navigator.pd_forward(
            driver,
            self._sensors(),
            0.2,
            base_speed=60.0,
        )

        parachute_detected = navigator.detect_parachute(self._sensors())
        if parachute_detected:
            self.logger.event("展開: 前方に紫色を検知。100%で10秒間後退します")
            try:
                driver.drive(-100.0)
                time.sleep(10.0)
            finally:
                driver.stop()
        else:
            self.logger.event("展開: 前方に紫色なし。100%で10秒間前進します")
            navigator.pd_forward(
                driver,
                self._sensors(),
                10.0,
                base_speed=100.0,
            )
        try:
            posture_restored = navigator.restore_posture(driver, self._sensors())
        except Exception as exc:
            self.logger.event(
                f"姿勢復帰失敗・ミッション続行 "
                f"({type(exc).__name__}: {exc})"
            )
        else:
            if posture_restored:
                self._send_event("溶断・姿勢復帰成功")
            else:
                self.logger.event(
                    "規定回数内に姿勢復帰を確認できませんでした・ミッション続行"
                )
        finally:
            self._stop_driver()

    def start_wifi_ap(self) -> None:
        """自撮り用APと常時待受TCPサーバーを起動する。"""
        if self.selfie_wifi_started:
            return

        try:
            if self.selfie is None:
                self.selfie = SelfieManager(logger=self.logger)
            self.selfie.start_ap()
            self.selfie.start_server()
            self.selfie_wifi_started = True
            self.logger.event("自撮り用Wi-Fi AP起動完了")
        except (Exception, SystemExit) as exc:
            self.logger.event(
                f"自撮り用Wi-Fi AP起動失敗・ミッション続行 "
                f"({type(exc).__name__}: {exc})"
            )

    def clear_landing_area(self) -> None:
        """目標方位を向き、前方から紫色がなくなるまでパラシュートを避ける。"""
        self._set_phase("clearing_landing_area")

        while True:
            gnss = self._wait_for_gnss_fix("clearing_landing_area")
            latitude_deg = float(gnss["latitude_deg"])
            longitude_deg = float(gnss["longitude_deg"])
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
            if not navigator.avoid_parachute(
                self._driver(),
                self._sensors(),
            ):
                self._send_event("パラシュート回避完了")
                return

    def run_selfie_mission(self) -> None:
        """自撮り、画像選択、無線送信を行い、失敗しても先へ進む。"""
        self._set_phase("selfie")
        captured_paths: list[Path] = []
        arm_expanded = False

        try:
            self._navigator().restore_posture(self._driver(), self._sensors())

            if not self.selfie_wifi_started:
                self.start_wifi_ap()
            if not self.selfie_wifi_started or self.selfie is None:
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

            processor = ImageProcessor(logger=self.logger)
            selection = processor.select_best_selfie_image(captured_paths)
            for evaluation in selection["evaluations"]:
                if not evaluation["is_valid"]:
                    self.logger.event(
                        "自撮り画像判定失敗 "
                        f"(path={evaluation['path']}, error={evaluation['error']})"
                    )
                    continue
                self.logger.event(
                    "自撮り画像判定 "
                    f"(path={evaluation['path']}, "
                    f"marker_detected={evaluation['aruco_detected']}, "
                    f"marker_id={evaluation['marker_id']}, "
                    f"capture_ok={evaluation['capture_ok']}, "
                    f"capture_reason={evaluation['capture_reason']}, "
                    f"sharpness={evaluation['sharpness']:.2f}, "
                    f"blurry={evaluation['is_blurry']}, "
                    f"white_clipping={evaluation['white_clipping_ratio']:.4f}, "
                    f"black_crush={evaluation['black_crush_ratio']:.4f}, "
                    f"candidate={evaluation['is_candidate']})"
                )
            selected_path = Path(selection["selected_path"])
            self.logger.event(
                "自撮り画像選択 "
                f"(selected={selected_path}, "
                f"candidates={selection['candidate_count']}, "
                "capture_ok_filter="
                f"{selection['capture_ok_filter_applied']}, "
                f"marker_filter={selection['aruco_filter_applied']})"
            )
            compressed_path = processor.compress_image(
                processor.load_image(selected_path),
                PROJECT_ROOT
                / "compressed_images"
                / f"{selected_path.stem}_compressed.jpg",
            )
            if self.telemetry is not None:
                self.telemetry.send_image(compressed_path)
            if len(captured_paths) != 5:
                raise RuntimeError(
                    f"自撮り画像の受信は{len(captured_paths)}/5枚"
                    f"でしたが、受信済み画像の送信は完了しました ({compressed_path})"
                )
            self.logger.event(
                f"自撮りミッション完了 ({compressed_path})"
            )
        except (Exception, SystemExit) as exc:
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

    def search_for_goal(self, *, relocate_before_search: bool = False) -> None:
        """GNSSゴール周辺で赤いゴールを発見するまで探索する。"""
        while True:
            if self._search_for_red_goal_target(
                relocate_before_search=relocate_before_search,
            ):
                self._send_event("赤いゴールを発見")
                return

            self.logger.event("ゴール探索をやり直します")
            self.navigate_to_goal_area()

    def guide_to_red_cone_goal(self) -> None:
        """発見済みの赤コーンへ誘導し、能代のゴールを判定する。"""
        max_attempts = int(self.config.GOAL_GUIDANCE_MAX_ATTEMPTS)
        if max_attempts <= 0:
            raise ValueError("GOAL_GUIDANCE_MAX_ATTEMPTS must be greater than 0")

        last_error: Exception | None = None
        for attempt in range(1, max_attempts + 1):
            self._set_phase("guiding_to_goal")
            goal_reached = False
            try:
                guidance_result = guide_to_red_cone(
                    self._navigator(),
                    self._driver(),
                    self._sensors(),
                    logger=self.logger,
                )
                goal_reached = bool(guidance_result.get("goal_reached"))
                if not goal_reached:
                    last_error = RuntimeError(
                        "赤コーンのゴール判定に失敗しました "
                        f"({guidance_result.get('reason')})"
                    )
            except Exception as exc:
                last_error = exc
            finally:
                self._stop_driver()

            if goal_reached:
                self._send_event("ゴール判定成功")
                return

            self.logger.event(
                f"ゴール誘導失敗 ({attempt}/{max_attempts}, "
                f"{type(last_error).__name__}: {last_error})"
            )
            if attempt < max_attempts:
                self.logger.event(
                    "ランダム探索地点への移動からゴール誘導をやり直します"
                )
                self.search_for_goal(relocate_before_search=True)

        raise RuntimeError(
            f"赤コーンのゴール誘導に{max_attempts}回失敗しました"
        ) from last_error

    def guide_to_arliss_goal(self) -> None:
        """発見済みの4つの赤ボールの中心へ誘導する。"""
        max_attempts = int(self.config.GOAL_GUIDANCE_MAX_ATTEMPTS)
        if max_attempts <= 0:
            raise ValueError("GOAL_GUIDANCE_MAX_ATTEMPTS must be greater than 0")

        last_error: Exception | None = None
        for attempt in range(1, max_attempts + 1):
            self._set_phase("guiding_to_goal")
            goal_reached = False
            try:
                goal_reached = self._guide_to_arliss_goal_once()
                if not goal_reached:
                    last_error = RuntimeError("ARLISSゴール誘導に失敗しました")
            except Exception as exc:
                last_error = exc
            finally:
                self._stop_driver()

            if goal_reached:
                self._send_event("ARLISSゴール判定成功")
                return

            self.logger.event(
                f"ARLISSゴール誘導失敗 ({attempt}/{max_attempts}, "
                f"{type(last_error).__name__}: {last_error})"
            )
            if attempt < max_attempts:
                self.logger.event(
                    "ランダム探索地点への移動からARLISSゴール誘導をやり直します"
                )
                self.search_for_goal(relocate_before_search=True)

        raise RuntimeError(
            f"ARLISSゴール誘導に{max_attempts}回失敗しました"
        ) from last_error

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
        if self.history is not None:
            try:
                self.history.stop()
            except Exception as exc:
                self.logger.event(
                    f"制御履歴終了失敗 ({type(exc).__name__}: {exc})"
                )
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
        reinitialize_timeout_s = float(
            self.config.GNSS_REINITIALIZE_NO_FIX_TIMEOUT_S
        )
        if reinitialize_timeout_s <= 0.0:
            raise ValueError(
                "GNSS_REINITIALIZE_NO_FIX_TIMEOUT_S must be greater than 0"
            )
        no_fix_since = time.monotonic()

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
                    self._set_phase("waiting_for_gnss_fix")
            except Exception as exc:
                self.logger.event(
                    f"GNSS取得失敗 ({type(exc).__name__}: {exc})"
                )

            if time.monotonic() - no_fix_since >= reinitialize_timeout_s:
                self._set_phase("recovering_gnss")
                try:
                    self._sensors().setup_gnss()
                except Exception as exc:
                    self.logger.event(
                        f"GNSS再初期化失敗 ({type(exc).__name__}: {exc})"
                    )
                no_fix_since = time.monotonic()

            time.sleep(float(self.config.GNSS_RETRY_INTERVAL_S))

    def _search_for_red_goal_target(
        self,
        *,
        relocate_before_search: bool = False,
    ) -> bool:
        """GNSSゴール周辺で360度走査と移動後の再走査を行う。"""
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
                relocate_before_scan=relocate_before_search,
                logger=self.logger,
            )
        except Exception as exc:
            self._stop_driver()
            self.logger.event(
                f"ゴール探索エラー・再試行 "
                f"({type(exc).__name__}: {exc})"
            )
            return False
        return bool(search_result.get("red_detected"))

    def _guide_to_arliss_goal_once(self) -> bool:
        """最初の赤ボールからスクエアゾーン中心まで誘導する。"""
        first_ball_result = guide_to_red_ball(
            self._navigator(),
            self._driver(),
            self._sensors(),
            logger=self.logger,
        )
        if not first_ball_result.get("target_reached"):
            self.logger.event(
                "ARLISS最初の赤ボール誘導失敗 "
                f"({first_ball_result.get('reason')})"
            )
            return False

        square_result = guide_to_square_zone(
            self._navigator(),
            self._driver(),
            self._sensors(),
            initial_ball_position=first_ball_result.get(
                "initial_ball_position"
            ),
            logger=self.logger,
        )
        if not square_result.get("square_zone_reached"):
            self.logger.event(
                "ARLISSスクエアゾーン誘導失敗 "
                f"({square_result.get('reason')})"
            )
            return False

        center_result = guide_to_center_of_zone(
            self._navigator(),
            self._driver(),
            self._sensors(),
            logger=self.logger,
        )
        if center_result.get("center_reached"):
            return True
        self.logger.event(
            "ARLISSスクエアゾーン中心誘導失敗 "
            f"({center_result.get('reason')})"
        )
        return False

    def _set_phase(self, phase: str) -> None:
        if self.phase == phase:
            return
        self.phase = phase
        self.logger.event(f"Mission phase: {phase}")
        if self.telemetry is not None:
            try:
                self.telemetry.set_phase(phase)
            except (Exception, SystemExit) as exc:
                self._disable_telemetry(
                    "テレメトリ状態更新失敗・ミッション続行",
                    exc,
                )

    def _send_event(self, message: str) -> None:
        self.logger.event(message)

    def _control_history_values(self, row: dict[str, Any]) -> dict[str, Any]:
        """同じ時刻のミッション状態、目標値、モーター指令を返す。"""
        values: dict[str, Any] = {"phase": self.phase}
        navigator = self.navigator
        if navigator is not None:
            values["target_latitude_deg"] = navigator.target_latitude_deg
            values["target_longitude_deg"] = navigator.target_longitude_deg

            latitude = row.get("latitude_deg")
            longitude = row.get("longitude_deg")
            if latitude != "" and longitude != "":
                distance_m = navigator.distance_to_target_m(
                    float(latitude),
                    float(longitude),
                )
                target_heading_deg = navigator.bearing_to_target(
                    float(latitude),
                    float(longitude),
                )
                values["distance_to_target_m"] = distance_m

                if self.phase in {"clearing_landing_area", "gnss_navigation"}:
                    values["target_heading_deg"] = target_heading_deg
                    heading_deg = row.get("heading_deg")
                    if heading_deg != "":
                        values["heading_error_deg"] = navigator.heading_error(
                            float(heading_deg),
                            target_heading_deg,
                        )

        if self.driver is not None:
            left_command, right_command = self.driver.get_motor_commands()
            values["left_motor_command_percent"] = left_command
            values["right_motor_command_percent"] = right_command
            values["control_mode"] = self._motor_control_mode(
                left_command,
                right_command,
            )

        return values

    @staticmethod
    def _motor_control_mode(left_command: float, right_command: float) -> str:
        if left_command == 0.0 and right_command == 0.0:
            return "stopped"
        if left_command >= 0.0 and right_command >= 0.0:
            return "forward"
        if left_command <= 0.0 and right_command <= 0.0:
            return "reverse"
        if left_command > 0.0 and right_command < 0.0:
            return "turn_right"
        if left_command < 0.0 and right_command > 0.0:
            return "turn_left"
        return "mixed"

    def _stop_driver(self) -> None:
        if self.driver is not None:
            try:
                self.driver.stop()
            except Exception as exc:
                self.logger.event(
                    f"モーター停止失敗 ({type(exc).__name__}: {exc})"
                )

    def _disable_telemetry(self, message: str, exc: BaseException) -> None:
        telemetry = self.telemetry
        self.telemetry = None
        self.logger.event(f"{message} ({type(exc).__name__}: {exc})")
        if telemetry is not None:
            try:
                telemetry.stop()
            except (Exception, SystemExit) as stop_exc:
                self.logger.event(
                    f"テレメトリ終了処理失敗・ミッション続行 "
                    f"({type(stop_exc).__name__}: {stop_exc})"
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
