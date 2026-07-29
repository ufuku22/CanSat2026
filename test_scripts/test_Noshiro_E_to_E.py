#!/usr/bin/env python3
"""能代で使用するCanSatのEnd-to-End試験シーケンス。"""

from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from communication_manager import CommunicationManager
from config import RedConeConfig
from drive_controller import DriveController
from fusing import fuse_and_kick
from image_judge import ImageJudge
from image_processor import ImageProcessor
from judge import judge_landing
from logger import GnssNavigationCsvLogger, Logger
from navigation_controller import NavigationController
from navigation_goal import guide_to_red_cone, search_around_gnss_goal
from selfie_manager import SelfieManager
from sensor_manager import SensorManager
from test_scripts.gps_pd_navigation_log_test import LoggingNavigationController


LANDING_TO_FUSING_DELAY_SECONDS = 10.0
PARACHUTE_MOVE_DURATION_SECONDS = 10.0
PARACHUTE_AVOIDANCE_MAX_ATTEMPTS = 3
SELFIE_CAPTURE_COUNT = 5
LANDING_SUCCESS_SEND_COUNT = 10
OTHER_SUCCESS_SEND_COUNT = 3

# ⑥の探索条件。実機試験の探索範囲とカメラ条件に合わせて変更する。
GNSS_GOAL_SEARCH_DISTANCE_M = 10.0
GNSS_GOAL_SEARCH_RED_RATIO_THRESHOLD = RedConeConfig.RED_THRESHOLD
GNSS_GOAL_SEARCH_MAX_ATTEMPTS = 20

# 実機試験前に、ゴールとして使用するGNSS座標へ書き換える。
GNSS_GOAL_LATITUDE_DEG: float | None = None
GNSS_GOAL_LONGITUDE_DEG: float | None = None


def send_success_notification(
    sensors: SensorManager,
    logger: Logger,
    success_message: str,
    *,
    repeat_count: int = OTHER_SUCCESS_SEND_COUNT,
) -> bool:
    """成功メッセージと送信時点のGNSS座標をPCへ指定回数送信する。"""
    repeat_count = int(repeat_count)
    if repeat_count < 1:
        raise ValueError("repeat_count must be at least 1")

    try:
        gnss = sensors.get_gnss()
    except Exception as exc:
        logger.event(
            "PC成功通知: GNSS座標取得失敗 "
            f"({type(exc).__name__}: {exc})"
        )
        gnss = {}

    latitude_deg = gnss.get("latitude_deg")
    longitude_deg = gnss.get("longitude_deg")
    latitude_text = (
        "unknown"
        if latitude_deg is None
        else f"{float(latitude_deg):.7f}"
    )
    longitude_text = (
        "unknown"
        if longitude_deg is None
        else f"{float(longitude_deg):.7f}"
    )
    has_fix = bool(gnss.get("has_fix"))
    message = (
        f"{success_message} "
        f"lat={latitude_text} lon={longitude_text} fix={int(has_fix)}"
    )

    successful_send_count = 0
    try:
        with CommunicationManager(logger=logger) as communication:
            for send_index in range(1, repeat_count + 1):
                try:
                    response = communication.send_text(message)
                except Exception as exc:
                    logger.event(
                        "PC成功通知: 送信失敗 "
                        f"({success_message}, "
                        f"{send_index}/{repeat_count}, "
                        f"{type(exc).__name__}: {exc})"
                    )
                    continue

                if "radio_tx_ok" in response:
                    successful_send_count += 1
                else:
                    logger.event(
                        "PC成功通知: radio_tx_ok未確認 "
                        f"({success_message}, "
                        f"{send_index}/{repeat_count})"
                    )
    except Exception as exc:
        logger.event(
            "PC成功通知: 通信開始失敗 "
            f"({success_message}, {type(exc).__name__}: {exc})"
        )

    all_succeeded = successful_send_count == repeat_count
    logger.event(
        "PC成功通知: 送信結果 "
        f"({success_message}, "
        f"radio_tx_ok={successful_send_count}/{repeat_count}, "
        f"lat={latitude_text}, lon={longitude_text}, fix={has_fix})"
    )
    return all_succeeded


def run_step_1_landing(
    sensors: SensorManager,
    logger: Logger,
) -> bool:
    """① 加速度が安定するまで待ち、着地を判定する。"""
    logger.event("E2E ①: 着地判定シーケンス開始")

    # drop_sequence_test.pyと同じ着地判定メソッド・引数を使用する。
    landed = judge_landing(
        sensors,
        logger=logger,
        timeout_s=None,
    )
    if not landed:
        logger.event("E2E ①: 着地判定失敗")
        return False

    logger.event("E2E ①: 着地判定成功")
    return True


def run_step_2_fusing(
    driver: DriveController,
    logger: Logger,
) -> bool:
    """② 溶断後にモーターを一瞬動かしてキャリアを展開する。"""
    logger.event("E2E ②: 溶断・キャリア展開開始")

    try:
        # drop_sequence_test.pyと同じ溶断・モーターパルスを使用する。
        fuse_and_kick(driver, pulse_time=0.5)
    except Exception as exc:
        logger.event(
            "E2E ②: 溶断・キャリア展開失敗 "
            f"({type(exc).__name__}: {exc})"
        )
        return False

    logger.event("E2E ②: 溶断成功・キャリア展開完了")
    return True


def run_step_3_parachute_avoidance(
    navigator: NavigationController,
    driver: DriveController,
    sensors: SensorManager,
    logger: Logger,
    *,
    move_duration_s: float = PARACHUTE_MOVE_DURATION_SECONDS,
    max_attempts: int = PARACHUTE_AVOIDANCE_MAX_ATTEMPTS,
    processor: ImageProcessor | None = None,
) -> bool:
    """③ GNSS目標方向を確認しながら紫色パラシュートを回避する。"""
    move_duration_s = float(move_duration_s)
    max_attempts = int(max_attempts)
    if move_duration_s <= 0.0:
        raise ValueError("move_duration_s must be greater than 0")
    if max_attempts < 1:
        raise ValueError("max_attempts must be at least 1")
    if processor is None:
        processor = ImageProcessor()

    config = navigator.parachute_avoidance_config
    logger.event(
        "E2E ③: パラシュート回避開始 "
        f"(直進時間={move_duration_s:.1f}秒, 最大確認回数={max_attempts})"
    )

    for attempt in range(1, max_attempts + 1):
        # 現在のGNSS座標からゴール座標への目標方位を計算する。
        gnss = sensors.get_gnss()
        latitude_deg = gnss.get("latitude_deg")
        longitude_deg = gnss.get("longitude_deg")
        if (
            not gnss.get("has_fix")
            or latitude_deg is None
            or longitude_deg is None
        ):
            logger.event(
                f"E2E ③: GNSS目標方位取得失敗 ({attempt}/{max_attempts})"
            )
            driver.stop()
            return False

        target_bearing_deg = navigator.bearing_to_target(
            float(latitude_deg),
            float(longitude_deg),
        )
        current_heading_deg = float(sensors.get_heading_deg())
        turn_to_target_deg = navigator.heading_error(
            target_bearing_deg,
            current_heading_deg,
        )
        logger.event(
            f"E2E ③: 目標方向確認 {attempt}/{max_attempts} "
            f"(lat={float(latitude_deg):.7f}, "
            f"lon={float(longitude_deg):.7f}, "
            f"現在方位={current_heading_deg:.1f}度, "
            f"目標方位={target_bearing_deg:.1f}度, "
            f"旋回角={turn_to_target_deg:.1f}度)"
        )

        face_target_result = navigator.rotate_by_angle(
            driver,
            sensors,
            turn_to_target_deg,
            speed=config.ROTATE_SPEED,
            tolerance_deg=config.ROTATE_TOLERANCE_DEG,
            timeout_s=config.ROTATE_TIMEOUT_S,
        )
        if not face_target_result["reached"]:
            logger.event(
                "E2E ③: GNSS目標方向への旋回失敗 "
                f"({attempt}/{max_attempts})"
            )
            return False

        # 目標方向を向いた状態で前方カメラから紫色を確認する。
        frame = sensors.capture_front_frame()
        purple_result = processor.detect_color(
            frame,
            hsv_ranges=processor.PURPLE_HSV_RANGES,
            color_threshold=config.PURPLE_THRESHOLD,
        )
        purple_result.pop("color_mask", None)
        purple_detected = bool(purple_result["is_color_detected"])
        purple_ratio = float(purple_result["total_color_ratio"])
        logger.event(
            f"E2E ③: 紫色確認 {attempt}/{max_attempts} "
            f"(detected={purple_detected}, ratio={purple_ratio:.4f}, "
            f"threshold={float(config.PURPLE_THRESHOLD):.4f})"
        )

        if not purple_detected:
            logger.event(
                "E2E ③: パラシュートなし。"
                f"GNSS目標方位を維持して{move_duration_s:.1f}秒直進"
            )
            navigator.follow_forward(
                driver,
                sensors,
                move_duration_s,
                base_speed=config.MOVE_SPEED,
            )
            logger.event("E2E ③: パラシュート回避成功")
            return True

        logger.event(
            "E2E ③: パラシュート検知。"
            f"{float(config.ROTATE_ANGLE_DEG):.1f}度右旋回"
        )
        avoidance_turn_result = navigator.rotate_by_angle(
            driver,
            sensors,
            config.ROTATE_ANGLE_DEG,
            speed=config.ROTATE_SPEED,
            tolerance_deg=config.ROTATE_TOLERANCE_DEG,
            timeout_s=config.ROTATE_TIMEOUT_S,
        )
        if not avoidance_turn_result["reached"]:
            logger.event(
                f"E2E ③: パラシュート回避旋回失敗 ({attempt}/{max_attempts})"
            )
            return False

        logger.event(
            "E2E ③: 回避後の方位を維持して"
            f"{move_duration_s:.1f}秒直進"
        )
        navigator.follow_forward(
            driver,
            sensors,
            move_duration_s,
            base_speed=config.MOVE_SPEED,
        )

    logger.event(
        "E2E ③: パラシュート回避失敗 "
        f"({max_attempts}回の確認で紫色が消えませんでした)"
    )
    return False


def run_step_4_selfie(
    logger: Logger,
    *,
    sensors: SensorManager | None = None,
    capture_count: int = SELFIE_CAPTURE_COUNT,
) -> bool:
    """④ アームを展開して5枚撮影し、最高評価の1枚だけをPCへ送信する。"""
    capture_count = int(capture_count)
    if capture_count != SELFIE_CAPTURE_COUNT:
        raise ValueError(
            f"capture_count must be exactly {SELFIE_CAPTURE_COUNT}"
        )

    logger.event(f"E2E ④: 自撮りシーケンス開始 ({capture_count}枚撮影)")
    selfie = SelfieManager(logger=logger)
    arm_expanded = False
    captured_paths: list[Path] = []

    try:
        # test_selfie_full_flow.pyと同じAP起動・ESP32S3同期を行う。
        with selfie:
            try:
                logger.event("E2E ④: AP起動開始")
                selfie.start_ap()
                logger.event("E2E ④: ESP32S3同期開始")
                selfie.wait_connection()
                logger.event("E2E ④: ESP32S3同期成功")

                selfie.expand()
                arm_expanded = True
                logger.event("E2E ④: アーム展開成功")
                if sensors is not None:
                    send_success_notification(
                        sensors,
                        logger,
                        "アーム展開成功",
                    )

                for capture_index in range(1, capture_count + 1):
                    captured_path = selfie.capture_connected()
                    captured_paths.append(Path(captured_path))
                    logger.event(
                        f"E2E ④: 撮影 {capture_index}/{capture_count} "
                        f"保存完了 ({captured_path})"
                    )
                logger.event(f"E2E ④: 撮影成功 ({capture_count}枚)")
                if sensors is not None:
                    send_success_notification(
                        sensors,
                        logger,
                        "撮影成功",
                    )

                # 従来フローと同様、撮影後は先にアームを収納する。
                selfie.retract()
                arm_expanded = False
                logger.event("E2E ④: アーム収納成功")
                if sensors is not None:
                    send_success_notification(
                        sensors,
                        logger,
                        "アーム収納成功",
                    )
            except BaseException:
                if arm_expanded:
                    try:
                        selfie.retract()
                        arm_expanded = False
                        logger.event("E2E ④: エラー後アーム収納成功")
                    except Exception as retract_exc:
                        logger.event(
                            "E2E ④: エラー後アーム収納失敗 "
                            f"({type(retract_exc).__name__}: {retract_exc})"
                        )
                raise

        selection = ImageJudge().select_best_image(captured_paths)
        selected_path = Path(selection["selected_path"])
        if selected_path not in captured_paths:
            raise RuntimeError(
                "ImageJudge selected a path outside the five captured images"
            )
        logger.event(
            "E2E ④: 最高評価画像選定完了 "
            f"(selected={selected_path}, "
            f"candidates={selection['candidate_count']}, "
            f"aruco_filter={selection['aruco_filter_applied']})"
        )

        # 選定された1枚だけをTLM922S P2PでPC側へ送信する。
        with CommunicationManager(logger=logger) as communication:
            send_result = communication.send_image(selected_path)
        if not send_result.all_radio_tx_ok:
            logger.event(
                "E2E ④: 送信失敗 "
                f"(radio_tx_ok={send_result.radio_tx_ok_count}/"
                f"{len(send_result.responses)})"
            )
            return False

        logger.event(
            "E2E ④: 送信成功 "
            f"(selected={selected_path}, "
            f"radio_tx_ok={send_result.radio_tx_ok_count}/"
            f"{len(send_result.responses)})"
        )
        if sensors is not None:
            send_success_notification(
                sensors,
                logger,
                "送信成功",
            )
        return True
    except Exception as exc:
        logger.event(
            f"E2E ④: 自撮りシーケンス失敗 ({type(exc).__name__}: {exc})"
        )
        return False


def _label_navigation_csv_points(
    csv_path: Path,
    *,
    goal_reached: bool,
) -> None:
    """GNSS誘導CSVへ開始・走行中・終了地点の種別を追加する。"""
    with csv_path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)

    if not rows:
        raise RuntimeError("GNSS navigation CSV contains no position rows")

    for row_index, row in enumerate(rows):
        if row_index == 0 and row_index == len(rows) - 1:
            point_type = (
                "navigation_start_and_goal"
                if goal_reached
                else "navigation_start_and_failure"
            )
        elif row_index == 0:
            point_type = "navigation_start"
        elif row_index == len(rows) - 1:
            point_type = "goal_reached" if goal_reached else "navigation_failure"
        else:
            point_type = "navigation"
        row["point_type"] = point_type

    output_fieldnames = ["point_type", *fieldnames]
    temporary_path = csv_path.with_suffix(csv_path.suffix + ".tmp")
    with temporary_path.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=output_fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    temporary_path.replace(csv_path)


def run_step_5_gnss_navigation(
    driver: DriveController,
    sensors: SensorManager,
    logger: Logger,
    csv_path: Path,
    *,
    target_latitude_deg: float | None = GNSS_GOAL_LATITUDE_DEG,
    target_longitude_deg: float | None = GNSS_GOAL_LONGITUDE_DEG,
) -> bool:
    """⑤ 固定GNSS目標までPD誘導し、開始からゴールまでCSV保存する。"""
    if target_latitude_deg is None or target_longitude_deg is None:
        raise ValueError(
            "Set GNSS_GOAL_LATITUDE_DEG and GNSS_GOAL_LONGITUDE_DEG "
            "before running the E2E test"
        )
    target_latitude_deg = float(target_latitude_deg)
    target_longitude_deg = float(target_longitude_deg)
    if not -90.0 <= target_latitude_deg <= 90.0:
        raise ValueError("target_latitude_deg must be between -90 and 90")
    if not -180.0 <= target_longitude_deg <= 180.0:
        raise ValueError("target_longitude_deg must be between -180 and 180")

    csv_path = Path(csv_path)
    navigator = LoggingNavigationController(
        target_latitude_deg=target_latitude_deg,
        target_longitude_deg=target_longitude_deg,
    )
    logger.event(
        "E2E ⑤: GNSS誘導開始 "
        f"(target_lat={target_latitude_deg:.7f}, "
        f"target_lon={target_longitude_deg:.7f}, csv={csv_path})"
    )

    reached_goal = False
    with GnssNavigationCsvLogger(
        sensors,
        csv_path,
        goal_latitude_deg=target_latitude_deg,
        goal_longitude_deg=target_longitude_deg,
    ) as logged_sensors:
        # follow_target()の前に開始地点を明示的にCSVへ保存する。
        start_gnss = logged_sensors.get_gnss()
        start_latitude_deg = start_gnss.get("latitude_deg")
        start_longitude_deg = start_gnss.get("longitude_deg")
        if (
            not start_gnss.get("has_fix")
            or start_latitude_deg is None
            or start_longitude_deg is None
        ):
            logged_sensors.discard_pending_sample()
            logger.event("E2E ⑤: GNSS誘導開始地点を取得できませんでした")
            return False

        start_latitude_deg = float(start_latitude_deg)
        start_longitude_deg = float(start_longitude_deg)
        start_distance_m = navigator.distance_to_target_m(
            start_latitude_deg,
            start_longitude_deg,
        )
        start_heading_deg = float(logged_sensors.get_heading_deg())
        logged_sensors.record_navigation(
            distance_to_goal_m=start_distance_m,
            heading_deg=start_heading_deg,
        )
        logger.event(
            "E2E ⑤: GNSS誘導開始地点 "
            f"(lat={start_latitude_deg:.7f}, "
            f"lon={start_longitude_deg:.7f}, "
            f"distance={start_distance_m:.2f}m, "
            f"heading={start_heading_deg:.1f}度)"
        )

        def log_navigation_status(message: str) -> None:
            logger.event(f"E2E ⑤: {message}")

        def avoid_stuck_during_navigation() -> bool:
            return navigator.avoid_stuck(driver, logged_sensors)

        reached_goal = navigator.follow_target(
            driver,
            logged_sensors,
            status_callback=log_navigation_status,
            stuck_avoidance_callback=avoid_stuck_during_navigation,
        )

        # ゴール判定に使われた未記録GNSSを最後の行として保存する。
        final_sample = logged_sensors._pending_sample
        if final_sample is None:
            logged_sensors.get_gnss()
            final_sample = logged_sensors._pending_sample
        if final_sample is None:
            logger.event("E2E ⑤: GNSS誘導終了地点を取得できませんでした")
            return False

        final_distance_m = navigator.distance_to_target_m(
            final_sample["latitude_deg"],
            final_sample["longitude_deg"],
        )
        final_heading_deg = float(logged_sensors.get_heading_deg())
        logged_sensors.record_navigation(
            distance_to_goal_m=final_distance_m,
            heading_deg=final_heading_deg,
        )
        final_latitude_deg = float(final_sample["latitude_deg"])
        final_longitude_deg = float(final_sample["longitude_deg"])
        logger.event(
            (
                "E2E ⑤: ゴール判定地点 "
                if reached_goal
                else "E2E ⑤: GNSS誘導終了地点 "
            )
            + f"(lat={final_latitude_deg:.7f}, "
            f"lon={final_longitude_deg:.7f}, "
            f"distance={final_distance_m:.2f}m, "
            f"heading={final_heading_deg:.1f}度)"
        )

    _label_navigation_csv_points(
        csv_path,
        goal_reached=reached_goal,
    )
    if not reached_goal:
        logger.event(f"E2E ⑤: GNSS誘導失敗 (csv={csv_path})")
        return False

    logger.event(f"E2E ⑤: GNSS誘導成功 (csv={csv_path})")
    return True


def run_step_6_search_gnss_goal(
    navigator: NavigationController,
    driver: DriveController,
    sensors: SensorManager,
    logger: Logger,
    *,
    search_distance_m: float = GNSS_GOAL_SEARCH_DISTANCE_M,
    red_ratio_threshold: float = GNSS_GOAL_SEARCH_RED_RATIO_THRESHOLD,
    max_attempts: int = GNSS_GOAL_SEARCH_MAX_ATTEMPTS,
) -> bool:
    """⑥ GNSSゴール周辺を指定回数探索し、赤コーンを検知する。"""
    search_distance_m = float(search_distance_m)
    red_ratio_threshold = float(red_ratio_threshold)
    max_attempts = int(max_attempts)
    if search_distance_m <= 0.0:
        raise ValueError("search_distance_m must be greater than 0")
    if not 0.0 <= red_ratio_threshold <= 1.0:
        raise ValueError("red_ratio_threshold must be between 0 and 1")
    if max_attempts < 1:
        raise ValueError("max_attempts must be at least 1")

    logger.event(
        "E2E ⑥: GNSSゴール周辺探索開始 "
        f"(移動距離={search_distance_m:.1f}m, "
        f"赤割合しきい値={red_ratio_threshold:.6f}, "
        f"最大試行回数={max_attempts})"
    )

    def log_search_status(message: str) -> None:
        logger.event(f"E2E ⑥: {message}")

    def avoid_stuck_during_search() -> bool:
        return navigator.avoid_stuck(driver, sensors)

    for attempt in range(1, max_attempts + 1):
        logger.event(f"E2E ⑥: 周辺探索 {attempt}/{max_attempts} 開始")
        result = search_around_gnss_goal(
            navigator,
            driver,
            sensors,
            search_distance_m,
            red_ratio_threshold,
            status_callback=log_search_status,
            stuck_avoidance_callback=avoid_stuck_during_search,
        )

        scan_result = result.get("scan_result") or {}
        last_red_result = scan_result.get("last_red_result") or {}
        red_ratio = float(last_red_result.get("total_color_ratio", 0.0))
        logger.event(
            f"E2E ⑥: 周辺探索 {attempt}/{max_attempts} 終了 "
            f"(探索地点到着={bool(result.get('target_reached'))}, "
            f"赤検知={bool(result.get('red_detected'))}, "
            f"赤割合={red_ratio:.6f}, reason={result.get('reason')})"
        )

        if result.get("red_detected"):
            driver.stop()
            logger.event(
                "E2E ⑥: ゴール検知成功 "
                f"(試行回数={attempt}/{max_attempts}, "
                f"赤割合={red_ratio:.6f})"
            )
            return True

    driver.stop()
    logger.event(
        "E2E ⑥: ゴール検知失敗 "
        f"({max_attempts}回探索しましたが赤を検知できませんでした)"
    )
    return False


def run_step_7_red_cone_guidance(
    navigator: NavigationController,
    driver: DriveController,
    sensors: SensorManager,
    logger: Logger,
) -> bool:
    """⑦ 既存の赤コーン誘導を実行し、ゴール到達を判定する。"""
    logger.event("E2E ⑦: 赤コーン誘導開始")

    try:
        result = guide_to_red_cone(
            navigator,
            driver,
            sensors,
        )
    except Exception as exc:
        logger.event(
            "E2E ⑦: 赤コーン誘導失敗 "
            f"({type(exc).__name__}: {exc})"
        )
        return False
    finally:
        driver.stop()

    if not result.get("goal_reached"):
        logger.event(
            "E2E ⑦: ゴール判定失敗 "
            f"(試行回数={result.get('steps')}, "
            f"reason={result.get('reason')})"
        )
        return False

    last_goal_result = result.get("last_goal_result") or {}
    total_red_ratio = float(
        last_goal_result.get("total_color_ratio", 0.0)
    )
    goal_angle_red_ratio = float(
        last_goal_result.get("goal_angle_color_ratio", 0.0)
    )
    logger.event(
        "E2E ⑦: ゴール判定成功 "
        f"(試行回数={result.get('steps')}, "
        f"赤割合={total_red_ratio:.6f}, "
        f"正面赤割合={goal_angle_red_ratio:.6f}, "
        f"reason={result.get('reason')})"
    )
    return True


def main() -> int:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    logger = Logger(
        log_dir=PROJECT_ROOT / "logs",
        filename=f"E_to_E_{timestamp}.txt",
    )
    gnss_navigation_csv_path = (
        PROJECT_ROOT / "logs" / f"E_to_E_gnss_{timestamp}.csv"
    )
    sensors: SensorManager | None = None
    driver: DriveController | None = None

    try:
        logger.event("E2E試験開始")

        if (
            GNSS_GOAL_LATITUDE_DEG is None
            or GNSS_GOAL_LONGITUDE_DEG is None
        ):
            logger.event(
                "E2E試験設定エラー: 実行前にGNSS_GOAL_LATITUDE_DEGと"
                "GNSS_GOAL_LONGITUDE_DEGを設定してください"
            )
            return 2

        sensors = SensorManager()
        sensors.setup()
        logger.event("全センサ初期化成功")

        if not run_step_1_landing(sensors, logger):
            logger.event("E2E試験中止: ①着地判定が完了しませんでした")
            return 1
        send_success_notification(
            sensors,
            logger,
            "着地判定成功",
            repeat_count=LANDING_SUCCESS_SEND_COUNT,
        )

        logger.event(
            "E2E ②: 着地判定後、溶断開始まで"
            f"{LANDING_TO_FUSING_DELAY_SECONDS:.1f}秒待機"
        )
        time.sleep(LANDING_TO_FUSING_DELAY_SECONDS)

        driver = DriveController()
        if not run_step_2_fusing(driver, logger):
            logger.event("E2E試験中止: ②溶断が完了しませんでした")
            return 1
        send_success_notification(
            sensors,
            logger,
            "溶断成功",
        )

        navigator = NavigationController()
        if not run_step_3_parachute_avoidance(
            navigator,
            driver,
            sensors,
            logger,
        ):
            logger.event("E2E試験中止: ③パラシュート回避が完了しませんでした")
            return 1
        send_success_notification(
            sensors,
            logger,
            "パラシュート回避成功",
        )

        if not run_step_4_selfie(
            logger,
            sensors=sensors,
        ):
            logger.event("E2E試験中止: ④自撮り・画像送信が完了しませんでした")
            return 1

        if not run_step_5_gnss_navigation(
            driver,
            sensors,
            logger,
            gnss_navigation_csv_path,
            target_latitude_deg=GNSS_GOAL_LATITUDE_DEG,
            target_longitude_deg=GNSS_GOAL_LONGITUDE_DEG,
        ):
            logger.event("E2E試験中止: ⑤GNSS誘導が完了しませんでした")
            return 1
        send_success_notification(
            sensors,
            logger,
            "GNSS誘導成功",
        )

        if not run_step_6_search_gnss_goal(
            navigator,
            driver,
            sensors,
            logger,
        ):
            logger.event("E2E試験中止: ⑥ゴール検知が完了しませんでした")
            return 1
        send_success_notification(
            sensors,
            logger,
            "ゴール検知成功",
        )

        if not run_step_7_red_cone_guidance(
            navigator,
            driver,
            sensors,
            logger,
        ):
            logger.event("E2E試験中止: ⑦ゴール判定が完了しませんでした")
            return 1
        send_success_notification(
            sensors,
            logger,
            "ゴール判定成功",
        )

        logger.event(
            "E2E試験: ①着地判定・②溶断・③パラシュート回避・"
            "④自撮り画像送信・⑤GNSS誘導・⑥ゴール検知・"
            "⑦赤コーン誘導完了"
        )
        return 0

    except KeyboardInterrupt:
        logger.event("E2E試験中断")
        return 130
    except Exception as exc:
        logger.event(
            f"E2E試験エラー ({type(exc).__name__}: {exc})"
        )
        return 1
    finally:
        if driver is not None:
            try:
                driver.cleanup()
                logger.event("モーター終了処理完了")
            except Exception as exc:
                logger.event(
                    f"モーター終了処理失敗 ({type(exc).__name__}: {exc})"
                )

        if sensors is not None:
            try:
                sensors.close()
                logger.event("センサ終了処理完了")
            except Exception as exc:
                logger.event(
                    f"センサ終了処理失敗 ({type(exc).__name__}: {exc})"
                )

        print(f"E2Eログ: {logger.log_path}")
        if gnss_navigation_csv_path.exists():
            print(f"GNSS誘導CSV: {gnss_navigation_csv_path}")


if __name__ == "__main__":
    raise SystemExit(main())
