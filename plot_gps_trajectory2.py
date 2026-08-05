#!/usr/bin/env python3
"""
CSVからGNSS軌跡と機体方位を2次元グラフとして描画する。

想定するCSV列:
    timestamp
    elapsed_s
    phase
    latitude_deg
    longitude_deg
    target_latitude_deg
    target_longitude_deg
    heading_deg

開始地点は、最初の有効なlatitude_deg、longitude_degを使用する。
目標地点は、最初の有効なtarget_latitude_deg、
target_longitude_degを使用する。

GNSSが取得できず緯度・経度が空欄の行は、軌跡の切れ目として扱う。
headingがあっても緯度・経度がない行は、方位矢印を描画しない。

実行例:
    python plot_gps_trajectory2.py logs/mission_20260804_181539_history.csv

矢印なし:
    python plot_gps_trajectory2.py logs/mission_20260804_181539_history.csv --no-heading

画像表示を行わず保存だけする場合:
    python plot_gps_trajectory2.py logs/mission_20260804_181539_history.csv --no-show

矢印なし、画像表示なし保存:
    python plot_gps_trajectory2.py logs/mission_20260804_181539_history.csv --no-heading --no-show

ラズパイ上で軌跡画像生成→pcのデスクトップにダウンロード:
    ラズパイで実行
    CSV=mission_20260804_181539_history.csv; MPLBACKEND=Agg python3 plot_gps_trajectory2.py "$CSV" --no-heading --no-show --output "$HOME/CanSat2026/logs/${CSV%.csv}_trajectory.png"
    
    pcのコマンドプロンプトで実行し、デスクトップにダウンロード
    $CSV = "mission_20260804_181539_history"; scp "argus@100.90.248.62:~/CanSat2026/logs/${CSV}_trajectory.png" "C:\Users\higuc\Desktop\"
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
from matplotlib.axes import Axes


REQUIRED_COLUMNS = {
    "phase",
    "latitude_deg",
    "longitude_deg",
    "target_latitude_deg",
    "target_longitude_deg",
    "heading_deg",
}

PHASE_COLORS = {
    "waiting_for_release": "tab:gray",
    "waiting_for_landing": "tab:brown",
    "landed": "black",
    "deploying": "tab:orange",
    "recovering_gnss": "tab:olive",
    "clearing_landing_area": "goldenrod",
    "selfie": "tab:pink",
    "gnss_navigation": "tab:blue",
    "searching_goal": "tab:purple",
    "guiding_to_goal": "tab:green",
    "completed": "tab:cyan",
}

UNKNOWN_PHASE_COLOR = "lightgray"
EARTH_RADIUS_M = 6_371_000.0

# phaseは文字列、緯度・経度・方位はfloatまたはNoneなので、
# 行データの値型をobjectとして扱う。
Row = dict[str, object]


def parse_args() -> argparse.Namespace:
    """コマンドライン引数を読み取る。"""
    parser = argparse.ArgumentParser(
        description="GNSS CSVからphase別の軌跡と機体方位を描画します。"
    )

    parser.add_argument(
        "csv_path",
        type=Path,
        help="入力CSVファイルのパス",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="保存する画像のパス。省略時はCSVと同じ場所へPNG保存",
    )

    parser.add_argument(
        "--arrow-length-m",
        type=float,
        default=None,
        help="方位矢印の長さ[m]。省略時は軌跡の大きさから自動決定",
    )

    parser.add_argument(
        "--arrow-every",
        type=int,
        default=1,
        help="方位矢印を何点ごとに描くか。既定値は1",
    )

    parser.add_argument(
        "--dpi",
        type=int,
        default=160,
        help="保存画像のDPI。既定値は160",
    )

    parser.add_argument(
        "--no-show",
        action="store_true",
        help="画面に表示せず、画像保存だけ行う",
    )

    parser.add_argument(
        "--no-heading",
        action="store_true",
        help="機体方位の矢印を表示しない",
    )

    return parser.parse_args()


def float_or_none(value: str | None) -> float | None:
    """CSVの文字列をfloatへ変換し、空欄や不正値ではNoneを返す。"""
    if value is None or not value.strip():
        return None

    try:
        return float(value)
    except ValueError:
        return None


def load_rows(csv_path: Path) -> list[Row]:
    """
    CSVファイルを読み込む。

    GNSSが空欄の行も削除せず保持し、後の描画時に軌跡の切れ目として扱う。
    """
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSVファイルが見つかりません: {csv_path}")

    with csv_path.open("r", newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)

        columns = {
            column.strip()
            for column in (reader.fieldnames or ())
            if column is not None
        }

        missing = REQUIRED_COLUMNS - columns
        if missing:
            raise ValueError(
                "CSVに必要な列がありません: " + ", ".join(sorted(missing))
            )

        rows: list[Row] = []

        for raw_row in reader:
            normalized_row = {
                key.strip(): value
                for key, value in raw_row.items()
                if key is not None
            }

            phase_text = (normalized_row.get("phase") or "").strip()

            row: Row = {
                "phase": phase_text or "unknown",
                "latitude_deg": float_or_none(
                    normalized_row.get("latitude_deg")
                ),
                "longitude_deg": float_or_none(
                    normalized_row.get("longitude_deg")
                ),
                "target_latitude_deg": float_or_none(
                    normalized_row.get("target_latitude_deg")
                ),
                "target_longitude_deg": float_or_none(
                    normalized_row.get("target_longitude_deg")
                ),
                "heading_deg": float_or_none(
                    normalized_row.get("heading_deg")
                ),
            }

            # GNSS空欄行も保持する。これにより欠損区間を線で結ばずに済む。
            rows.append(row)

    if not rows:
        raise ValueError("CSVにデータ行がありません。")

    if not any(has_position(row) for row in rows):
        raise ValueError(
            "有効なlatitude_degとlongitude_degを持つ行がありません。"
        )

    return rows


def has_position(row: Row) -> bool:
    """行に有効な緯度と経度が両方あるか判定する。"""
    return (
        row.get("latitude_deg") is not None
        and row.get("longitude_deg") is not None
    )


def phase_name(row: Row) -> str:
    """行のphase名を取得し、空欄の場合はunknownを返す。"""
    value = row.get("phase")
    if value is None:
        return "unknown"

    result = str(value).strip()
    return result or "unknown"


def first_position(rows: Iterable[Row]) -> tuple[float, float]:
    """最初の有効なGNSS位置を開始地点として取得する。"""
    for row in rows:
        if has_position(row):
            return (
                float(row["latitude_deg"]),
                float(row["longitude_deg"]),
            )

    raise ValueError("開始地点として使えるGNSS位置がありません。")


def first_target_position(rows: Iterable[Row]) -> tuple[float, float]:
    """緯度と経度が同じ行で有効な最初の目標位置を取得する。"""
    for row in rows:
        latitude = row.get("target_latitude_deg")
        longitude = row.get("target_longitude_deg")

        if latitude is not None and longitude is not None:
            return float(latitude), float(longitude)

    raise ValueError(
        "target_latitude_degとtarget_longitude_degに"
        "有効な目標位置がありません。"
    )


def haversine_m(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
) -> float:
    """緯度・経度で表された2地点間の地表距離を計算する。"""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)

    a = (
        math.sin(d_phi / 2.0) ** 2
        + math.cos(phi1)
        * math.cos(phi2)
        * math.sin(d_lambda / 2.0) ** 2
    )

    # 浮動小数点誤差でaが0未満または1超過になる場合への対策。
    a = min(1.0, max(0.0, a))

    return (
        EARTH_RADIUS_M
        * 2.0
        * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    )


def automatic_arrow_length_m(
    latitudes: list[float],
    longitudes: list[float],
    start_lat: float,
    start_lon: float,
    target_lat: float,
    target_lon: float,
) -> float:
    """軌跡全体の大きさから方位矢印の長さを自動決定する。"""
    all_lats = [*latitudes, start_lat, target_lat]
    all_lons = [*longitudes, start_lon, target_lon]

    diagonal_m = haversine_m(
        min(all_lats),
        min(all_lons),
        max(all_lats),
        max(all_lons),
    )

    return max(0.5, diagonal_m * 0.08)


def heading_vector_degrees(
    latitude_deg: float,
    heading_deg: float,
    length_m: float,
) -> tuple[float, float]:
    """
    方位角と長さから経度差と緯度差を求める。

    0度=北、90度=東、180度=南、270度=西。
    """
    heading_rad = math.radians(heading_deg)
    north_m = length_m * math.cos(heading_rad)
    east_m = length_m * math.sin(heading_rad)

    delta_lat = math.degrees(north_m / EARTH_RADIUS_M)
    cos_lat = max(abs(math.cos(math.radians(latitude_deg))), 1.0e-9)
    delta_lon = math.degrees(east_m / (EARTH_RADIUS_M * cos_lat))

    return delta_lon, delta_lat


def plot_phase_trajectory(ax: Axes, rows: list[Row]) -> set[str]:
    """
    phaseごとに点と線を同じ色で描画する。

    phaseが変わった場合、またはGNSSが空欄の行があった場合に線を分断する。
    """
    plotted_phases: set[str] = set()
    segment_lons: list[float] = []
    segment_lats: list[float] = []
    segment_phase: str | None = None

    def draw_segment() -> None:
        nonlocal segment_lons, segment_lats, segment_phase

        if segment_lons and segment_phase is not None:
            color = PHASE_COLORS.get(segment_phase, UNKNOWN_PHASE_COLOR)
            label = (
                segment_phase
                if segment_phase not in plotted_phases
                else None
            )

            ax.plot(
                segment_lons,
                segment_lats,
                color=color,
                linewidth=2.0,
                marker="o",
                markersize=4.5,
                markerfacecolor=color,
                markeredgecolor=color,
                label=label,
                zorder=3,
            )

            plotted_phases.add(segment_phase)

        segment_lons = []
        segment_lats = []
        segment_phase = None

    for row in rows:
        current_phase = phase_name(row)

        # GNSS欠損行では、それ以前と以後を線で結ばない。
        if not has_position(row):
            draw_segment()
            continue

        # phaseの切り替わりでも線を分け、色が混ざらないようにする。
        if segment_phase is not None and current_phase != segment_phase:
            draw_segment()

        if segment_phase is None:
            segment_phase = current_phase

        segment_lons.append(float(row["longitude_deg"]))
        segment_lats.append(float(row["latitude_deg"]))

    draw_segment()
    return plotted_phases


def plot_trajectory(
    rows: list[Row],
    output_path: Path,
    arrow_length_m: float | None,
    arrow_every: int,
    dpi: int,
    show: bool,
    show_heading: bool,
) -> None:
    """GNSS軌跡をphase別に描画し、必要に応じて機体方位も描画する。"""
    start_lat, start_lon = first_position(rows)
    target_lat, target_lon = first_target_position(rows)

    valid_position_rows = [row for row in rows if has_position(row)]
    latitudes = [float(row["latitude_deg"]) for row in valid_position_rows]
    longitudes = [float(row["longitude_deg"]) for row in valid_position_rows]

    if arrow_every <= 0:
        raise ValueError("--arrow-everyは1以上にしてください。")

    if dpi <= 0:
        raise ValueError("--dpiは1以上にしてください。")

    if show_heading:
        if arrow_length_m is None:
            arrow_length_m = automatic_arrow_length_m(
                latitudes,
                longitudes,
                start_lat,
                start_lon,
                target_lat,
                target_lon,
            )

        if arrow_length_m <= 0:
            raise ValueError(
                "--arrow-length-mは0より大きい値にしてください。"
            )

    fig, ax = plt.subplots(figsize=(11, 8))

    # phaseごとに点と線を同じ色で描画する。
    plotted_phases = plot_phase_trajectory(ax, rows)

    ax.scatter(
        [start_lon],
        [start_lat],
        marker="*",
        s=180,
        color="lime",
        edgecolor="black",
        linewidth=0.6,
        label="START",
        zorder=6,
    )

    ax.scatter(
        [target_lon],
        [target_lat],
        marker="*",
        s=180,
        color="gold",
        edgecolor="black",
        linewidth=0.6,
        label="TARGET",
        zorder=6,
    )

    ax.annotate(
        "START",
        (start_lon, start_lat),
        xytext=(6, -13),
        textcoords="offset points",
    )

    ax.annotate(
        "TARGET",
        (target_lon, target_lat),
        xytext=(6, 6),
        textcoords="offset points",
    )

    arrow_count = 0
    heading_without_position_count = 0

    if show_heading:
        heading_position_index = 0

        for row in rows:
            heading = row.get("heading_deg")

            if heading is None:
                continue

            # headingがあっても位置がなければ地図上に矢印を配置できない。
            if not has_position(row):
                heading_without_position_count += 1
                continue

            # GNSS位置とheadingが両方ある行だけを間引き対象にする。
            should_draw = heading_position_index % arrow_every == 0
            heading_position_index += 1

            if not should_draw:
                continue

            latitude = float(row["latitude_deg"])
            longitude = float(row["longitude_deg"])
            current_phase = phase_name(row)
            heading_color = PHASE_COLORS.get(
                current_phase,
                UNKNOWN_PHASE_COLOR,
            )

            # show_heading=Trueの場合、上で必ずfloat値が設定されている。
            assert arrow_length_m is not None

            delta_lon, delta_lat = heading_vector_degrees(
                latitude,
                float(heading),
                arrow_length_m,
            )

            # 矢印もその行のphaseと同じ色にする。
            ax.quiver(
                longitude,
                latitude,
                delta_lon,
                delta_lat,
                angles="xy",
                scale_units="xy",
                scale=1,
                color=heading_color,
                width=0.004,
                headwidth=4.5,
                headlength=6,
                headaxislength=5,
                zorder=5,
            )

            arrow_count += 1

    mean_lat = (
        sum(latitudes) + start_lat + target_lat
    ) / (len(latitudes) + 2)

    ax.set_aspect(
        1.0 / max(math.cos(math.radians(mean_lat)), 1.0e-9)
    )

    ax.ticklabel_format(
        style="plain",
        axis="both",
        useOffset=False,
    )

    ax.grid(True, alpha=0.35)
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title(
        "GNSS trajectory by phase with heading"
        if show_heading
        else "GNSS trajectory by phase"
    )

    # phaseが多くても軌跡と重なりにくいよう、凡例を右側へ配置する。
    ax.legend(
        loc="upper left",
        bbox_to_anchor=(1.02, 1.0),
        borderaxespad=0,
    )

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")

    gnss_missing_count = len(rows) - len(valid_position_rows)

    print(f"グラフを保存しました: {output_path}")
    print(f"CSVデータ行数: {len(rows)}")
    print(f"有効なGNSS点数: {len(valid_position_rows)}")
    print(f"GNSS欠損行数: {gnss_missing_count}")
    print(f"描画したphase数: {len(plotted_phases)}")

    if show_heading:
        print(f"描画した方位矢印数: {arrow_count}")
        print(
            "headingはあるがGNSS位置がなく、"
            f"矢印を描画できなかった行数: {heading_without_position_count}"
        )
        assert arrow_length_m is not None
        print(f"方位矢印の長さ: {arrow_length_m:.2f} m")
    else:
        print("機体方位の矢印: 非表示")

    print(
        f"開始地点: latitude={start_lat:.8f}, "
        f"longitude={start_lon:.8f}"
    )
    print(
        f"目標地点: latitude={target_lat:.8f}, "
        f"longitude={target_lon:.8f}"
    )

    if show:
        plt.show()
    else:
        plt.close(fig)


def main() -> int:
    """スクリプト全体の処理を実行する。"""
    args = parse_args()

    output_path = args.output or args.csv_path.with_name(
        args.csv_path.stem + "_trajectory.png"
    )

    rows = load_rows(args.csv_path)

    plot_trajectory(
        rows,
        output_path=output_path,
        arrow_length_m=args.arrow_length_m,
        arrow_every=args.arrow_every,
        dpi=args.dpi,
        show=not args.no_show,
        show_heading=not args.no_heading,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
