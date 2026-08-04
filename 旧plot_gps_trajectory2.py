#!/usr/bin/env python3
"""
CSVからGNSS軌跡と機体方位を2次元グラフとして描画する。

想定するCSV列:
    timestamp
    elapsed_s
    latitude_deg
    longitude_deg
    target_latitude_deg
    target_longitude_deg
    heading_deg

開始地点は、最初の有効なlatitude_deg、longitude_degを使用する。
目標地点は、最初の有効なtarget_latitude_deg、
target_longitude_degを使用する。

実行例:
    python plot_gps_trajectory2.py mission_20260804_181539_history.csv
    
矢印なし
    python plot_gps_trajectory2.py mission_20260804_181539_history.csv --no-heading

画像表示を行わず保存だけする場合:
    python plot_gps_trajectory2.py mission_20260804_181539_history.csv --no-show
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt


REQUIRED_COLUMNS = {
    "latitude_deg",
    "longitude_deg",
    "target_latitude_deg",
    "target_longitude_deg",
    "heading_deg",
}

EARTH_RADIUS_M = 6_371_000.0


def parse_args() -> argparse.Namespace:
    """コマンドライン引数を読み取る。"""
    parser = argparse.ArgumentParser(
        description="GNSS CSVから緯度・経度の軌跡と機体方位を描画します。"
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
    """
    CSVの文字列をfloatへ変換する。

    空欄や数値に変換できない値の場合はNoneを返す。
    """
    if value is None or not value.strip():
        return None

    try:
        return float(value)
    except ValueError:
        return None


def load_rows(csv_path: Path) -> list[dict[str, float | None]]:
    """
    CSVファイルを読み込む。

    latitude_degとlongitude_degの両方が有効な行だけを返す。
    """
    if not csv_path.is_file():
        raise FileNotFoundError(
            f"CSVファイルが見つかりません: {csv_path}"
        )

    with csv_path.open(
        "r",
        newline="",
        encoding="utf-8-sig",
    ) as file:
        reader = csv.DictReader(file)

        columns = {
            column.strip()
            for column in (reader.fieldnames or ())
            if column is not None
        }

        missing = REQUIRED_COLUMNS - columns

        if missing:
            raise ValueError(
                "CSVに必要な列がありません: "
                + ", ".join(sorted(missing))
            )

        rows: list[dict[str, float | None]] = []

        for raw_row in reader:
            # 列名の前後に空白がある場合も吸収する。
            normalized_row = {
                key.strip(): value
                for key, value in raw_row.items()
                if key is not None
            }

            # 今回の描画で使用する数値列だけを変換する。
            row: dict[str, float | None] = {
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

            # 現在位置が取得できない行は、軌跡に使用できないため除外する。
            if (
                row["latitude_deg"] is None
                or row["longitude_deg"] is None
            ):
                continue

            rows.append(row)

    if not rows:
        raise ValueError(
            "有効なlatitude_degとlongitude_degを持つ行がありません。"
        )

    return rows


def first_value(
    rows: Iterable[dict[str, float | None]],
    key: str,
) -> float:
    """指定した列から最初の有効な数値を取得する。"""
    for row in rows:
        value = row.get(key)

        if value is not None:
            return value

    raise ValueError(f"{key}に有効な値がありません。")


def first_position(
    rows: Iterable[dict[str, float | None]],
) -> tuple[float, float]:
    """
    最初の有効なGNSS位置を開始地点として取得する。

    戻り値:
        (緯度, 経度)
    """
    for row in rows:
        latitude = row.get("latitude_deg")
        longitude = row.get("longitude_deg")

        if latitude is not None and longitude is not None:
            return latitude, longitude

    raise ValueError("開始地点として使えるGNSS位置がありません。")


def first_target_position(
    rows: Iterable[dict[str, float | None]],
) -> tuple[float, float]:
    """
    target_latitude_degとtarget_longitude_degの両方が
    同じ行で有効な最初の目標位置を取得する。

    戻り値:
        (緯度, 経度)
    """
    for row in rows:
        latitude = row.get("target_latitude_deg")
        longitude = row.get("target_longitude_deg")

        if latitude is not None and longitude is not None:
            return latitude, longitude

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

    # 浮動小数点誤差によってaが1をわずかに超える場合への対策。
    a = min(1.0, max(0.0, a))

    return (
        EARTH_RADIUS_M
        * 2.0
        * math.atan2(
            math.sqrt(a),
            math.sqrt(1.0 - a),
        )
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
    all_lats = [
        *latitudes,
        start_lat,
        target_lat,
    ]

    all_lons = [
        *longitudes,
        start_lon,
        target_lon,
    ]

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
    方位角と長さから、経度差と緯度差を求める。

    方位角の定義:
        0度   = 北
        90度  = 東
        180度 = 南
        270度 = 西

    戻り値:
        (経度差, 緯度差)
    """
    heading_rad = math.radians(heading_deg)

    north_m = length_m * math.cos(heading_rad)
    east_m = length_m * math.sin(heading_rad)

    delta_lat = math.degrees(
        north_m / EARTH_RADIUS_M
    )

    cos_lat = max(
        abs(math.cos(math.radians(latitude_deg))),
        1.0e-9,
    )

    delta_lon = math.degrees(
        east_m / (EARTH_RADIUS_M * cos_lat)
    )

    return delta_lon, delta_lat


def plot_trajectory(
    rows: list[dict[str, float | None]],
    output_path: Path,
    arrow_length_m: float | None,
    arrow_every: int,
    dpi: int,
    show: bool,
    show_heading:bool,
) -> None:
    """GNSS軌跡、開始位置、目標位置、機体方位を描画する。"""

    # 開始地点は最初の有効なGNSS位置とする。
    start_lat, start_lon = first_position(rows)

    # 目標地点はtarget列から取得する。
    target_lat, target_lon = first_target_position(rows)

    latitudes = [
        float(row["latitude_deg"])
        for row in rows
    ]

    longitudes = [
        float(row["longitude_deg"])
        for row in rows
    ]

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

    if arrow_every <= 0:
        raise ValueError(
            "--arrow-everyは1以上にしてください。"
        )

    if dpi <= 0:
        raise ValueError(
            "--dpiは1以上にしてください。"
        )

    fig, ax = plt.subplots(figsize=(9, 7))

    # GNSS軌跡
    ax.plot(
        longitudes,
        latitudes,
        color="tab:blue",
        linewidth=1.6,
        marker="o",
        markersize=4.5,
        markerfacecolor="tab:red",
        markeredgecolor="tab:red",
        label="GNSS track",
        zorder=3,
    )

    # 開始地点
    ax.scatter(
        [start_lon],
        [start_lat],
        marker="*",
        s=180,
        color="tab:green",
        edgecolor="black",
        linewidth=0.6,
        label="START",
        zorder=6,
    )

    # 目標地点
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

    # 機体方位の描画
    arrow_count = 0

    if show_heading:
        arrow_label_added = False

        for index in range(0, len(rows), arrow_every):
            row = rows[index]
            heading = row.get("heading_deg")

            if heading is None:
                continue

            latitude = float(row["latitude_deg"])
            longitude = float(row["longitude_deg"])

            delta_lon, delta_lat = heading_vector_degrees(
                latitude,
                float(heading),
                arrow_length_m,
            )

            ax.quiver(
                longitude,
                latitude,
                delta_lon,
                delta_lat,
                angles="xy",
                scale_units="xy",
                scale=1,
                color="tab:red",
                width=0.004,
                headwidth=4.5,
                headlength=6,
                headaxislength=5,
                label="Heading" if not arrow_label_added else None,
                zorder=5,
            )

            arrow_label_added = True
            arrow_count += 1

    # 緯度と経度の縮尺差を補正する。
    mean_lat = (
        sum(latitudes)
        + start_lat
        + target_lat
    ) / (len(latitudes) + 2)

    ax.set_aspect(
        1.0
        / max(
            math.cos(math.radians(mean_lat)),
            1.0e-9,
        )
    )

    ax.ticklabel_format(
        style="plain",
        axis="both",
        useOffset=False,
    )

    ax.grid(True, alpha=0.35)
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    if show_heading:
        ax.set_title("GNSS trajectory and heading")
    else:
        ax.set_title("GNSS trajectory")
    ax.legend(loc="best")

    fig.tight_layout()

    output_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    fig.savefig(
        output_path,
        dpi=dpi,
        bbox_inches="tight",
    )

    print(f"グラフを保存しました: {output_path}")
    print(f"有効なGNSS点数: {len(rows)}")
    if show_heading:
        print(f"描画した方位矢印数: {arrow_count}")
        print(f"方位矢印の長さ: {arrow_length_m:.2f} m")
    else:
        print("機体方位の矢印: 非表示")
    print(
        f"開始地点: "
        f"latitude={start_lat:.8f}, "
        f"longitude={start_lon:.8f}"
    )
    print(
        f"目標地点: "
        f"latitude={target_lat:.8f}, "
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