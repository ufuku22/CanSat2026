#!/usr/bin/env python3
"""
CSVからGNSS軌跡と機体方位を2次元グラフとして描画する。
実行コマンド
python plot_gps_trajectory.py gps_pd_navigation_test_data.csv
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt


REQUIRED_COLUMNS = {
    "start_latitude_deg",
    "start_longitude_deg",
    "goal_latitude_deg",
    "goal_longitude_deg",
    "latitude_deg",
    "longitude_deg",
    "heading_deg",
}
EARTH_RADIUS_M = 6_371_000.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="GNSS CSVから緯度・経度の軌跡と機体方位を描画します。"
    )
    parser.add_argument("csv_path", type=Path, help="入力CSVファイルのパス")
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
    return parser.parse_args()


def float_or_none(value: str | None) -> float | None:
    if value is None or not value.strip():
        return None
    try:
        return float(value)
    except ValueError:
        return None


def load_rows(csv_path: Path) -> list[dict[str, float | None]]:
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSVファイルが見つかりません: {csv_path}")

    with csv_path.open("r", newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)
        columns = set(reader.fieldnames or ())
        missing = REQUIRED_COLUMNS - columns
        if missing:
            raise ValueError(
                "CSVに必要な列がありません: " + ", ".join(sorted(missing))
            )

        rows: list[dict[str, float | None]] = []
        for raw_row in reader:
            row = {key: float_or_none(value) for key, value in raw_row.items()}
            if row.get("latitude_deg") is None or row.get("longitude_deg") is None:
                continue
            rows.append(row)

    if not rows:
        raise ValueError("有効なlatitude_degとlongitude_degを持つ行がありません。")
    return rows


def first_value(
    rows: Iterable[dict[str, float | None]],
    key: str,
) -> float:
    for row in rows:
        value = row.get(key)
        if value is not None:
            return value
    raise ValueError(f"{key}に有効な値がありません。")


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = (
        math.sin(d_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2
    )
    return EARTH_RADIUS_M * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def automatic_arrow_length_m(
    latitudes: list[float],
    longitudes: list[float],
    start_lat: float,
    start_lon: float,
    goal_lat: float,
    goal_lon: float,
) -> float:
    all_lats = [*latitudes, start_lat, goal_lat]
    all_lons = [*longitudes, start_lon, goal_lon]
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
    """0度=北、90度=東として、矢印の経度差・緯度差を返す。"""
    heading_rad = math.radians(heading_deg)
    north_m = length_m * math.cos(heading_rad)
    east_m = length_m * math.sin(heading_rad)

    delta_lat = math.degrees(north_m / EARTH_RADIUS_M)
    cos_lat = max(abs(math.cos(math.radians(latitude_deg))), 1.0e-9)
    delta_lon = math.degrees(east_m / (EARTH_RADIUS_M * cos_lat))
    return delta_lon, delta_lat


def plot_trajectory(
    rows: list[dict[str, float | None]],
    output_path: Path,
    arrow_length_m: float | None,
    arrow_every: int,
    dpi: int,
    show: bool,
) -> None:
    start_lat = first_value(rows, "start_latitude_deg")
    start_lon = first_value(rows, "start_longitude_deg")
    goal_lat = first_value(rows, "goal_latitude_deg")
    goal_lon = first_value(rows, "goal_longitude_deg")

    latitudes = [float(row["latitude_deg"]) for row in rows]
    longitudes = [float(row["longitude_deg"]) for row in rows]

    if arrow_length_m is None:
        arrow_length_m = automatic_arrow_length_m(
            latitudes,
            longitudes,
            start_lat,
            start_lon,
            goal_lat,
            goal_lon,
        )
    if arrow_length_m <= 0:
        raise ValueError("--arrow-length-mは0より大きい値にしてください。")
    if arrow_every <= 0:
        raise ValueError("--arrow-everyは1以上にしてください。")

    fig, ax = plt.subplots(figsize=(9, 7))

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
    ax.scatter(
        [goal_lon],
        [goal_lat],
        marker="*",
        s=180,
        color="gold",
        edgecolor="black",
        linewidth=0.6,
        label="GOAL",
        zorder=6,
    )

    ax.annotate("START", (start_lon, start_lat), xytext=(6, -13), textcoords="offset points")
    ax.annotate("GOAL", (goal_lon, goal_lat), xytext=(6, 6), textcoords="offset points")

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

    # 緯度・経度の縮尺差を補正し、地上距離として形が歪みにくい比率にする。
    mean_lat = sum(latitudes + [start_lat, goal_lat]) / (len(latitudes) + 2)
    ax.set_aspect(1.0 / max(math.cos(math.radians(mean_lat)), 1.0e-9))
    ax.ticklabel_format(style="plain", axis="both", useOffset=False)
    ax.grid(True, alpha=0.35)
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title("GNSS trajectory and heading")
    ax.legend(loc="best")

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    print(f"グラフを保存しました: {output_path}")
    print(f"有効なGNSS点数: {len(rows)}")
    print(f"方位矢印の長さ: {arrow_length_m:.2f} m")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main() -> int:
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
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
