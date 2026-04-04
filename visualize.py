"""
Time-space visualization utility for exported dashboard CSVs.

This script reads a `*_timespace.csv.gz` file exported by `dashboard.py`
and produces one or more time-space scatter plots colored by speed.

It supports both:
- highway-style longitudinal trajectories (`position_mode="pos"`)
- urban corridor exports where the `pos` column may actually store `x` or `y`
  coordinates from the SUMO network.

Typical usage:
    python visualize.py --scenario roosevelt --p 0.3
    python visualize.py --csv sumo_scenarios/roosevelt/excel_file/roosevelt_p0.3_cav30_timespace.csv.gz

Outputs:
- A combined time-space PNG
- Optional direction-split PNGs for eastbound / westbound traffic
- Optional intersection marker lines for urban corridors
"""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import font_manager
import pandas as pd


def configure_fonts():
    preferred_fonts = [
        "Times New Roman",
        "Nimbus Roman No9 L",
        "Liberation Serif",
        "DejaVu Serif",
        "FreeSerif",
        "Times",
    ]
    available_fonts = {f.name for f in font_manager.fontManager.ttflist}
    chosen_font = next((f for f in preferred_fonts if f in available_fonts), None)
    if chosen_font is None:
        chosen_font = "serif"
        print(
            "Warning: Preferred serif fonts not found. Falling back to 'serif'."
        )
    mpl.rcParams["font.family"] = chosen_font
    mpl.rcParams["xtick.labelsize"] = 13
    mpl.rcParams["ytick.labelsize"] = 13
    mpl.rcParams["axes.labelsize"] = 15
    mpl.rcParams["axes.titlesize"] = 18


def format_penetration_tag(value: float) -> str:
    return f"p{value:g}"


def parse_penetrations(spec: str | None) -> list[float]:
    if spec is None or not spec.strip():
        return [round(i * 0.1, 10) for i in range(1, 10)]
    vals = []
    for token in spec.split(","):
        token = token.strip()
        if not token:
            continue
        vals.append(round(float(token), 10))
    return vals


def resolve_csv_path(args) -> Path:
    if args.csv:
        path = Path(args.csv)
        if not path.exists():
            raise FileNotFoundError(f"CSV not found: {path}")
        return path

    if not args.scenario:
        raise SystemExit("Provide either --csv or --scenario.")

    excel_dir = Path(args.scenario_folder) / args.scenario / "excel_file"
    tag = format_penetration_tag(args.p)
    candidates = sorted(excel_dir.glob(f"*_{tag}_*_timespace.csv.gz"))
    if not candidates:
        # legacy fallback
        pct = int(round(float(args.p) * 100))
        candidates = sorted(excel_dir.glob(f"*_cav{pct}_timespace.csv.gz"))
    if not candidates:
        raise FileNotFoundError(
            f"No timespace CSV found in {excel_dir} for penetration {args.p:g}."
        )
    return candidates[-1]


def infer_position_label(csv_path: Path, scenario: str | None) -> str:
    lower = str(csv_path).lower()
    if scenario and scenario.lower() == "roosevelt":
        return "X Coordinate Along Corridor (m)"
    if "roosevelt" in lower:
        return "X Coordinate Along Corridor (m)"
    return "Longitudinal Position (m)"


def is_urban_corridor(csv_path: Path, scenario: str | None) -> bool:
    if scenario and scenario.lower() == "roosevelt":
        return True
    return "roosevelt" in str(csv_path).lower()


def resolve_speed_limits(csv_path: Path, args) -> tuple[float | None, float | None]:
    """
    Keep urban corridor plots on a fixed color scale so east/west figures across
    penetrations remain visually comparable.
    """
    if args.speed_min is not None or args.speed_max is not None:
        return args.speed_min, args.speed_max

    if is_urban_corridor(csv_path, args.scenario):
        return 0.0, 14.0

    return None, None


def resolve_intersection_markers(args) -> dict[str, list[tuple[str, float]]]:
    """
    For urban corridors like Roosevelt, use POIs stored in sensors.add.xml as
    corridor landmarks. Since the exported time-space uses the SUMO x coordinate
    in the `pos` column, we use poi.x as the marker position.
    """
    if args.mark_intersections == "no":
        return {"all": [], "eastbound": [], "westbound": []}

    if not args.scenario:
        return {"all": [], "eastbound": [], "westbound": []}

    sensors_path = Path(args.scenario_folder) / args.scenario / "sensors.add.xml"
    if not sensors_path.exists():
        return {"all": [], "eastbound": [], "westbound": []}

    try:
        root = ET.parse(sensors_path).getroot()
    except Exception as e:
        print(f"[visualize] Failed to parse intersection markers from {sensors_path}: {e}")
        return {"all": [], "eastbound": [], "westbound": []}

    markers = []
    for poi in root.findall("poi"):
        try:
            poi_id = str(poi.attrib["id"])
            x = float(poi.attrib["x"])
            y = float(poi.attrib.get("y", 0.0))
        except Exception:
            continue
        markers.append({"id": poi_id, "x": x, "y": y})

    if args.mark_intersections == "auto" and not markers:
        return {"all": [], "eastbound": [], "westbound": []}

    markers = sorted(markers, key=lambda item: item["x"])
    all_markers = [(m["id"], m["x"]) for m in markers]

    # Roosevelt has two clear carriageway bands in y. Lower y aligns with eastbound,
    # higher y aligns with westbound for the current network geometry.
    if args.scenario and args.scenario.lower() == "roosevelt" and len(markers) >= 2:
        ys = sorted(m["y"] for m in markers)
        y_split = 0.5 * (ys[len(ys) // 2 - 1] + ys[len(ys) // 2]) if len(ys) % 2 == 0 else ys[len(ys) // 2]
        eastbound = [(m["id"], m["x"]) for m in markers if m["y"] < y_split]
        westbound = [(m["id"], m["x"]) for m in markers if m["y"] >= y_split]
        return {
            "all": all_markers,
            "eastbound": eastbound,
            "westbound": westbound,
        }

    return {
        "all": all_markers,
        "eastbound": all_markers,
        "westbound": all_markers,
    }


def infer_direction(df: pd.DataFrame) -> pd.Series:
    veh = df["veh_id"].astype(str).str.lower()

    direction = pd.Series(index=df.index, dtype="object")
    direction.loc[veh.str.startswith("east_")] = "eastbound"
    direction.loc[veh.str.startswith("west_")] = "westbound"

    unresolved = direction.isna()
    if unresolved.any():
        delta = (
            df.loc[unresolved]
            .groupby("veh_id")["pos"]
            .agg(lambda s: float(s.iloc[-1] - s.iloc[0]) if len(s) >= 2 else 0.0)
        )
        sign_map = {
            vid: ("eastbound" if d >= 0 else "westbound")
            for vid, d in delta.items()
        }
        direction.loc[unresolved] = df.loc[unresolved, "veh_id"].map(sign_map)

    return direction.fillna("unknown")


def load_timespace(csv_path: Path, max_points: int | None = None) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    required = {"time", "pos", "veh_id"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"Missing required columns in {csv_path.name}: {sorted(missing)}")

    if "speed" not in df.columns:
        df["speed"] = 0.0
    if "type" not in df.columns:
        df["type"] = "ALL"

    df = df.sort_values(["veh_id", "time"]).reset_index(drop=True)
    df["direction"] = infer_direction(df)

    if max_points is not None and len(df) > max_points:
        step = max(1, len(df) // max_points)
        df = df.iloc[::step].copy()

    return df


def save_plot(
    df: pd.DataFrame,
    out_path: Path,
    *,
    title: str,
    position_label: str,
    point_size: float,
    alpha: float,
    speed_min: float | None,
    speed_max: float | None,
    intersection_markers: list[tuple[str, float]] | None = None,
):
    if df.empty:
        print(f"[visualize] Skip empty plot: {out_path.name}")
        return

    out_path.parent.mkdir(parents=True, exist_ok=True)

    vmin = speed_min if speed_min is not None else float(df["speed"].min())
    vmax = speed_max if speed_max is not None else float(df["speed"].max())

    fig, ax = plt.subplots(figsize=(11, 6.5))
    sc = ax.scatter(
        df["time"],
        df["pos"],
        c=df["speed"],
        cmap="turbo",
        s=point_size,
        alpha=alpha,
        linewidths=0,
        vmin=vmin,
        vmax=vmax,
    )

    cbar = fig.colorbar(sc, ax=ax)
    cbar.ax.tick_params(labelsize=13)
    cbar.set_label("Speed (m/s)", fontsize=15)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(position_label)
    ax.set_title(title)
    ax.grid(True, linestyle="--", alpha=0.25)

    if intersection_markers:
        x_text = float(df["time"].min()) + 0.01 * max(1.0, float(df["time"].max()) - float(df["time"].min()))
        for name, pos_val in intersection_markers:
            ax.axhline(pos_val, color="#6c757d", linestyle="--", linewidth=0.8, alpha=0.45, zorder=0)
            ax.text(
                x_text,
                pos_val,
                f" {name}",
                fontsize=9,
                color="#495057",
                va="bottom",
                ha="left",
                alpha=0.85,
                bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.55, "pad": 0.2},
            )

    fig.tight_layout()
    fig.savefig(out_path, dpi=220, bbox_inches="tight")
    plt.close(fig)
    print(f"[visualize] Saved {out_path}")


def build_output_paths(csv_path: Path, args) -> tuple[Path, Path]:
    if args.output_dir:
        out_dir = Path(args.output_dir)
    else:
        out_dir = csv_path.parent / "plots"

    base_name = csv_path.name.replace(".csv.gz", "").replace(".csv", "")
    return out_dir, out_dir / f"{base_name}_timespace.png"


def render_one_csv(csv_path: Path, args):
    df = load_timespace(csv_path, max_points=args.max_points)
    position_label = args.position_label or infer_position_label(csv_path, args.scenario)
    markers = resolve_intersection_markers(args)
    speed_min, speed_max = resolve_speed_limits(csv_path, args)
    out_dir, combined_path = build_output_paths(csv_path, args)
    title = args.title or f"Time-Space Diagram: {csv_path.stem}"

    save_plot(
        df,
        combined_path,
        title=title,
        position_label=position_label,
        point_size=args.point_size,
        alpha=args.alpha,
        speed_min=speed_min,
        speed_max=speed_max,
        intersection_markers=markers.get("all", []),
    )

    if args.split_direction:
        for direction_name in ["eastbound", "westbound"]:
            sub = df[df["direction"] == direction_name].copy()
            if sub.empty:
                continue
            save_plot(
                sub,
                out_dir / f"{combined_path.stem}_{direction_name}.png",
                title=f"{title} ({direction_name})",
                position_label=position_label,
                point_size=args.point_size,
                alpha=args.alpha,
                speed_min=speed_min,
                speed_max=speed_max,
                intersection_markers=markers.get(direction_name, markers.get("all", [])),
            )


def main():
    parser = argparse.ArgumentParser(description="Visualize time-space CSV exports.")
    parser.add_argument("--csv", type=str, default=None, help="Direct path to *_timespace.csv.gz")
    parser.add_argument("--scenario_folder", type=str, default="sumo_scenarios")
    parser.add_argument("--scenario", type=str, default=None)
    parser.add_argument("--p", type=float, default=0.1, help="Penetration used to resolve the CSV when --csv is not given.")
    parser.add_argument("--batch", action="store_true", help="Render multiple penetrations in one run.")
    parser.add_argument(
        "--penetrations",
        type=str,
        default=None,
        help="Comma-separated penetration list for --batch. Default: 0.1..0.9",
    )
    parser.add_argument("--output_dir", type=str, default=None)
    parser.add_argument("--title", type=str, default=None)
    parser.add_argument("--position_label", type=str, default=None)
    parser.add_argument("--max_points", type=int, default=200000)
    parser.add_argument("--point_size", type=float, default=1.4)
    parser.add_argument("--alpha", type=float, default=0.65)
    parser.add_argument("--speed_min", type=float, default=None)
    parser.add_argument("--speed_max", type=float, default=None)
    parser.add_argument(
        "--split_direction",
        action="store_true",
        help="Also save separate eastbound / westbound plots when detectable.",
    )
    parser.add_argument(
        "--mark_intersections",
        choices=["auto", "yes", "no"],
        default="auto",
        help="Overlay horizontal intersection marker lines when POIs are available.",
    )
    args = parser.parse_args()

    configure_fonts()

    if args.batch:
        if args.csv:
            raise SystemExit("--batch cannot be combined with --csv. Use --scenario and --penetrations instead.")
        if not args.scenario:
            raise SystemExit("--batch requires --scenario.")
        pens = parse_penetrations(args.penetrations)
        for p in pens:
            args.p = p
            csv_path = resolve_csv_path(args)
            print(f"[visualize] Rendering penetration {p:g} from {csv_path}")
            render_one_csv(csv_path, args)
    else:
        csv_path = resolve_csv_path(args)
        render_one_csv(csv_path, args)


if __name__ == "__main__":
    main()
