#!/usr/bin/env python3

"""
Traffic Simulation Dashboard

This script creates an interactive web dashboard to visualize and analyze traffic simulation 
data from SUMO. It provides:
- Real-time visualization of traffic metrics
- Multiple analysis tabs (Safety, Mobility, Behavioral, Time-Space)
- Interactive plots and filters
- Statistics and performance metrics
- Support for comparing CAV and HDV behaviors

Usage:
    python dashboard.py [options]

Options:
    --scenario_folder  Path to scenario folder (default: "sumo_scenarios")
    --scenario         Scenario name: "onramp" or "i24" (default: "onramp")
    --file             Input FCD file name (default: "fcd.xml")
    --direction        Filter data by vehicle direction (default: None)
    --urban            Show Urban Signals tab: 'auto' (default), 'yes' (force show), 'no' (hide)

Example:
    # Visualize onramp scenario results
    python dashboard.py --scenario onramp --file fcd.xml

Requirements:
    - dash
    - plotly
    - pandas
    - numpy
    - dash-bootstrap-components
"""

'''
ZY [10/17/2025]
1) Publication‑style visuals
   - Added `paper_pub` Plotly template (clean grid, serif font, tight margins).
   - Unified color map for HDV/CAV and urban metrics.
   - `_apply_paper_style()` standardizes axis labels/units/grid/legend across plots.

2) Urban signals integration (toggle with `--urban`)
   - Optional “Urban Signals” tab: Throughput, Avg Delay, Queue Length, v/c Ratio, Approach Delay, and spillback charts.
   - Modes: `auto` (show when detector+TLS exist), `yes` (force show), `no` (hide).

3) Time–Space analysis (viewer + exporter)
   - Time–Space tab added (Plotly hook to `vis.visualize_fcd()`; placeholder enabled by default).
   - CSV exporter `export_time_space_csv(...)` with robust filters:
     warm‑up trimming, position gate, queue‑clearing (CLEAR_V/CLEAR_M), jump detection, stride, optional speed, gzip.

4) Data loading & caching
   - Uses `load_or_build_metrics()` to reuse cached metrics when inputs are unchanged (signature = mtime+size).
   - Wired into the dashboard entrypoint to avoid rebuilding metrics on every run.

5) Result export for reproducible figures (CSV/XLSX/JSON)
   - `export_dashboard_datasets()` writes compact, plot‑ready tables to XLSX (multi‑sheet).
   - `export_spillback()` saves spillback events to CSV/JSON and prints a console summary.
   - Added time‑series and histogram bin exports so figures can be regenerated offline.

6) Guardrails & thresholds (UI aligned with analysis defaults)
   - Harmonized small‑value cutoffs for TTC/PET/Headway/Space‑gap and light clipping
     for more stable, publication‑friendly plots.
   - Consistent palette ordering (HDV blue, CAV red) across all tabs.

Note: This commit only refines comments/docstrings/labels/printouts. No functional changes.
'''

'''
Visualize traffic metrics in an interactive web dashboard.

graph TD
    A[SUMO FCD XML] --> B[TrafficMetrics Class]
    B --> C[Preprocessed Data]
    C --> D[Dash Web App]
    D --> E[Interactive Components]
    D --> F[Plots/Visualizations]
    D --> G[Stats/Metrics]

'''
"""
Usage examples:
    omramp:
    python dashboard.py --scenario onramp --urban no --exclude-lane ramp_0 --exclude-lane E2_0 --p 0.1

    i24
    python dashboard.py --scenario i24 --urban no --exclude-lane E2_0 --exclude-lane E4_0 --exclude-lane E6_0  --exclude-lane E1_0 --p 0.1

    python dashboard.py \
    --scenario i24b --urban no \
    $(printf -- '--exclude-lane %q ' "${ALL_EXCLUDES[@]}")
"""

import dash
from dash import dcc, html, Input, Output
import dash_bootstrap_components as dbc
import plotly.express as px
import pandas as pd
import numpy as np
from analysis import TrafficMetrics
from analysis import load_or_build_metrics, LaneSelector
import argparse
import os
import scripts.utils_vis as vis
import plotly.graph_objects as go  # Added: indicators/gauges/placeholder figures
import plotly.io as pio
from contextlib import contextmanager
import datetime as _dt
# Note: exclude_lane.sh should be sourced in the shell before running this script
# Or pass --exclude-lane arguments via command line


def _safe_histogram(df, x, color=None, nbins=50, barmode='overlay',
                    histnorm=None, title=''):

    if df is None or df.empty or (x not in df.columns) or df[x].dropna().empty:
        fig = go.Figure()
        fig.update_layout(title=f"{title} — no data", template="paper_pub")
        return fig

    fig = px.histogram(
        df, x=x, color=color,
        nbins=nbins, barmode=barmode,
        histnorm=histnorm, title=title,
        pattern_shape_sequence=[]   
    )
    return fig

# Unified, high-contrast palette
COLOR_MAP = {
    "HDV": "#1f77b4",
    "CAV": "#d62728",
    "Throughput": "#ff7f0e",
    "QueueLength": "#9467bd",
    "VC": "#2ca02c",
}

paper_pub = go.layout.Template(
    layout=dict(
        font=dict(family="Times New Roman, Arial, Helvetica, sans-serif", size=16),
        paper_bgcolor="white",
        plot_bgcolor="white",
        margin=dict(l=70, r=20, t=60, b=60),
        xaxis=dict(
            showline=True, linewidth=1, linecolor="#444",
            mirror=True, ticks="outside",
            showgrid=True, gridcolor="#e6e6e6", gridwidth=1
        ),
        yaxis=dict(
            showline=True, linewidth=1, linecolor="#444",
            mirror=True, ticks="outside",
            showgrid=True, gridcolor="#e6e6e6", gridwidth=1,
            zeroline=False
        ),
        legend=dict(
            bgcolor="rgba(255,255,255,0.8)",
            bordercolor="#e6e6e6",
            borderwidth=1,
            orientation="h",
            yanchor="bottom", y=1.02,
            xanchor="right", x=1
        ),
        colorway=[COLOR_MAP["HDV"], COLOR_MAP["CAV"], "#8c564b", "#17becf", "#bcbd22"]
    )
)
pio.templates["paper_pub"] = paper_pub

def _apply_paper_style(fig, x_title=None, y_title=None, x_unit=None, y_unit=None, is_pct=False):
    """
    Apply consistent axes units/grid/font/template; styles the figure in place.
    """
    fig.update_layout(template="paper_pub")
    if x_title:
        fig.update_xaxes(title=f"{x_title}" + (f" ({x_unit})" if x_unit else ""))
    if y_title:
        fig.update_yaxes(title=f"{y_title}" + (f" ({y_unit})" if y_unit else ""))
    if is_pct:
        fig.update_yaxes(tickformat=".0%")
    # Slightly narrower bars for a more publication-like look
    if any(t in str(type(fig)) for t in ["Bar", "bar"]):
        fig.update_traces(marker_line_width=0.5, marker_line_color="#333")
    return fig

# --- Spillback export + console summary ---
def export_spillback(metrics, out_dir):
    """
    Export `metrics.spillback_events` to CSV/JSON and print a brief summary.
    """
    try:
        os.makedirs(out_dir, exist_ok=True)
        events = getattr(metrics, 'spillback_events', [])
        summary = getattr(metrics, 'spillback_summary', {})

        if not events:
            print("[Spillback] No spillback events found. Nothing to export.")
            return

        df = pd.DataFrame(events)
        pen_tag = getattr(metrics, "penetration_tag", None) or "p0"
        seed_suffix = f"_seed{int(args.seed)}" if getattr(args, "seed", None) is not None else ""
        f = f"{args.scenario}_{pen_tag}{seed_suffix}_cav{int(100 * (metrics.num_cavs / max(1, metrics.num_cavs + metrics.num_hdvs)))}"
        csv_path = os.path.join(out_dir, f"{f}_spillback_events.csv")
        json_path = os.path.join(out_dir, f"{f}_spillback_events_1h_{pen_tag}.json")
        df.to_csv(csv_path, index=False)
        df.to_json(json_path, orient="records", indent=2)

        # Summary
        cnt = int(summary.get("count", len(events)))
        avg_dur = float(summary.get("avg_duration", df["end"].sub(df["start"]).mean() if "start" in df and "end" in df else 0.0))
        max_dur = float(summary.get("max_duration", df["end"].sub(df["start"]).max() if "start" in df and "end" in df else 0.0))
        min_dur = float(df["end"].sub(df["start"]).min()) if "start" in df and "end" in df else 0.0

        print("\n[Spillback] Exported spillback events:")
        print(f"  - File (CSV):  {csv_path}")
        print(f"  - File (JSON): {json_path}")
        print(f"  - Count:       {cnt}")
        print(f"  - Avg duration:{avg_dur:.2f} s")
        print(f"  - Max duration:{max_dur:.2f} s")
        print(f"  - Min duration:{min_dur:.2f} s\n")

    except Exception as e:
        print(f"[Spillback][ERROR] Failed to export/print spillback: {e}")


@contextmanager
def _maybe_excel_writer(path):
    # If you only need XLSX, always use ExcelWriter; consider CSV if file size is a concern.
    with pd.ExcelWriter(path, engine="xlsxwriter") as xw:
        yield xw

def _hist_2type(values_hdv, values_cav, bins):
    """Bin two series with the same edges; return DataFrame: bin_left, bin_right, bin_mid, HDV, CAV."""
    a = np.asarray(values_hdv, float)
    b = np.asarray(values_cav, float)
    a = a[np.isfinite(a)]; b = b[np.isfinite(b)]
    if isinstance(bins, int):
        # Auto edges from the union of two series
        allv = np.concatenate([a, b]) if a.size and b.size else (a if a.size else b)
        if allv.size == 0:
            return pd.DataFrame(columns=["bin_left","bin_right","bin_mid","HDV","CAV"])
        edges = np.histogram_bin_edges(allv, bins=bins)
    else:
        edges = np.asarray(bins, float)
    h_hdv, _ = np.histogram(a, bins=edges)
    h_cav, _ = np.histogram(b, bins=edges)
    df = pd.DataFrame({
        "bin_left":  edges[:-1],
        "bin_right": edges[1:],
        "bin_mid":  (edges[:-1] + edges[1:]) / 2.0,
        "HDV": h_hdv,
        "CAV": h_cav
    })
    return df

def export_dashboard_datasets(metrics, out_dir: str, run_label: str = "run"):
    """
    Export only the “plot-ready” small tables (instead of raw per-sample data).
    Each sheet/CSV is an aggregated dataset sufficient to reproduce the figures.
    """
    os.makedirs(out_dir, exist_ok=True)
    xlsx_path = os.path.join(out_dir, f"{run_label}_datasets.xlsx")

    # ---- Keep thresholds consistent with the front end ----
    SPACE_GAP_XMAX =  getattr(metrics, "SG_X_MAX", 80.0)
    EPS_TTC = getattr(metrics, "EPS_TTC", 0.05)   # s
    EPS_PET = getattr(metrics, "EPS_PET", 0.10)   # s
    EPS_HW  = getattr(metrics, "EPS_HW",  0.20)   # s
    EPS_GAP = getattr(metrics, "EPS_GAP", 0.50)   # m
    TT_MIN  = getattr(metrics, "TT_MIN",  0.0)    # s
    TT_MAX  = getattr(metrics, "TT_MAX",  None)   # s or None
    vmin = getattr(metrics, "SPEED_MIN_HIST", 0.5)

    with _maybe_excel_writer(xlsx_path) as xw:
        # ===================== Safety =====================
        # TTC histogram
        ttc_hdv = [v for v in metrics.ttc_values.get("hdv", []) if np.isfinite(v) and v >= EPS_TTC]
        ttc_cav = [v for v in metrics.ttc_values.get("cav", []) if np.isfinite(v) and v >= EPS_TTC]
        ttc_max = max([max(ttc_hdv or [0]), max(ttc_cav or [0]), 1.0])
        ttc_bins = np.linspace(0, ttc_max, 50)
        ttc_hist = _hist_2type(ttc_hdv, ttc_cav, ttc_bins)
        ttc_hist.to_excel(xw, sheet_name="safety_TTC_hist", index=False)

        # PET histogram
        pet_hdv = [v for v in metrics.pet_list.get("hdv", []) if np.isfinite(v) and v >= EPS_PET]
        pet_cav = [v for v in metrics.pet_list.get("cav", []) if np.isfinite(v) and v >= EPS_PET]
        pet_max = max([np.percentile(pet_hdv, 99, method="nearest") if pet_hdv else 0,
                       np.percentile(pet_cav, 99, method="nearest") if pet_cav else 0, 1.0])
        pet_bins = np.linspace(0, pet_max, 50)
        pet_hist = _hist_2type(pet_hdv, pet_cav, pet_bins)
        pet_hist.to_excel(xw, sheet_name="safety_PET_hist", index=False)

        # DRAC box summary (aggregate, not per-sample)
        def _summary(name, vals):
            v = np.asarray([x for x in vals if np.isfinite(x)], float)
            if v.size == 0:
                return {"Type": name, "count": 0, "mean": np.nan, "std": np.nan,
                        "p50": np.nan, "p90": np.nan, "p95": np.nan, "min": np.nan, "max": np.nan}
            return {"Type": name, "count": int(v.size), "mean": v.mean(), "std": v.std(ddof=1) if v.size>1 else 0.0,
                    "p50": np.percentile(v, 50), "p90": np.percentile(v, 90),
                    "p95": np.percentile(v, 95), "min": v.min(), "max": v.max()}
        drac_df = pd.DataFrame([
            _summary("HDV", metrics.drac_values.get("hdv", [])),
            _summary("CAV", metrics.drac_values.get("cav", []))
        ])
        drac_df.to_excel(xw, sheet_name="safety_DRAC_summary", index=False)

        # Headway histogram
        hw_hdv = [v for v in metrics.time_headways.get("hdv", []) if np.isfinite(v) and v >= EPS_HW]
        hw_cav = [v for v in metrics.time_headways.get("cav", []) if np.isfinite(v) and v >= EPS_HW]
        hw_max = max([np.percentile(hw_hdv, 99, method="nearest") if hw_hdv else 0,
                      np.percentile(hw_cav, 99, method="nearest") if hw_cav else 0, 1.0])
        hw_bins = np.linspace(0, hw_max, 50)
        hw_hist = _hist_2type(hw_hdv, hw_cav, hw_bins)
        hw_hist.to_excel(xw, sheet_name="safety_Headway_hist", index=False)

        # ===================== Mobility =====================
        # Speed histogram
        sv = getattr(metrics, "speeds_vis", metrics.speeds)
        sp_hdv = [v for v in sv.get("hdv", []) if np.isfinite(v)]
        sp_cav = [v for v in sv.get("cav", []) if np.isfinite(v)]
        sp_max = max([max(sp_hdv or [0]), max(sp_cav or [0]), 1.0])
        sp_bins = np.linspace(0, sp_max, 30)
        speed_hist = _hist_2type(sp_hdv, sp_cav, sp_bins)
        speed_hist.to_excel(xw, sheet_name="mobility_Speed_hist", index=False)

        # Acceleration histogram (fixed edges)
        av = getattr(metrics, "accel_vis", metrics.accelerations)
        ac_hdv = [v for v in av.get("hdv", []) if np.isfinite(v)]
        ac_cav = [v for v in av.get("cav", []) if np.isfinite(v)]
        ac_bins = np.linspace(-6, 6, 30)
        accel_hist = _hist_2type(ac_hdv, ac_cav, ac_bins)
        accel_hist.to_excel(xw, sheet_name="mobility_Accel_hist", index=False)

        # Travel time histogram (auto edges + TT_MIN/TT_MAX filtering)
        def _filter_tt(d):
            out = []
            for v in (d or {}).values():
                try:
                    x = float(v)
                except Exception:
                    continue
                if x < TT_MIN: 
                    continue
                if TT_MAX is not None and x > TT_MAX:
                    continue
                if np.isfinite(x):
                    out.append(x)
            return out
        tt_hdv = _filter_tt(metrics.travel_times.get("hdv", {}))
        tt_cav = _filter_tt(metrics.travel_times.get("cav", {}))
        tt_hist = _hist_2type(tt_hdv, tt_cav, bins=30)
        tt_hist.to_excel(xw, sheet_name="mobility_TT_hist", index=False)

        # Vehicle count time series (light downsampling)
        ts = np.asarray(metrics.timesteps, float)
        hdv = np.asarray(metrics.num_hdvs_per_timestep, float)
        cav = np.asarray(metrics.num_cavs_per_timestep, float)
        step = 5 if ts.size > 2000 else 1
        vc_df = pd.DataFrame({
            "Time(s)": ts[::step],
            "HDV":     hdv[::step],
            "CAV":     cav[::step]
        })
        vc_df.to_excel(xw, sheet_name="mobility_VehCount_ts", index=False)

        # Delay comparison table (two rows)
        d = metrics.simulation_stats.get("delay", {}) or {}
        delay_df = pd.DataFrame({
            "Vehicle Type": ["HDV","CAV"],
            "Delay per Mile (s/mile)": [float(d.get("hdv_delay_per_mile",0) or 0), float(d.get("cav_delay_per_mile",0) or 0)],
            "Total Delay (s)":         [float(d.get("hdv_total_delay",0) or 0),     float(d.get("cav_total_delay",0) or 0)],
            "Miles Traveled":          [float(d.get("hdv_total_miles",0) or 0),     float(d.get("cav_total_miles",0) or 0)]
        })
        delay_df.to_excel(xw, sheet_name="mobility_Delay_compare", index=False)

        # ===================== Behavioral =====================
        # Lane-change frequency (discrete counts)
        lc = pd.DataFrame({
            "Lane Changes": (metrics.lane_change_frequency.get("hdv", []) + metrics.lane_change_frequency.get("cav", [])),
            "Type": (["HDV"]*len(metrics.lane_change_frequency.get("hdv", [])) +
                     ["CAV"]*len(metrics.lane_change_frequency.get("cav", [])))
        })
        # Discrete histogram via value_counts
        lc_h = (lc[lc["Type"]=="HDV"]["Lane Changes"].value_counts().sort_index()).rename("HDV")
        lc_c = (lc[lc["Type"]=="CAV"]["Lane Changes"].value_counts().sort_index()).rename("CAV")
        lc_hist = pd.concat([lc_h, lc_c], axis=1).fillna(0).astype(int).reset_index().rename(columns={"index":"k"})
        lc_hist.to_excel(xw, sheet_name="behav_LC_hist", index=False)

        # Gap acceptance (s): threshold cleaning + histogram
        gap_hdv = [v for v in metrics.accepted_gaps.get("hdv", []) if np.isfinite(v) and v >= EPS_PET]
        gap_cav = [v for v in metrics.accepted_gaps.get("cav", []) if np.isfinite(v) and v >= EPS_PET]
        # Use 99th percentile as an upper bound to avoid extreme long tails
        gap_max = max([np.percentile(gap_hdv, 99, method="nearest") if gap_hdv else 0,
                       np.percentile(gap_cav, 99, method="nearest") if gap_cav else 0, 1.0])
        gap_bins = np.linspace(0, gap_max, 50)
        gap_hist = _hist_2type(gap_hdv, gap_cav, gap_bins)
        gap_hist.to_excel(xw, sheet_name="behav_Gap_hist", index=False)

        # --- Space gap (m): threshold cleaning + fixed visual upper bound ---
        EPS_GAP = getattr(metrics, "EPS_GAP", 0.50)   # physical lower limit (m)

        sg_hdv = [v for v in (metrics.space_gaps.get("hdv", []) or [])
                if np.isfinite(v) and (v >= EPS_GAP) and (v <= SPACE_GAP_XMAX)]
        sg_cav = [v for v in (metrics.space_gaps.get("cav", []) or [])
                if np.isfinite(v) and (v >= EPS_GAP) and (v <= SPACE_GAP_XMAX)]

        if sg_hdv or sg_cav:
            # Fixed bins aligned with the visual cap
            sg_bins = np.linspace(0.0, SPACE_GAP_XMAX, 50)
            sg_hist = _hist_2type(sg_hdv, sg_cav, sg_bins)
        else:
            sg_hist = pd.DataFrame({"bin_left": [], "bin_right": [], "HDV": [], "CAV": []})

        # Export histogram counts (sufficient for publication figures)
        sg_hist.to_excel(xw, sheet_name="behav_SpaceGap_hist", index=False)

        # ===== Energy & Fuel (from analysis) =====
        fuel_stats = metrics.simulation_stats.get('fuel', {})
        fuel_df = pd.DataFrame({
            "Vehicle Type": ["HDV", "CAV"],
            "Total Fuel (g)": [
                float(fuel_stats.get('hdv_total_fuel_g', 0) or 0),
                float(fuel_stats.get('cav_total_fuel_g', 0) or 0)
            ],
            "Avg Fuel per Vehicle (g)": [
                float(fuel_stats.get('hdv_avg_fuel_per_vehicle_g', 0) or 0),
                float(fuel_stats.get('cav_avg_fuel_per_vehicle_g', 0) or 0)
            ],
            "Fuel Efficiency (mile/gal)": [
                float(fuel_stats.get('hdv_fuel_efficiency_mpg', 0) or 0),
                float(fuel_stats.get('cav_fuel_efficiency_mpg', 0) or 0)
            ]
        })
        # Extra sheet: CAV fuel-savings rate (stored once)
        fuel_df_extra = pd.DataFrame({
            "Metric": ["CAV Fuel Savings (%)"],
            "Value": [float(fuel_stats.get('cav_fuel_efficiency_mpg_savings_percent', 0) or 0)]
        })
        fuel_df.to_excel(xw, sheet_name="energy_fuel_summary", index=False)
        fuel_df_extra.to_excel(xw, sheet_name="energy_fuel_summary2", index=False)

        # ===================== Urban Signals (if available) =====================
        # Throughput (summary + timeseries)
        if hasattr(metrics, "throughput_values") and isinstance(metrics.throughput_values, dict):
            tp = metrics.throughput_values
            tp_df = pd.DataFrame({
                "Metric": ["Total Vehicles", "Vehicles/hr", "CAV Vehicles", "HDV Vehicles",
                           "CAV Vehicles/hr", "HDV Vehicles/hr"],
                "Value": [tp.get("total_vehicles", 0), tp.get("vehicles_per_hour", 0),
                          tp.get("cav_vehicles", 0), tp.get("hdv_vehicles", 0),
                          tp.get("cav_vehicles_per_hour", 0), tp.get("hdv_vehicles_per_hour", 0)],
            })
            tp_df.to_excel(xw, sheet_name="urban_Throughput", index=False)
            ts_rows = tp.get("timeseries", [])
            if ts_rows:
                pd.DataFrame(ts_rows).to_excel(xw, sheet_name="urban_Throughput_ts", index=False)
            tp_summary = tp.get("summary", {})
            if tp_summary:
                pd.DataFrame(list(tp_summary.values())).to_excel(
                    xw, sheet_name="urban_Throughput_int", index=False)

        # Average Delay (summary + timeseries)
        if hasattr(metrics, "average_delay_values") and isinstance(metrics.average_delay_values, dict):
            ad = metrics.average_delay_values
            ad_df = pd.DataFrame({
                "Vehicle Type": ["ALL", "HDV", "CAV"],
                "Avg Delay (s)": [ad.get("avg_delay_all", 0), ad.get("avg_delay_hdv", 0),
                                  ad.get("avg_delay_cav", 0)],
                "Avg Stopped Delay (s)": [ad.get("avg_stopped_delay_all", 0),
                                          ad.get("avg_stopped_delay_hdv", 0),
                                          ad.get("avg_stopped_delay_cav", 0)],
                "Sample Count": [ad.get("n_all", 0), ad.get("n_hdv", 0), ad.get("n_cav", 0)],
            })
            ad_df.to_excel(xw, sheet_name="urban_AvgDelay", index=False)
            ad_ts = ad.get("timeseries", [])
            if ad_ts:
                pd.DataFrame(ad_ts).to_excel(xw, sheet_name="urban_AvgDelay_ts", index=False)

        # Queue Count (summary + timeseries)
        if hasattr(metrics, "queue_length_values") and isinstance(metrics.queue_length_values, dict):
            ql = metrics.queue_length_values
            ql_rows = ql.get("summary_rows", [])
            if ql_rows:
                pd.DataFrame(ql_rows).to_excel(xw, sheet_name="urban_QueueLength", index=False)
            else:
                ql_df = pd.DataFrame({
                    "Metric": ["Avg Queued Vehicles", "Max Queued Vehicles",
                               "Avg Queued CAV", "Avg Queued HDV"],
                    "Value": [ql.get("avg_queue_vehicles", 0), ql.get("max_queue_vehicles", 0),
                              ql.get("avg_queue_cav", 0), ql.get("avg_queue_hdv", 0)],
                })
                ql_df.to_excel(xw, sheet_name="urban_QueueLength", index=False)
            ql_cycles = ql.get("cycle_records", [])
            if ql_cycles:
                pd.DataFrame(ql_cycles).to_excel(xw, sheet_name="urban_QueueLength_cycles", index=False)

        # v/c Ratio (summary + timeseries)
        if hasattr(metrics, "vc_ratio_values") and isinstance(metrics.vc_ratio_values, dict):
            vc = metrics.vc_ratio_values
            vc_df = pd.DataFrame({
                "Metric": ["Avg v/c", "Max v/c (critical)", "Total Volume (vph)", "Total Capacity (vph)"],
                "Value": [vc.get("vc_ratio_avg", 0), vc.get("vc_ratio_max", 0),
                          vc.get("volume_vph", 0), vc.get("capacity_vph", 0)],
            })
            vc_df.to_excel(xw, sheet_name="urban_VC_Ratio", index=False)
            vc_ts = vc.get("timeseries", [])
            if vc_ts:
                pd.DataFrame(vc_ts).to_excel(xw, sheet_name="urban_VC_Ratio_ts", index=False)

    print(f"[EXPORT] Plot-ready datasets saved to: {xlsx_path}")

def create_dashboard(metrics, direction=None, urban_mode='auto'):
    '''
    If `direction` is None, plot all vehicle headings from the FCD; otherwise filter.
    '''
    app = dash.Dash(
        __name__,
        external_stylesheets=[dbc.themes.BOOTSTRAP],
        suppress_callback_exceptions=True,
    )
    
    # Basic CSS for stat cards
    app.index_string = '''
    <!DOCTYPE html>
    <html>
        <head>
            {%metas%}
            <title>{%title%}</title>
            {%favicon%}
            {%css%}
            <style>
                .stat-item {
                    margin-bottom: 8px;
                    display: flex;
                    justify-content: space-between;
                }
                .stat-label {
                    font-weight: 600;
                    color: #555;
                }
                .stat-value {
                    font-weight: 400;
                }
                .stats-grid {
                    display: flex;
                    flex-direction: column;
                    gap: 8px;
                }
                .card-title {
                    margin-bottom: 16px;
                    color: #333;
                    border-bottom: 1px solid #eee;
                    padding-bottom: 8px;
                }
                .delay-highlight {
                    background-color: rgba(0, 123, 255, 0.05);
                }
            </style>
        </head>
        <body>
            {%app_entry%}
            <footer>
                {%config%}
                {%scripts%}
                {%renderer%}
            </footer>
        </body>
    </html>
    '''
    
    # Calculate CAV penetration rate
    try:
        cav_rate = metrics.num_cavs / (metrics.num_cavs + metrics.num_hdvs)
    except Exception as e:
        cav_rate = 0

    # Simple card builder
    def create_stat_card(title, items):
        return dbc.Card(
            dbc.CardBody([
                html.H5(title, className="card-title"),
                html.Div([
                    html.Div([
                        html.Span(f"{k}: ", className="stat-label"),
                        html.Span(f"{v}", className="stat-value")
                    ], className="stat-item") for k, v in items
                ], className="stats-grid")
            ]),
            className="mb-3"
        )

    stats = metrics.simulation_stats.get('performance', {})

    performance_stats = [
        ('Duration', f"{stats.get('duration', 0):.1f} s"),
        ('Real Time Factor', f"{stats.get('realTimeFactor', 0):.2f}"),
        ('Updates/s', f"{stats.get('vehicleUpdatesPerSecond', 0):.1f}")
    ]

    vehicle_stats = [
        ('Inserted', metrics.simulation_stats.get('vehicles', {}).get('inserted', 0)),
        ('Running', metrics.simulation_stats.get('vehicles', {}).get('running', 0)),
        ('Waiting', metrics.simulation_stats.get('vehicles', {}).get('waiting', 0))
    ]

    teleport_stats = [
        ('Total', metrics.simulation_stats.get('teleports', {}).get('total', 0)),
        ('Jams', metrics.simulation_stats.get('teleports', {}).get('jam', 0))
    ]

    safety_stats = [
        ('Collisions', metrics.simulation_stats.get('safety', {}).get('collisions', 0)),
        ('Emergency Brakes', metrics.simulation_stats.get('safety', {}).get('emergencyBraking', 0))
    ]
    
    # Delay stats (from analysis metrics)
    delay_stats = metrics.simulation_stats.get('delay', {})
    delay_performance_stats = [
        ('HDV Delay/Mile', f"{delay_stats.get('hdv_delay_per_mile', 0):.1f} s/mile"),
        ('CAV Delay/Mile', f"{delay_stats.get('cav_delay_per_mile', 0):.1f} s/mile"),
        ('HDV Total Delay', f"{delay_stats.get('hdv_total_delay', 0):.1f} s"),
        ('CAV Total Delay', f"{delay_stats.get('cav_total_delay', 0):.1f} s")
    ]
    
    # Fuel stats (from analysis metrics)
    fuel_stats = metrics.simulation_stats.get('fuel', {})
    fuel_performance_stats = [
        ('HDV Fuel Efficiency', f"{fuel_stats.get('hdv_fuel_efficiency_mpg', 0):.4f} mile/gal"),
        ('CAV Fuel Efficiency', f"{fuel_stats.get('cav_fuel_efficiency_mpg', 0):.4f} mile/gal"),
        ('CAV Fuel Savings', f"{fuel_stats.get('cav_fuel_efficiency_mpg_savings_percent', 0):.1f}%"),
        ('HDV Avg Fuel', f"{fuel_stats.get('hdv_avg_fuel_per_vehicle_g', 0):.2f} g"),
        ('CAV Avg Fuel', f"{fuel_stats.get('cav_avg_fuel_per_vehicle_g', 0):.2f} g")
    ]
    
    def _urban_intersection_ids():
        """Collect all intersection IDs across urban metrics."""
        ids = set()
        for attr in ("throughput_values", "average_delay_values", "queue_length_values",
                      "vc_ratio_values"):
            vals = getattr(metrics, attr, None)
            if isinstance(vals, dict):
                for row in vals.get("timeseries", []):
                    iid = row.get("intersection_id")
                    if iid and iid != "unknown":
                        ids.add(iid)
                for row in vals.get("summary_rows", []):
                    iid = row.get("intersection_id")
                    if iid and iid != "unknown":
                        ids.add(iid)
                summary = vals.get("summary", {})
                if isinstance(summary, dict):
                    for iid in summary.keys():
                        if iid and iid != "unknown":
                            ids.add(iid)
        return sorted(ids) or ["all"]

    def urban_signals_tab():
        int_ids = _urban_intersection_ids()
        return dcc.Tab(label='Urban Signals', children=[
            # Intersection selector
            dbc.Row([
                dbc.Col([
                    html.Label("Intersection:", style={"fontWeight": 600, "marginRight": "8px"}),
                    dcc.Dropdown(
                        id='urban-intersection-select',
                        options=[{"label": i, "value": i} for i in int_ids],
                        value=int_ids[0] if int_ids else None,
                        clearable=False,
                        style={"width": "300px", "display": "inline-block"},
                    ),
                ], md=6, style={"display": "flex", "alignItems": "center", "padding": "12px"}),
            ]),
            # Time-series line charts
            dbc.Row([
                dbc.Col(dcc.Graph(id='throughput-plot'), md=6),
                dbc.Col(dcc.Graph(id='avg-delay-plot'), md=6),
            ]),
            dbc.Row([
                dbc.Col(dcc.Graph(id='queue-length-plot'), md=6),
                dbc.Col(dcc.Graph(id='vc-ratio-plot'), md=6),
            ]),
            # Summary cards
            dbc.Row([
                dbc.Col(html.Div(id='urban-summary-cards'), md=12),
            ], style={"marginTop": "16px"}),
        ])
      
    tabs_children = [
        # Safety Metrics Tab
        dcc.Tab(label='Safety Metrics', children=[
            dbc.Row([
                dbc.Col(dcc.Graph(id='ttc-plot')), 
                dbc.Col(dcc.Graph(id='pet-plot'))
            ]),
            dbc.Row([
                dbc.Col(dcc.Graph(id='drac-plot')),
                dbc.Col(dcc.Graph(id='headway-plot'))
            ])
        ]),
        
        # Mobility Metrics Tab
        dcc.Tab(label='Mobility Metrics', children=[
            dcc.Interval(id='mobility-init', interval=10000, n_intervals=0),
            dbc.Row([
                dbc.Col(dcc.Graph(id='speed-plot'), md=6),
                dbc.Col(dcc.Graph(id='accel-plot'), md=6)
            ]),
            dbc.Row([
                dbc.Col(dcc.Graph(id='travel-time-plot'), md=6),
                dbc.Col(dcc.Graph(id='vehicle-count-plot'), md=6)
            ]),
            dbc.Row([
                dbc.Col(dcc.Graph(id='delay-comparison-plot'), md=12),
            ])
        ]),
        
        # Behavioral Metrics Tab
        dcc.Tab(label='Behavioral Metrics', children=[
            dbc.Row([
                dbc.Col(dcc.Graph(id='lane-change-plot')),
                dbc.Col(dcc.Graph(id='gap-acceptance-plot'))
            ]),
            dbc.Row([
                dbc.Col(dcc.Graph(id='space-gap-plot'), md=12)
            ])
        ]),

        # Time-Space Analysis Tab
        dcc.Tab(label='Time-Space Analysis', children=[
            dbc.Row([
                dbc.Col(dcc.Graph(id='time-space-plot'), md=12)
            ]),
            dbc.Row([
                dbc.Col(dcc.Slider(
                    id='cav-penetration-slider',
                    min=0,
                    max=100,
                    value=int((metrics.num_cavs/(metrics.num_cavs+metrics.num_hdvs))*100) if (metrics.num_cavs+metrics.num_hdvs)>0 else 0,
                    marks={i: f'{i}%' for i in range(0, 101, 10)},
                    tooltip={"placement": "bottom", "always_visible": True}
                ), md=12)
            ])
        ]),
        
        # Fuel Consumption Tab
        dcc.Tab(label='Fuel Consumption', children=[
            dbc.Row([
                dbc.Col(dcc.Graph(id='fuel-efficiency-plot'), md=12)
            ]),
            dbc.Row([
                dbc.Col(dcc.Graph(id='fuel-consumption-dist-plot'), md=6),
                dbc.Col(dcc.Graph(id='fuel-time-plot'), md=6)
            ])
        ])
    ]

    # ------- Whether to show Urban Signals tab -------
    def has_urban_data():
        has_throughput = hasattr(metrics, 'throughput_values') and isinstance(metrics.throughput_values, dict) and metrics.throughput_values.get('total_vehicles', 0) > 0
        has_avg_delay = hasattr(metrics, 'average_delay_values') and isinstance(metrics.average_delay_values, dict) and metrics.average_delay_values.get('n_all', 0) > 0
        has_queue = hasattr(metrics, 'queue_length_values') and isinstance(metrics.queue_length_values, dict)
        has_vc = hasattr(metrics, 'vc_ratio_values') and isinstance(metrics.vc_ratio_values, dict)
        has_spill = hasattr(metrics, 'spillback_events') and isinstance(metrics.spillback_events, list) and len(metrics.spillback_events) > 0
        return has_throughput or has_avg_delay or has_queue or has_vc or has_spill


    show_urban_tab = False
    if urban_mode == 'yes':
        show_urban_tab = True
    elif urban_mode == 'auto':
        show_urban_tab = has_urban_data()
    else:  # 'no'
        show_urban_tab = False

    if show_urban_tab:
        tabs_children.append(urban_signals_tab())

    tabs_component = dcc.Tabs(tabs_children)

    app.layout = dbc.Container([
        html.H1(f"Traffic Simulation Dashboard - CAV Penetration: {cav_rate*100:.1f}%",
                style={'textAlign': 'center', 'margin': '20px'}),
        # Statistics row
        dbc.Row([
            dbc.Col(create_stat_card("Performance", performance_stats), md=3),
            dbc.Col(create_stat_card("Vehicles", vehicle_stats), md=3),
            dbc.Col(create_stat_card("Teleports", teleport_stats), md=3),
            dbc.Col(create_stat_card("Safety", safety_stats), md=3),
        ], className="mb-4"),
        
        # Delay statistics row
        dbc.Row([
            dbc.Col(create_stat_card("Delay Performance", delay_performance_stats), md=6),
            dbc.Col([
                html.Div([
                    html.H5("Distance Traveled", className="card-title"),
                    html.Div([
                        html.Div([
                            html.Span(f"HDV: ", className="stat-label"),
                            html.Span(f"{delay_stats.get('hdv_total_miles', 0):.2f} miles", className="stat-value")
                        ], className="stat-item"),
                        html.Div([
                            html.Span(f"CAV: ", className="stat-label"),
                            html.Span(f"{delay_stats.get('cav_total_miles', 0):.2f} miles", className="stat-value")
                        ], className="stat-item")
                    ], className="stats-grid")
                ], className="mb-3 p-3 border rounded")
            ], md=6),
        ], className="mb-4"),
        
        # Fuel consumption statistics row
        dbc.Row([
            dbc.Col(create_stat_card("Fuel Performance", fuel_performance_stats), md=6),
            dbc.Col([
                html.Div([
                    html.H5("Total Fuel Consumption", className="card-title"),
                    html.Div([
                        html.Div([
                            html.Span(f"HDV: ", className="stat-label"),
                            html.Span(f"{fuel_stats.get('hdv_total_fuel_g', 0):.2f} g", className="stat-value")
                        ], className="stat-item"),
                        html.Div([
                            html.Span(f"CAV: ", className="stat-label"),
                            html.Span(f"{fuel_stats.get('cav_total_fuel_g', 0):.2f} g", className="stat-value")
                        ], className="stat-item")
                    ], className="stats-grid")
                ], className="mb-3 p-3 border rounded")
            ], md=6),
        ], className="mb-4"),

        tabs_component
    ], fluid=True)
    
    # Callback for safety metrics
    @app.callback(
        [Output('ttc-plot', 'figure'),
         Output('pet-plot', 'figure'),
         Output('drac-plot', 'figure'),
         Output('headway-plot', 'figure')],
        [Input('ttc-plot', 'relayoutData')]
    )
    def update_safety_metrics(_):
        try:
            
            ttc_hdv = list(metrics.ttc_values.get('hdv', []) or [])
            ttc_cav = list(metrics.ttc_values.get('cav', []) or [])
            pet_hdv = list(metrics.pet_list.get('hdv', []) or [])
            pet_cav = list(metrics.pet_list.get('cav', []) or [])
            drac_hdv = list(metrics.drac_values.get('hdv', []) or [])
            drac_cav = list(metrics.drac_values.get('cav', []) or [])
            hw_hdv = list(metrics.time_headways.get('hdv', []) or [])
            hw_cav = list(metrics.time_headways.get('cav', []) or [])

            EPS_TTC = float(getattr(metrics, "EPS_TTC", 0.05))
            EPS_PET = float(getattr(metrics, "EPS_PET", 0.10))
            EPS_HW  = float(getattr(metrics, "EPS_HW",  0.20))

            import numpy as _np

            def _flt(arr, minv=None):
                v = _np.asarray(arr, float)
                v = v[_np.isfinite(v)]
                if minv is not None:
                    v = v[v >= float(minv)]
                return v

            ttc_hdv = _flt(ttc_hdv, EPS_TTC); ttc_cav = _flt(ttc_cav, EPS_TTC)
            pet_hdv = _flt(pet_hdv, EPS_PET); pet_cav = _flt(pet_cav, EPS_PET)
            drac_hdv = _flt(drac_hdv, None); drac_cav = _flt(drac_cav, None)
            hw_hdv  = _flt(hw_hdv,  EPS_HW); hw_cav  = _flt(hw_cav,  EPS_HW)

            def _empty_fig(title):
                fig = go.Figure()
                fig.update_layout(title=f"{title} — no data", template="paper_pub")
                return fig

            if ttc_hdv.size + ttc_cav.size == 0:
                ttc_fig = _empty_fig("Time to Collision Distribution")
            else:
                ttc_fig = go.Figure()
                ttc_fig.add_trace(go.Histogram(x=ttc_hdv, name="HDV", nbinsx=50, opacity=0.55, marker_color=COLOR_MAP["HDV"]))
                ttc_fig.add_trace(go.Histogram(x=ttc_cav, name="CAV", nbinsx=50, opacity=0.55, marker_color=COLOR_MAP["CAV"]))
                ttc_fig.update_layout(barmode='overlay', title='Time to Collision Distribution')

            if pet_hdv.size + pet_cav.size == 0:
                pet_fig = _empty_fig("Post-Encroachment Time Distribution")
            else:
                pet_fig = go.Figure()
                pet_fig.add_trace(go.Histogram(x=pet_hdv, name="HDV", nbinsx=50, opacity=0.55,
                                            marker_color=COLOR_MAP["HDV"]))
                pet_fig.add_trace(go.Histogram(x=pet_cav, name="CAV", nbinsx=50, opacity=0.55,
                                            marker_color=COLOR_MAP["CAV"]))
                pet_fig.update_layout(barmode='overlay', title='Post-Encroachment Time Distribution')

            if drac_hdv.size + drac_cav.size == 0:
                drac_fig = _empty_fig("Deceleration Rate to Avoid Crash")
            else:
                drac_fig = go.Figure()
                drac_fig.add_trace(go.Box(y=drac_hdv, name="HDV", marker_color=COLOR_MAP["HDV"]))
                drac_fig.add_trace(go.Box(y=drac_cav, name="CAV", marker_color=COLOR_MAP["CAV"]))
                drac_fig.update_layout(title='Deceleration Rate to Avoid Crash', boxmode='group')

            if hw_hdv.size + hw_cav.size == 0:
                headway_fig = _empty_fig("Time Headway Distribution")
            else:
                headway_fig = go.Figure()
                headway_fig.add_trace(go.Histogram(x=hw_hdv, name="HDV", nbinsx=50, opacity=0.55,
                                                marker_color=COLOR_MAP["HDV"], histnorm='density'))
                headway_fig.add_trace(go.Histogram(x=hw_cav, name="CAV", nbinsx=50, opacity=0.55,
                                                marker_color=COLOR_MAP["CAV"], histnorm='density'))
                headway_fig.update_layout(barmode='overlay', title='Time Headway Distribution')

            _apply_paper_style(ttc_fig,     x_title="TTC",     x_unit="s", y_title="Count")
            _apply_paper_style(pet_fig,     x_title="PET",     x_unit="s", y_title="Count")
            _apply_paper_style(drac_fig,    y_title="DRAC",    y_unit="m/s²")
            _apply_paper_style(headway_fig, x_title="Headway", x_unit="s", y_title="Count")

            return ttc_fig, pet_fig, drac_fig, headway_fig

        except Exception as e:
            print(f"[Safety] ERROR: {e}")
            empty = go.Figure(); empty.update_layout(template="paper_pub", title="Error")
            return empty, empty, empty, empty
    # ===== Precompute (once) for Mobility tab =====

    # Histogram bin settings (tunable)
    SPEED_BINS = np.linspace(0, max(1, np.nanmax((metrics.speeds['hdv'] or [0]) + (metrics.speeds['cav'] or [0]))), 30)
    ACC_BINS   = np.linspace(-6, 6, 30)
    TT_BINS    = 30  # Travel time: auto 30 bins is usually robust

    def _hist_2group(vals_hdv, vals_cav, bins):
        a = np.array(vals_hdv, dtype=float)
        b = np.array(vals_cav, dtype=float)
        a = a[np.isfinite(a)]; b = b[np.isfinite(b)]
        h_hdv, edges = np.histogram(a, bins=bins)
        h_cav, _     = np.histogram(b, bins=edges)
        df = pd.DataFrame({
            "bin_left":  edges[:-1],
            "bin_right": edges[1:],
            "HDV": h_hdv,
            "CAV": h_cav
        })
        df["bin_mid"] = (df["bin_left"] + df["bin_right"]) / 2.0
        return df, edges

    speed_hist_df, speed_edges = _hist_2group(metrics.speeds["hdv"], metrics.speeds["cav"], SPEED_BINS)
    acc_hist_df,   acc_edges   = _hist_2group(metrics.accelerations["hdv"], metrics.accelerations["cav"], ACC_BINS)

    # Travel time: automatic binning (unlike fixed edges for speed/accel)
    def _hist_auto(vals_hdv, vals_cav, bins=TT_BINS):
        a = np.array(list((vals_hdv or {}).values()), dtype=float)
        b = np.array(list((vals_cav or {}).values()), dtype=float)
        a = a[np.isfinite(a)]; b = b[np.isfinite(b)]
        if a.size + b.size == 0:
            return pd.DataFrame({"bin_mid":[], "HDV":[], "CAV":[]})
        all_vals = np.concatenate([a, b]) if a.size and b.size else (a if a.size else b)
        edges = np.histogram_bin_edges(all_vals, bins=bins)
        h_hdv, edges = np.histogram(a, bins=edges)
        h_cav, _     = np.histogram(b, bins=edges)
        df = pd.DataFrame({
            "bin_left":  edges[:-1],
            "bin_right": edges[1:],
            "HDV": h_hdv,
            "CAV": h_cav
        })
        df["bin_mid"] = (df["bin_left"] + df["bin_right"]) / 2.0
        return df

    tt_hist_df = _hist_auto(metrics.travel_times["hdv"], metrics.travel_times["cav"], bins=TT_BINS)

    # Vehicle count: light downsampling
    def _downsample(x, step=5):
        return x[::step] if len(x) > step else x

    ts = np.array(metrics.timesteps, dtype=float)
    cnt_hdv = np.array(metrics.num_hdvs_per_timestep, dtype=float)
    cnt_cav = np.array(metrics.num_cavs_per_timestep, dtype=float)
    DS = 5  # take one out of every five points
    ts_ds, hdv_ds, cav_ds = _downsample(ts, DS), _downsample(cnt_hdv, DS), _downsample(cnt_cav, DS)

    # Delay comparison: compact bar chart is enough
    delay_stats = metrics.simulation_stats.get('delay', {})

    # Callback for mobility metrics
    @app.callback(
        [Output('speed-plot', 'figure'),
         Output('accel-plot', 'figure'),
         Output('travel-time-plot', 'figure'),
         Output('vehicle-count-plot', 'figure'),
         Output('delay-comparison-plot', 'figure')],
        [Input('mobility-init', 'n_intervals')]
    )
    def update_mobility_metrics(_):
        # --------- DataFrames ---------
        sv = getattr(metrics, "speeds_vis", metrics.speeds)
        av = getattr(metrics, "accel_vis",  metrics.accelerations)

        speed_data = pd.DataFrame({
            'Speed': (sv['hdv'] + sv['cav']),
            'Type':  (['HDV']*len(sv['hdv']) + ['CAV']*len(sv['cav']))
        }).replace([np.inf, -np.inf], np.nan).dropna()

        accel_data = pd.DataFrame({
            'Acceleration': (av['hdv'] + av['cav']),
            'Type':  (['HDV']*len(av['hdv']) + ['CAV']*len(av['cav']))
        }).replace([np.inf, -np.inf], np.nan).dropna()

        TT_MIN = getattr(metrics, "TT_MIN", 8.0)
        TT_MAX = getattr(metrics, "TT_MAX", None)

        travel_time_data = pd.DataFrame({
            'Travel Time': list(metrics.travel_times['hdv'].values()) + list(metrics.travel_times['cav'].values()),
            'Type':  ['HDV']*len(metrics.travel_times['hdv']) + ['CAV']*len(metrics.travel_times['cav'])
        }).replace([np.inf, -np.inf], np.nan).dropna()

        # If desired, filter short/long trips only at visualization layer
        # tt = travel_time_data['Travel Time'].astype(float)
        # mask = (tt >= TT_MIN)
        # if TT_MAX is not None: mask &= (tt <= TT_MAX)
        # travel_time_data = travel_time_data[mask]

        vehicle_count_data = pd.DataFrame({
            'Time':  metrics.timesteps * 2,
            'Count': metrics.num_hdvs_per_timestep + metrics.num_cavs_per_timestep,
            'Type':  ['HDV']*len(metrics.timesteps) + ['CAV']*len(metrics.timesteps)
        }).replace([np.inf, -np.inf], np.nan).dropna()

        # --------- Figures ---------
        speed_fig = _safe_histogram(
            speed_data, x='Speed', color='Type',
            nbins=50, barmode='overlay', title='Speed Distribution'
        )
        accel_fig = _safe_histogram(
            accel_data, x='Acceleration', color='Type',
            nbins=50, barmode='overlay', title='Acceleration Distribution'
        )
        travel_time_fig = _safe_histogram(
            travel_time_data, x='Travel Time', color='Type',
            nbins=50, barmode='overlay', title='Travel Time Distribution'
        )
        count_fig = px.line(
            vehicle_count_data, x='Time', y='Count', color='Type',
            title='Vehicle Count Over Time'
        )

        delay_stats = metrics.simulation_stats.get('delay', {}) or {}
        delay_data = pd.DataFrame({
            'Vehicle Type': ['HDV', 'CAV'],
            'Delay per Mile (s)': [
                float(delay_stats.get('hdv_delay_per_mile', 0) or 0),
                float(delay_stats.get('cav_delay_per_mile', 0) or 0)
            ],
            'Total Delay (s)': [
                float(delay_stats.get('hdv_total_delay', 0) or 0),
                float(delay_stats.get('cav_total_delay', 0) or 0)
            ],
            'Miles Traveled': [
                float(delay_stats.get('hdv_total_miles', 0) or 0),
                float(delay_stats.get('cav_total_miles', 0) or 0)
            ]
        })

        delay_comparison_fig = px.bar(
            delay_data, 
            x='Vehicle Type', 
            y='Delay per Mile (s)',
            color='Vehicle Type',
            title='Delay Comparison: HDV vs CAV',
            text_auto='.1f',
            hover_data=['Total Delay (s)', 'Miles Traveled']
        )

        # Summary indicators
        ind_fig = go.Figure()
        ind_fig.add_trace(go.Indicator(
            mode="number+delta",
            value=float(delay_stats.get('cav_delay_per_mile', 0) or 0),
            number={"suffix": " s/mi"},
            delta={"reference": float(delay_stats.get('hdv_delay_per_mile', 0) or 0),
                   "increasing": {"color": COLOR_MAP["CAV"]},
                   "decreasing": {"color": COLOR_MAP["VC"]}},
            title={"text": "CAV Delay per Mile vs HDV"},
            domain={'x': [0.0, 0.48], 'y': [0, 1]}
        ))
        ind_fig.add_trace(go.Indicator(
            mode="number",
            value=float(delay_stats.get('hdv_total_delay', 0) or 0),
            number={"suffix": " s"},
            title={"text": "HDV Total Delay"},
            domain={'x': [0.52, 1.0], 'y': [0.5, 1]}
        ))
        ind_fig.add_trace(go.Indicator(
            mode="number",
            value=float(delay_stats.get('cav_total_delay', 0) or 0),
            number={"suffix": " s"},
            title={"text": "CAV Total Delay"},
            domain={'x': [0.52, 1.0], 'y': [0.0, 0.5]}
        ))

        # Unified style/units/palette
        for fig in [speed_fig, accel_fig, travel_time_fig, count_fig, delay_comparison_fig]:
            fig.update_layout(colorway=[COLOR_MAP["HDV"], COLOR_MAP["CAV"]])

        _apply_paper_style(speed_fig,         x_title="Speed",        x_unit="m/s",  y_title="Count")
        _apply_paper_style(accel_fig,         x_title="Acceleration", x_unit="m/s²", y_title="Count")
        _apply_paper_style(travel_time_fig,   x_title="Travel Time",  x_unit="s",    y_title="Count")
        _apply_paper_style(count_fig,         x_title="Time",         x_unit="s",    y_title="Vehicle Count")
        _apply_paper_style(delay_comparison_fig, x_title=None,        y_title="Seconds per Mile", y_unit="s/mile")
        _apply_paper_style(ind_fig)

        delay_comparison_fig.update_traces(width=0.4)

        return speed_fig, accel_fig, travel_time_fig, count_fig, delay_comparison_fig


    # Callback for behavioral metrics
    @app.callback(
        [Output('lane-change-plot', 'figure'),
         Output('gap-acceptance-plot', 'figure'),
         Output('space-gap-plot', 'figure')],
        [Input('lane-change-plot', 'relayoutData')]
    )
    def update_behavioral_metrics(_):
        try:
            # === Data prep ===
            lc_data = pd.DataFrame({
                'Lane Changes': metrics.lane_change_frequency['hdv'] + metrics.lane_change_frequency['cav'],
                'Type': ['HDV']*len(metrics.lane_change_frequency['hdv']) + ['CAV']*len(metrics.lane_change_frequency['cav'])
            })
            gap_data = pd.DataFrame({
                'Gap (s)': metrics.accepted_gaps['hdv'] + metrics.accepted_gaps['cav'],
                'Type': ['HDV']*len(metrics.accepted_gaps['hdv']) + ['CAV']*len(metrics.accepted_gaps['cav'])
            })
            space_gap_data = pd.DataFrame({
                'Space Gap (m)': metrics.space_gaps['hdv'] + metrics.space_gaps['cav'],
                'Type': ['HDV']*len(metrics.space_gaps['hdv']) + ['CAV']*len(metrics.space_gaps['cav'])
            })

            # === Cleaning ===
            lc_data = lc_data.replace([np.inf, -np.inf], np.nan).dropna()
            gap_data = gap_data.replace([np.inf, -np.inf], np.nan).dropna()
            space_gap_data = space_gap_data.replace([np.inf, -np.inf], np.nan).dropna()

            # Thresholds (can be read from metrics for consistent config)
            EPS_GAP = getattr(metrics, "EPS_GAP", 0.5)   # m
            EPS_PET = getattr(metrics, "EPS_PET", 0.05)  # s

            # Use the correct column names while filtering
            gap_data = gap_data[gap_data['Gap (s)'] >= EPS_PET]
            space_gap_data = space_gap_data[space_gap_data['Space Gap (m)'] >= EPS_GAP]

            # === Plotting ===
            lc_fig = _safe_histogram(
                lc_data, x='Lane Changes', color='Type',
                nbins=20, barmode='overlay',
                title='Lane Change Frequency Distribution',
            )

            gap_fig = _safe_histogram(
                gap_data, x='Gap (s)', color='Type',
                nbins=50, barmode='overlay',
                title='Gap Acceptance Distribution',
            )

            space_gap_fig = _safe_histogram(
                space_gap_data, x='Space Gap (m)', color='Type',
                nbins=50, barmode='overlay',
                title='Space Gap Distribution',
            )
            space_gap_fig.update_xaxes(range=[0, 80.0])

            # === Layout standardization ===
            for fig in [lc_fig, gap_fig, space_gap_fig]:
                fig.update_layout(
                    margin=dict(l=70, r=20, t=60, b=60),
                    bargap=0.12,
                    legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
                )

            _apply_paper_style(lc_fig,        x_title="Lane Changes", y_title="Count")
            _apply_paper_style(gap_fig,       x_title="Gap",          x_unit="s", y_title="Count")
            _apply_paper_style(space_gap_fig, x_title="Space Gap",    x_unit="m", y_title="Count")

            return lc_fig, gap_fig, space_gap_fig

        except Exception as e:
            print(f"Error in behavioral metrics update: {e}")
            raise


    # Time-Space diagram
    @app.callback(
        Output('time-space-plot', 'figure'),
        [Input('time-space-plot', 'relayoutData')]
    )
    def update_time_space(_):
        """
        To use the actual FCD visualization, re-enable the call below:
        return vis.visualize_fcd(
            metrics.fcd_file,
            lanes=None,
            direction=direction,
            return_plotly=True)
        """
        fig = go.Figure()
        fig.update_layout(
            template="paper_pub",
            title="Time–Space Diagram (temporarily disabled)",
            xaxis_title=None,
            yaxis_title=None,
            showlegend=False,
            margin=dict(l=70, r=20, t=60, b=60)
        )
        # Cleaner axes
        fig.update_xaxes(showgrid=True, gridcolor="#e6e6e6", zeroline=False)
        fig.update_yaxes(showgrid=True, gridcolor="#e6e6e6", zeroline=False)

        _apply_paper_style(fig, x_title="Time", x_unit="s", y_title="Position", y_unit="m")
        return fig

    # Fuel consumption tab
    @app.callback(
        [Output('fuel-efficiency-plot', 'figure'),
         Output('fuel-consumption-dist-plot', 'figure'),
         Output('fuel-time-plot', 'figure')],
        [Input('fuel-efficiency-plot', 'relayoutData')]
    )
    def update_fuel_metrics(_):
        try:
            fuel_stats = metrics.simulation_stats.get('fuel', {})

            # 1) Fuel efficiency comparison (HDV vs CAV)
            fuel_comparison_data = pd.DataFrame({
                'Vehicle Type': ['HDV', 'CAV'],
                'Fuel Efficiency (mile/gal)': [
                    fuel_stats.get('hdv_fuel_efficiency_mpg', 0),
                    fuel_stats.get('cav_fuel_efficiency_mpg', 0)
                ],
                'Total Fuel (g)': [
                    fuel_stats.get('hdv_total_fuel_g', 0),
                    fuel_stats.get('cav_total_fuel_g', 0)
                ],
                'Avg Fuel per Vehicle (g)': [
                    fuel_stats.get('hdv_avg_fuel_per_vehicle_g', 0),
                    fuel_stats.get('cav_avg_fuel_per_vehicle_g', 0)
                ]
            })
            fuel_efficiency_fig = px.bar(
                fuel_comparison_data,
                x='Vehicle Type',
                y='Fuel Efficiency (mile/gal)',
                color='Vehicle Type',
                title=f'Fuel Efficiency Comparison — CAV Savings: {fuel_stats.get("cav_fuel_efficiency_mpg_savings_percent", 0):.1f}%',
                text_auto='.4f',
                hover_data=['Total Fuel (g)', 'Avg Fuel per Vehicle (g)']
            )

            # 2) Per-trip fuel distribution
            fuel_dist_data = pd.DataFrame({
                'Fuel Consumption (g)': metrics.per_trip_fuel_consumption['hdv'] + metrics.per_trip_fuel_consumption['cav'],
                'Type': ['HDV']*len(metrics.per_trip_fuel_consumption['hdv']) + ['CAV']*len(metrics.per_trip_fuel_consumption['cav'])
            })
            fuel_dist_fig = _safe_histogram(
                fuel_dist_data.dropna(),
                x='Fuel Consumption (g)',
                color='Type',
                nbins=50,
                barmode='overlay',
                title='Per-Trip Fuel Consumption Distribution'
            )

            # 3) Average fuel per vehicle (with totals/vehicle count in hover)
            total_fuel_data = pd.DataFrame({
                'Vehicle Type': ['HDV', 'CAV'],
                'Total Fuel (g)': [
                    fuel_stats.get('hdv_total_fuel_g', 0),
                    fuel_stats.get('cav_total_fuel_g', 0)
                ],
                'Avg Fuel per Vehicle (g)': [
                    fuel_stats.get('hdv_avg_fuel_per_vehicle_g', 0),
                    fuel_stats.get('cav_avg_fuel_per_vehicle_g', 0)
                ],
                'Vehicle Count': [metrics.num_hdvs, metrics.num_cavs]
            })
            fuel_time_fig = px.bar(
                total_fuel_data,
                x='Vehicle Type',
                y='Avg Fuel per Vehicle (g)',
                color='Vehicle Type',
                title='Average Fuel Consumption per Vehicle',
                text_auto='.2f',
                hover_data=['Total Fuel (g)', 'Vehicle Count']
            )

            # Unified palette ordering: HDV blue, CAV red
            for fig in [fuel_efficiency_fig, fuel_dist_fig, fuel_time_fig]:
                fig.update_layout(colorway=[COLOR_MAP["HDV"], COLOR_MAP["CAV"]])

            # Publication style + axis units
            _apply_paper_style(fuel_efficiency_fig, x_title=None, y_title="Fuel Efficiency", y_unit="mile/gal")
            _apply_paper_style(fuel_dist_fig,      x_title="Fuel per Trip",  x_unit="g",        y_title="Count")
            _apply_paper_style(fuel_time_fig,      x_title=None,             y_title="Fuel per Vehicle", y_unit="g")

            # Narrower bars for a cleaner look
            fuel_efficiency_fig.update_traces(width=0.4)
            fuel_time_fig.update_traces(width=0.4)

            # Minor tweaks: gaps and margins for the distribution chart
            fuel_dist_fig.update_layout(bargap=0.1)

            return fuel_efficiency_fig, fuel_dist_fig, fuel_time_fig

        except Exception as e:
            print(f"Error in fuel metrics update: {e}")
            empty_fig = px.bar(title="No fuel data available")
            _apply_paper_style(empty_fig)
            return empty_fig, empty_fig, empty_fig

    # Callback: Urban Signals — time-series line charts per intersection
    @app.callback(
        [Output('throughput-plot', 'figure'),
         Output('avg-delay-plot', 'figure'),
         Output('queue-length-plot', 'figure'),
         Output('vc-ratio-plot', 'figure'),
         Output('urban-summary-cards', 'children')],
        [Input('urban-intersection-select', 'value')]
    )
    def update_urban_signals(selected_int):
        """Render urban-signal time-series line charts for selected intersection."""
        def empty_fig(title):
            fig = go.Figure()
            fig.update_layout(title=title, template="paper_pub")
            return fig

        def _filter_ts(vals_attr, int_id):
            vals = getattr(metrics, vals_attr, None)
            if not isinstance(vals, dict):
                return []
            ts = vals.get("timeseries", [])
            if not ts:
                return ts
            if int_id and int_id != "all":
                return [r for r in ts if r.get("intersection_id") == int_id]
            return ts

        def _ts_to_df(ts_rows):
            if not ts_rows:
                return pd.DataFrame()
            df = pd.DataFrame(ts_rows)
            if "window_start" in df.columns:
                df["time_min"] = df["window_start"] / 60.0
            return df

        # 1) Throughput line chart
        tp_ts = _filter_ts("throughput_values", selected_int)
        tp_df = _ts_to_df(tp_ts)
        if not tp_df.empty and "vehicles_per_hour" in tp_df.columns:
            tp_fig = go.Figure()
            tp_fig.add_trace(go.Scatter(
                x=tp_df["time_min"], y=tp_df["vehicles_per_hour"],
                mode="lines+markers", name="Total",
                line=dict(color=COLOR_MAP["Throughput"], width=2),
            ))
            tp_fig.update_layout(title="Throughput", template="paper_pub")
            _apply_paper_style(tp_fig, x_title="Time", x_unit="min", y_title="Vehicles/hr")
        else:
            tp_fig = empty_fig("Throughput: No data")

        # 2) Average Delay line chart
        ad_ts = _filter_ts("average_delay_values", selected_int)
        ad_df = _ts_to_df(ad_ts)
        if not ad_df.empty and "avg_delay_all" in ad_df.columns:
            delay_fig = go.Figure()
            for col, name, color in [("avg_delay_hdv", "HDV", COLOR_MAP["HDV"]),
                                      ("avg_delay_cav", "CAV", COLOR_MAP["CAV"]),
                                      ("avg_delay_all", "All", "#9467bd")]:
                if col in ad_df.columns:
                    delay_fig.add_trace(go.Scatter(
                        x=ad_df["time_min"], y=ad_df[col],
                        mode="lines+markers", name=name,
                        line=dict(color=color, width=2),
                    ))
            delay_fig.update_layout(title="Average Delay", template="paper_pub")
            _apply_paper_style(delay_fig, x_title="Time", x_unit="min", y_title="Avg Delay", y_unit="s")
        else:
            delay_fig = empty_fig("Average Delay: No data")

        # 3) Queue summary by approach
        ql_vals = getattr(metrics, "queue_length_values", {})
        ql_rows = []
        if isinstance(ql_vals, dict):
            summary_rows = ql_vals.get("summary_rows", [])
            if summary_rows:
                ql_rows = list(summary_rows)
            else:
                summary = ql_vals.get("summary", {})
                if isinstance(summary, dict):
                    if selected_int and selected_int != "all" and isinstance(summary.get(selected_int), list):
                        ql_rows = list(summary.get(selected_int, []))
                    elif selected_int and selected_int != "all" and isinstance(summary.get(selected_int), dict):
                        ql_rows = [summary.get(selected_int)]

        if selected_int and selected_int != "all":
            ql_rows = [r for r in ql_rows if r.get("intersection_id") == selected_int]
        else:
            ql_rows = [r for r in ql_rows if r.get("intersection_id") != "unknown"]

        ql_df = pd.DataFrame(ql_rows)
        if not ql_df.empty and "Average Queue" in ql_df.columns:
            if "approach_direction" not in ql_df.columns:
                ql_df["approach_direction"] = ql_df.get("approach_edge", "")
            ql_df["approach_label"] = ql_df["approach_direction"].astype(str)
            ql_plot_df = ql_df.melt(
                id_vars=["approach_label"],
                value_vars=[c for c in ["Average Queue", "Max Queue"] if c in ql_df.columns],
                var_name="Metric",
                value_name="Queued Vehicles",
            )
            ql_fig = px.bar(
                ql_plot_df,
                x="approach_label",
                y="Queued Vehicles",
                color="Metric",
                barmode="group",
                title="Queue by Approach",
                template="paper_pub",
                text_auto=".1f",
            )
            _apply_paper_style(ql_fig, x_title="Approach", y_title="Queued Vehicles")
            ql_fig.update_layout(bargap=0.25)
        else:
            ql_fig = empty_fig("Queue Count: No data")

        # 4) v/c Ratio line chart
        vc_ts = _filter_ts("vc_ratio_values", selected_int)
        vc_df = _ts_to_df(vc_ts)
        if not vc_df.empty and "vc_ratio_avg" in vc_df.columns:
            vc_fig = go.Figure()
            vc_fig.add_trace(go.Scatter(
                x=vc_df["time_min"], y=vc_df["vc_ratio_avg"],
                mode="lines+markers", name="Avg v/c",
                line=dict(color=COLOR_MAP["VC"], width=2),
            ))
            if "vc_ratio_max" in vc_df.columns:
                vc_fig.add_trace(go.Scatter(
                    x=vc_df["time_min"], y=vc_df["vc_ratio_max"],
                    mode="lines", name="Critical v/c",
                    line=dict(color=COLOR_MAP["VC"], width=1, dash="dash"),
                ))
            vc_fig.add_hline(y=1.0, line_dash="dot", line_color="red",
                             annotation_text="Capacity", annotation_position="top right")
            vc_fig.update_layout(title="v/c Ratio", template="paper_pub")
            _apply_paper_style(vc_fig, x_title="Time", x_unit="min", y_title="v/c Ratio")
        else:
            vc_fig = empty_fig("v/c Ratio: No data")

        # Summary cards (global or per-intersection)
        summary_items = []
        tp_vals = getattr(metrics, "throughput_values", {})
        if isinstance(tp_vals, dict):
            summary_items.append(f"Throughput: {tp_vals.get('vehicles_per_hour', 0):.0f} veh/hr")
        ad_vals = getattr(metrics, "average_delay_values", {})
        if isinstance(ad_vals, dict):
            summary_items.append(f"Avg Delay: {ad_vals.get('avg_delay_all', 0):.1f}s")
        ql_vals = getattr(metrics, "queue_length_values", {})
        if isinstance(ql_vals, dict):
            ql_summary_rows = ql_vals.get("summary_rows", [])
            if selected_int and selected_int != "all":
                ql_summary_rows = [r for r in ql_summary_rows if r.get("intersection_id") == selected_int]
            if ql_summary_rows:
                ql_summary_df = pd.DataFrame(ql_summary_rows)
                summary_items.append(f"Average Queue: {float(ql_summary_df['Average Queue'].mean()):.1f} veh")
                summary_items.append(f"Max Queue: {float(ql_summary_df['Max Queue'].max()):.1f} veh")
            else:
                summary_items.append(f"Average Queue: {ql_vals.get('avg_queue_vehicles', 0):.1f} veh")
        vc_vals = getattr(metrics, "vc_ratio_values", {})
        if isinstance(vc_vals, dict):
            summary_items.append(f"v/c: {vc_vals.get('vc_ratio_avg', 0):.3f}")

        cards = html.Div([
            html.Span(item, style={"marginRight": "24px", "fontWeight": "500"})
            for item in summary_items
        ], style={"padding": "8px 0", "fontSize": "14px", "color": "#555"})

        return tp_fig, delay_fig, ql_fig, vc_fig, cards

    return app


def export_time_space_csv(
    metrics,
    out_dir: str,
    filename: str = "timespace.csv.gz",
    stride: int = 10,
    max_vehicles_per_type: int | None = 9999,
    include_speed: bool = True,
    compress: bool = True,
    # —— Cleaning parameters —— #
    WARMUP_S: float = 3.0,
    CLEAR_V: float = 2.0,
    CLEAR_M: int = 3,
    JUMP_POS_THRESH: float = 200.0,
    SPEED_MIN_VIS: float = 0.0,
    SPEED_MAX_VIS: float = 60.0,
    prefer_raw_speed: bool = True,
    # —— Applies only when position_mode == 'pos' —— #
    POS_MIN_FILTER: float = 0.0,
    # —— Position source: 'pos' | 'x' | 'y' —— #
    position_mode: str = "pos",
):

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, filename)

    rows = []
    speed_traj = getattr(metrics, "speed_traj", None)
    first_seen = getattr(metrics, "first_seen", {"hdv": {}, "cav": {}})

    # Trajectory source
    def _traj_iter(vtype: str, vid: str):
        if position_mode == "pos":
            seq = (metrics.positions.get(vtype, {}) or {}).get(vid, [])
            for (t, s) in seq:
                yield float(t), float(s)
        elif position_mode in ("x", "y"):
            xy = (getattr(metrics, "xy_traj", {}).get(vtype, {}) or {}).get(vid, [])
            if position_mode == "x":
                for (t, x, y) in xy:
                    yield float(t), float(x)
            else:
                for (t, x, y) in xy:
                    yield float(t), float(y)
        else:
            raise ValueError("position_mode must be 'pos'|'x'|'y'.")

    # —— Speed lookup: nearest-neighbor + tolerance —— #
    def _speed_lookup(vtype: str, vid: str):
        if not (prefer_raw_speed and isinstance(speed_traj, dict)):
            return None
        seq = speed_traj.get(vtype, {}).get(vid, None)
        if not seq:
            return None

        ts = np.array([float(t) for (t, _) in seq], dtype=float)
        vs = np.array([float(v) for (_, v) in seq], dtype=float)
        if ts.size == 0:
            return None
        order = np.argsort(ts)
        ts = ts[order]; vs = vs[order]
    # Infer base timestep and provide tolerance (covers/overrides stride sampling)
        dt_base = float(np.median(np.diff(ts))) if ts.size >= 3 else 0.1
        tol = max(0.45 * dt_base * max(1, int(stride)), 0.06)

        def nearest(t: float) -> float:
            i = int(np.searchsorted(ts, t))
            cand = []
            if 0 <= i < ts.size: cand.append(i)
            if i - 1 >= 0:       cand.append(i - 1)
            if i + 1 < ts.size:  cand.append(i + 1)
            if not cand:
                return float('nan')
            j = min(cand, key=lambda k: abs(ts[k] - t))
            return float(vs[j]) if abs(ts[j] - t) <= tol else float('nan')

        return nearest

    for vtype in ["hdv", "cav"]:
        if position_mode == "pos":
            ids_all = list((metrics.positions.get(vtype, {}) or {}).keys())
        else:
            ids_all = list((getattr(metrics, "xy_traj", {}).get(vtype, {}) or {}).keys())
        if not ids_all:
            continue

    # —— Adjust vehicle truncation logic —— #
        if (max_vehicles_per_type is None) or (int(max_vehicles_per_type) <= 0):
            veh_ids = ids_all
        else:
            veh_ids = ids_all[: int(max_vehicles_per_type)]

        for vid in veh_ids:
            traj = list(_traj_iter(vtype, vid))
            if not traj:
                continue
            traj.sort(key=lambda x: x[0])
            if stride > 1:
                traj = traj[::stride]

            # warm-up (based on first_seen)
            t0_seen = float(first_seen.get(vtype, {}).get(vid, traj[0][0]))
            cut = t0_seen + float(WARMUP_S)
            traj = [(t, s) for (t, s) in traj if t >= cut]
            if not traj:
                continue

            if position_mode == "pos" and POS_MIN_FILTER > 0.0:
                traj = [(t, s) for (t, s) in traj if s >= POS_MIN_FILTER]
                if not traj:
                    continue

            times = [t for (t, _) in traj]
            vals  = [s for (_, s) in traj]

            # Speed: prefer raw (nearest-neighbor); otherwise finite-difference (pos only)
            speeds = None
            if include_speed:
                nn = _speed_lookup(vtype, vid)
                if nn is not None:
                    speeds = [nn(t) for t in times]
                else:
                    if position_mode == "pos":
                        sp = []
                        for i in range(len(traj)):
                            if i == 0:
                                sp.append(np.nan)
                            else:
                                dt = max(1e-9, times[i] - times[i-1])
                                ds = vals[i] - vals[i-1]
                                sp.append(np.nan if abs(ds) > JUMP_POS_THRESH else ds / dt)
                        speeds = sp
                    else:
                        speeds = [np.nan] * len(times)

            # Clear queue (only when we have speeds)
            if include_speed and (speeds is not None):
                start_idx = 0; consec = 0
                for i, v in enumerate(speeds):
                    vv = 0.0 if not np.isfinite(v) else max(0.0, float(v))
                    if vv >= CLEAR_V:
                        consec += 1
                        if consec >= CLEAR_M:
                            start_idx = i - CLEAR_M + 1
                            break
                    else:
                        consec = 0
                if start_idx > 0:
                    times  = times[start_idx:]
                    vals   = vals[start_idx:]
                    speeds = speeds[start_idx:]
                if not times:
                    continue

            # Clamp values + write rows
            if include_speed and (speeds is not None):
                out_sp = []
                for v in speeds:
                    if np.isfinite(v):
                        out_sp.append(max(SPEED_MIN_VIS, min(SPEED_MAX_VIS, float(v))))
                    else:
                        out_sp.append(np.nan)
                for t, s, v in zip(times, vals, out_sp):
                    rows.append({"time": t, "pos": s, "veh_id": str(vid), "type": vtype.upper(), "speed": v})
            else:
                for t, s in zip(times, vals):
                    rows.append({"time": t, "pos": s, "veh_id": str(vid), "type": vtype.upper()})

    if not rows:
        print("[TimeSpace] No trajectory data to export.")
        return


    df = pd.DataFrame(rows).sort_values(["veh_id", "time"])
    df["veh_id"] = df["veh_id"].astype(str)
    df["type"]   = df["type"].astype("category")

    if compress:
        if not filename.endswith(".gz"):
            path += ".gz"
        df.to_csv(path, index=False, compression="gzip")
    else:
        df.to_csv(path, index=False)

    print(f"[TimeSpace] Exported time–space dataset: {path} (rows={len(df)})")
    print(f"[TimeSpace] position_mode={position_mode}")


if __name__ == '__main__':

    # ---------------------------------------------------------------------
    # CLI lane filtering (sample-level) & run examples
    #
    # Options (can be repeated):
    #   --exclude-lane LANE_ID
    #   --exclude-lane-prefix PREFIX
    #   --exclude-lane-regex REGEX
    #   --edge-pos-window EDGE_ID:POS_MIN:POS_MAX
    #   --lane-pos-window LANE_ID:POS_MIN:POS_MAX
    #
    # Run examples:
    # 1) Exclude a few ramp lanes (sample-level; keep the same vehicle on mainline):
    #    python dashboard_onramp_git.py --scenario onramp --urban no \
    #      --exclude-lane ramp_0 --exclude-lane onramp_1 --exclude-lane off_r_2
    #
    # 2) Exclude all ramp* lanes and drop the first 120 m on a mainline edge:
    #    python dashboard_onramp_git.py --scenario onramp --urban no \
    #      --exclude-lane-prefix ramp_ --exclude-lane-prefix onramp_ --exclude-lane-prefix offramp_ \
    #      --edge-pos-window I24_EB_main_3:0:120
    #
    # 3) Use regex to exclude typical ramp/link lanes:
    #    python dashboard_onramp_git.py --scenario onramp --urban no \
    #      --exclude-lane-regex "^(ramp|onramp|offramp|.*_link)_\\d+$"
    #
    # 4) Drop the first 120 m on a specific lane (sample-level only):
    #    python dashboard_onramp_git.py --scenario onramp --urban no \
    #      --lane-pos-window I24_EB_main_3_0:0:120
    #
    # Notes:
    # - Filtering is *sample-level* (strict_vehicle_filter="sample"): samples on excluded lanes/windows
    #   are dropped, but the same vehicle’s samples on allowed mainline lanes are kept.
    # - Internal/junction lanes (IDs starting with ':') are dropped by default in the code (drop_internal=True).
    # ---------------------------------------------------------------------

    parser = argparse.ArgumentParser(description='Traffic Simulation Dashboard')
    parser.add_argument('--scenario_folder', default="sumo_scenarios", type=str)
    parser.add_argument('--scenario', default="onramp", type=str)
    parser.add_argument('--file', default="fcd.xml", type=str)
    parser.add_argument('--direction', default=None, type=str)  # Select direction based on heading in FCD data
    parser.add_argument('--urban', choices=['auto', 'yes', 'no'], default='auto',
                        help="Show Urban Signals tab: 'auto' (default, show when detector+TLS present), 'yes' (force show), 'no' (hide)")
    parser.add_argument('--p', '--penetration-tag', dest='p_tag', default="0",
                        help="Penetration tag for output filenames, e.g. 0.1 or p0.1. If provided, dashboard will look for fcd_<tag>.xml and stats_<tag>.xml and will use metrics_<tag>.pkl cache. Default: 0 -> p0")
    parser.add_argument('--seed', default=None, type=int,
                        help='Optional run seed used only to keep exported dataset filenames distinct across repeated runs.')
    parser.add_argument('--export-only', action='store_true',
                        help='Build metrics and export dashboard datasets without starting the Dash web server.')
    parser.add_argument('--rebuild-cache', action='store_true',
                        help='Delete the cached metrics_<tag>.pkl before analysis so metrics are rebuilt from source files.')
    parser.add_argument('--host', default='0.0.0.0', type=str,
                        help='Host for the Dash server. Default 0.0.0.0')
    parser.add_argument('--port', default=8050, type=int,
                        help='Port for the Dash server. Default 8050')

    # ---- (optional) CLI lane-filtering rules ----
    parser.add_argument('--exclude-lane', action='append', default=[],
                        help='Exclude specific lane IDs (can be used multiple times)')
    parser.add_argument('--exclude-lane-prefix', action='append', default=[],
                        help='Exclude lanes by prefix (e.g., ramp_)')
    parser.add_argument('--exclude-lane-regex', action='append', default=[],
                        help='Exclude lanes by regex (e.g., "^(ramp|onramp|offramp)_")')
    # If you want to remove the first 120 m on mainline transitions, you can pass these (repeatable):
    parser.add_argument('--edge-pos-window', action='append', default=[],
                        help='Drop samples on an edge within a pos window: EDGE_ID:POS_MIN:POS_MAX (e.g., I24_EB_main_3:0:120)')
    parser.add_argument('--lane-pos-window', action='append', default=[],
                        help='Drop samples on a lane within a pos window: LANE_ID:POS_MIN:POS_MAX (e.g., I24_EB_main_3_0:0:120)')

    args = parser.parse_args()

    # Normalize penetration tag argument to form like 'p0.1' (or None to use defaults)
    if args.p_tag is None:
        pen_tag = None
    else:
        p_raw = str(args.p_tag)
        try:
            p_val = float(p_raw)
            pen_tag = f"p{p_val:g}"
        except Exception:
            pen_tag = p_raw if p_raw.startswith('p') else f"p{p_raw}"

    output_path = os.path.join(args.scenario_folder, args.scenario, 'output')

    if args.rebuild_cache:
        cache_path = os.path.join(output_path, f"metrics_{pen_tag}.pkl")
        if os.path.exists(cache_path):
            try:
                os.remove(cache_path)
                print(f"Deleted cached metrics file: {cache_path}")
            except Exception as e:
                print(f"[WARN] Failed to delete cached metrics file {cache_path}: {e}")

    # -------------------- Build LaneSelector (sample-level filtering) --------------------
    selector = None
    if (args.exclude_lane or args.exclude_lane_prefix or args.exclude_lane_regex
        or args.edge_pos_window or args.lane_pos_window):
        selector = LaneSelector(
            # Use only "exclude" rules; avoid include-lists so you don't accidentally drop mainline lanes.
            exclude_lanes=args.exclude_lane,
            exclude_lane_prefixes=args.exclude_lane_prefix,
            exclude_lane_regexes=args.exclude_lane_regex,
            drop_internal=True,  # For highways, it’s usually best to drop internal/junction lanes (IDs start with ':')
        )
        # Parse position-window specs (e.g., drop the first 120 m on specific edges/lanes)
        for spec in args.edge_pos_window:
            try:
                edge_id, a, b = spec.split(':', 2)
                selector.add_edge_pos_window(edge_id, float(a), float(b), regex=False)
            except Exception as e:
                print(f"[WARN] Bad --edge-pos-window '{spec}': {e}")
        for spec in args.lane_pos_window:
            try:
                lane_id, a, b = spec.split(':', 2)
                selector.add_lane_pos_window(lane_id, float(a), float(b), regex=False)
            except Exception as e:
                print(f"[WARN] Bad --lane-pos-window '{spec}': {e}")

    # -------------------- Key step: pass selector + enable sample-level mode --------------------
    enable_urban_metrics = args.urban != 'no'

    metrics = load_or_build_metrics(
        file_dir=output_path,
        urban_mode=enable_urban_metrics,
        lane_selector=selector,
        strict_vehicle_filter="sample",  # Sample-level filtering; keeps the same vehicle’s samples on allowed lanes
        penetration_tag=pen_tag,
    )

    export_spillback(metrics, out_dir=output_path)

    # Call exporters (run_label uses scenario + CAV penetration for clarity)
    output_excel = os.path.join(args.scenario_folder, args.scenario, 'excel_file')
    actual_cav_pct = int(100 * (metrics.num_cavs / max(1, metrics.num_cavs + metrics.num_hdvs)))
    seed_suffix = f"_seed{int(args.seed)}" if args.seed is not None else ""
    run_label = f"{args.scenario}_{pen_tag}{seed_suffix}_cav{actual_cav_pct}"
    time_space_name = f"{run_label}_timespace.csv.gz"

    """
        export_time_space_csv(
        
        metrics,
        out_dir=output_excel,
        filename=time_space_name,
        stride=10,
        max_vehicles_per_type=9999,
        include_speed=True,
        compress=True
    )
    """
    
    export_time_space_csv(metrics, out_dir=output_excel, filename=time_space_name, position_mode="x")

    export_dashboard_datasets(metrics, out_dir=output_excel, run_label=run_label)

    print(f"[EXPORT] Metrics and plot-ready datasets saved to: {output_excel}")

    if args.export_only:
        print("Export-only mode enabled. Skipping Dash server startup.")
        raise SystemExit(0)

    app = create_dashboard(metrics, direction=args.direction, urban_mode=args.urban)
    app.run(debug=False, host=args.host, port=args.port)


