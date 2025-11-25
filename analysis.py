"""
Author: Yanbing Wang
This script supports post-simulation analysis. Planned/implemented metrics include:

0. Visualizations & Statistics
    - Time–space diagrams with CAV penetration
    - Number of CAVs and HDVs present over time
    - Stats: simulation duration; simulation logs (teleported, jammed, waiting, hard braking, etc.)

1. Safety
    a. DONE TTC distribution
    b. DONE PET distribution
    c. DONE Time headway distribution (HDV and CAV)
    d. Proportion of Stopping Distance (PSD). A PSD value < 1 indicates an unsafe situation,
       since a collision cannot be avoided even with maximum deceleration.
    e. DONE Deceleration Rate to Avoid a Crash (DRAC)

2. Mobility
    a. Throughput vs. penetration rate
    b. DONE Total delay
    c. Lane-specific travel time
    d. Level of service
    c. DONE Average speed
    - Queue length: average/maximum length of vehicle queues at intersections or bottlenecks.
    - Stop time: total time vehicles spend stopped or idling during the simulation.

3. Environmental impact
    a. Emissions
    b. Fuel consumption rate

4. Behavioral
    a. Overtaking rate (frequency and duration of overtaking maneuvers)
    b. DONE HDV/CAV following gap distribution
    c. DONE Lane-change frequency (number of lane changes per vehicle)
    d. Lane-change duration
    e. Reaction time
    f. Gap acceptance (accepted gaps during lane changes or merges)

5. Macroscopic characteristics
    a. Shockwave propagation: extent/intensity of stop‑and‑go waves in the traffic stream.

6. Microsimulation quality‑control report
    a. Lane/edge‑level travel time
    b. Real‑time factor, vehicles inserted/teleported/jammed/waiting, etc.

SUMO output files
1) fcd data looks like:
    </timestep>
    <timestep time="0.50">
        <vehicle id="ramp_0_hdv.4" x="723.98" y="-24.86" angle="73.04" type="hdv" speed="13.24" pos="129.27" lane="ramp_0" slope="0.00" acceleration="0.02" accelerationLat="0.00" leaderID="ramp_0_hdv.3" leaderSpeed="14.04" leaderGap="70.46"/>
        <vehicle id="ramp_0_hdv.5" x="668.01" y="-41.92" angle="73.04" type="hdv" speed="13.26" pos="55.44" lane="ramp_0" slope="0.00" acceleration="-0.11" accelerationLat="0.00" leaderID="ramp_0_cav.2" leaderSpeed="13.58" leaderGap="28.68"/>
    </timestep>

2) stats.xml
    <statistics ...>
        <performance clockBegin="1741283428.73" clockEnd="1741283445.20" clockDuration="16.47" traciDuration="0.63" realTimeFactor="36.43" vehicleUpdatesPerSecond="29088.22" personUpdatesPerSecond="0.00" begin="0.00" end="600.00" duration="600.00"/>
        <vehicles loaded="54035" inserted="1598" running="479" waiting="52434"/>
        <teleports total="1" jam="0" yield="0" wrongLane="1"/>
        <safety collisions="0" emergencyStops="0" emergencyBraking="24"/>
    </statistics>

3) by_edge.xml, by_lane.xml
"""

'''
ZY [10/17/2025]
1) Introduced an "urban_mode" switch and a full urban‑signal analytics pipeline:
   - Parsers for TLS time‑series/static tlLogic, detector outputs, and add.xml lane mapping;
     composed detector→lane→TLS mapping.
   - Implemented urban metrics: PAoG, GOR (overall & by type), intersection approach delay,
     queue spillback detection, and Time‑To‑Service (TTS), with optional partial execution
     when only detector data is available.
   - Built green‑window extraction and lane‑occupancy grids to support the above metrics.

2) Added `load_or_build_metrics(file_dir, urban_mode)` as a cache loader:
   - Computes an input‑file signature (mtime + size) and reuses `metrics_p0.1t.pkl` when
     inputs are unchanged; auto‑rebuilds the cache when they differ.
   - Enables the Dashboard to load metrics without recomputing when prior results exist.

3) Added robust gating/limits & sanitization knobs for FCD/XML parsing and metric stability:
   - Gates: `POS_MIN_ANALYSIS`, `TT_USE_POS_GATE`, `ENERGY_USE_POS_GATE`.
   - Bounds: `V_MAX`, `ACC_MIN`/`ACC_MAX`, `HW_V_EPS`.
   - Visualization bounds: `SPEED_MIN_HIST`, `SPEED_MAX_HIST`, `ACC_MAX_HIST`.
   - Sanitization thresholds for PET/TTC/headway/space‑gap to remove numerical artifacts.

4) Reworked `fuel_rate_g_per_s` (fuel model):
   - Physics‑based traction‑power + auxiliaries with an idle floor and BSFC conversion (g/s),
     numerically stable at low speed/load; unified, easily tunable parameters (mass, CdA, Crr).


'''



import xml.etree.ElementTree as ET
import numpy as np
from math import nan, isnan
from collections import defaultdict
import argparse
import warnings
import scripts.utils_vis as vis
import matplotlib.pyplot as plt
import dill
from bisect import bisect_right
import json
import hashlib
import os
from typing import List, Tuple

import re
from typing import Optional, Iterable, Set, Dict, Any, Tuple, List
import bisect


class LaneSelector:
    """
    Sample-level lane/edge filter with optional position and XY windows.

    What it can filter (all optional, combined with AND/OR rules):
    1) Lane/Edge allow/deny by exact ID, prefix, or regex (case-insensitive).
    2) Position windows on lanes/edges: e.g., drop samples with pos in [0, 120] on a given edge.
       (SUMO FCD 'pos' is along the current lane's shape.)
    3) XY regions: rectangles or circles in (x,y) world coordinates (drop samples inside).

    Evaluation:
      - If any *include* list exists, a sample must match at least one include rule.
      - After include passes (or if not set), any *exclude* rule will veto the sample.
      - Position/XY windows are additional vetoes (if a window matches, sample is dropped).
      - Internal lanes (IDs starting with ':') can be dropped via 'drop_internal'.

    Typical usage:
      selector = LaneSelector(
          include_edge_prefixes=['I24_EB_', 'I24_WB_'],           # keep mainline
          exclude_edge_regexes=[r'(ramp|onramp|offramp|link)'],   # drop ramps & links
          drop_internal=True
      )
      # Also drop first 120 m on downstream mainline edges (transition segment)
      selector.add_edge_pos_window(r'^I24_EB_main_\d+$', 0.0, 120.0, regex=True)
      # Or drop a gore area by XY box:
      selector.add_xy_rect(xmin=680.0, ymin=-60.0, xmax=740.0, ymax=-10.0)
    """

    # ---------- base ID rules ----------
    def __init__(
        self,
        include_edges: Optional[Iterable[str]] = None,
        include_edge_prefixes: Optional[Iterable[str]] = None,
        include_edge_regexes: Optional[Iterable[str]] = None,
        include_lanes: Optional[Iterable[str]] = None,
        include_lane_prefixes: Optional[Iterable[str]] = None,
        include_lane_regexes: Optional[Iterable[str]] = None,
        exclude_edges: Optional[Iterable[str]] = None,
        exclude_edge_prefixes: Optional[Iterable[str]] = None,
        exclude_edge_regexes: Optional[Iterable[str]] = None,
        exclude_lanes: Optional[Iterable[str]] = None,
        exclude_lane_prefixes: Optional[Iterable[str]] = None,
        exclude_lane_regexes: Optional[Iterable[str]] = None,
        drop_internal: bool = True,
    ):
        self.inc_edges = set(include_edges or [])
        self.inc_edge_pfx = tuple(include_edge_prefixes or [])
        self.inc_edge_re = [re.compile(pat, re.I) for pat in (include_edge_regexes or [])]

        self.inc_lanes = set(include_lanes or [])
        self.inc_lane_pfx = tuple(include_lane_prefixes or [])
        self.inc_lane_re = [re.compile(pat, re.I) for pat in (include_lane_regexes or [])]

        self.exc_edges = set(exclude_edges or [])
        self.exc_edge_pfx = tuple(exclude_edge_prefixes or [])
        self.exc_edge_re = [re.compile(pat, re.I) for pat in (exclude_edge_regexes or [])]

        self.exc_lanes = set(exclude_lanes or [])
        self.exc_lane_pfx = tuple(exclude_lane_prefixes or [])
        self.exc_lane_re = [re.compile(pat, re.I) for pat in (exclude_lane_regexes or [])]

        self.drop_internal = drop_internal
        self._has_include = any([
            self.inc_edges, self.inc_edge_pfx, self.inc_edge_re,
            self.inc_lanes, self.inc_lane_pfx, self.inc_lane_re
        ])

        # ---------- position-window rules (per lane/edge) ----------
        # store as specs for cache signature + compiled for fast matching
        self._lane_pos_specs: List[Dict[str, Any]] = []   # [{pattern, regex, windows:[(a,b),...]}, ...]
        self._edge_pos_specs: List[Dict[str, Any]] = []
        self._lane_pos_rules: List[Tuple[Optional[re.Pattern], Optional[str], List[Tuple[float, float]]]] = []
        self._edge_pos_rules: List[Tuple[Optional[re.Pattern], Optional[str], List[Tuple[float, float]]]] = []

        # ---------- XY region rules ----------
        self._rects: List[Tuple[float, float, float, float]] = []  # (xmin,ymin,xmax,ymax)
        self._circles: List[Tuple[float, float, float]] = []       # (cx,cy,radius)

    # ---------- helpers ----------
    @staticmethod
    def edge_from_lane(lane_id: str) -> str:
        if not lane_id:
            return ""
        if lane_id.startswith(":"):  # internal/junction
            return lane_id
        parts = lane_id.rsplit("_", 1)
        return parts[0] if len(parts) == 2 else lane_id

    def _match_any(self, s: str, exact: Set[str], pfx: Tuple[str, ...], regs: Iterable[re.Pattern]) -> bool:
        if s in exact:
            return True
        if pfx and any(s.startswith(p) for p in pfx):
            return True
        if regs and any(r.search(s) for r in regs):
            return True
        return False

    # ---------- configure position windows ----------
    def add_lane_pos_window(self, lane_id_or_regex: str, pos_min: float, pos_max: float, regex: bool = False):
        """Drop samples on a lane when pos in [pos_min, pos_max]."""
        spec = {"pattern": lane_id_or_regex, "regex": bool(regex), "windows": [(float(pos_min), float(pos_max))]}
        self._lane_pos_specs.append(spec)
        if regex:
            self._lane_pos_rules.append((re.compile(lane_id_or_regex, re.I), None, spec["windows"]))
        else:
            self._lane_pos_rules.append((None, lane_id_or_regex, spec["windows"]))

    def add_edge_pos_window(self, edge_id_or_regex: str, pos_min: float, pos_max: float, regex: bool = False):
        """Drop samples on an edge when pos in [pos_min, pos_max] (pos is along current lane on that edge)."""
        spec = {"pattern": edge_id_or_regex, "regex": bool(regex), "windows": [(float(pos_min), float(pos_max))]}
        self._edge_pos_specs.append(spec)
        if regex:
            self._edge_pos_rules.append((re.compile(edge_id_or_regex, re.I), None, spec["windows"]))
        else:
            self._edge_pos_rules.append((None, edge_id_or_regex, spec["windows"]))

    # ---------- configure XY regions ----------
    def add_xy_rect(self, xmin: float, ymin: float, xmax: float, ymax: float):
        """Drop samples whose (x,y) lies inside this rectangle (inclusive)."""
        x0, y0, x1, y1 = float(xmin), float(ymin), float(xmax), float(ymax)
        if x0 > x1: x0, x1 = x1, x0
        if y0 > y1: y0, y1 = y1, y0
        self._rects.append((x0, y0, x1, y1))

    def add_xy_circle(self, cx: float, cy: float, radius: float):
        """Drop samples within radius of (cx,cy)."""
        self._circles.append((float(cx), float(cy), float(radius)))

    # ---------- evaluation ----------
    def allow(self, lane_id: Optional[str], pos: Optional[float] = None,
              x: Optional[float] = None, y: Optional[float] = None) -> bool:
        """Return True if this sample (lane_id, pos, x, y) should be kept."""
        if not lane_id:
            return False
        if self.drop_internal and lane_id.startswith(":"):
            return False

        edge_id = self.edge_from_lane(lane_id)

        # include gate
        if self._has_include:
            inc = (
                self._match_any(edge_id, self.inc_edges, self.inc_edge_pfx, self.inc_edge_re)
                or self._match_any(lane_id, self.inc_lanes, self.inc_lane_pfx, self.inc_lane_re)
            )
            if not inc:
                return False

        # exclude veto
        if (
            self._match_any(edge_id, self.exc_edges, self.exc_edge_pfx, self.exc_edge_re)
            or self._match_any(lane_id, self.exc_lanes, self.exc_lane_pfx, self.exc_lane_re)
        ):
            return False

        # position windows (drop if inside any)
        if pos is not None:
            # lane-level windows
            for rex, exact, windows in self._lane_pos_rules:
                hit = (rex.search(lane_id) if rex else (lane_id == exact))
                if hit:
                    for a, b in windows:
                        if a <= pos <= b:
                            return False
            # edge-level windows
            for rex, exact, windows in self._edge_pos_rules:
                hit = (rex.search(edge_id) if rex else (edge_id == exact))
                if hit:
                    for a, b in windows:
                        if a <= pos <= b:
                            return False

        # XY regions (drop if inside)
        if x is not None and y is not None:
            for (x0, y0, x1, y1) in self._rects:
                if (x0 <= x <= x1) and (y0 <= y <= y1):
                    return False
            for (cx, cy, r) in self._circles:
                dx, dy = (x - cx), (y - cy)
                if (dx*dx + dy*dy) <= (r*r):
                    return False

        return True

    # ---------- cache signature ----------
    def to_signature_dict(self) -> Dict[str, Any]:
        """Options for cache invalidation (order-insensitive)."""
        def _sorted_list(x):
            if isinstance(x, (list, tuple, set)):
                return sorted(list(x))
            return x
        return {
            "inc_edges": _sorted_list(self.inc_edges),
            "inc_edge_pfx": list(self.inc_edge_pfx),
            "inc_edge_re": [r.pattern for r in self.inc_edge_re],
            "inc_lanes": _sorted_list(self.inc_lanes),
            "inc_lane_pfx": list(self.inc_lane_pfx),
            "inc_lane_re": [r.pattern for r in self.inc_lane_re],
            "exc_edges": _sorted_list(self.exc_edges),
            "exc_edge_pfx": list(self.exc_edge_pfx),
            "exc_edge_re": [r.pattern for r in self.exc_edge_re],
            "exc_lanes": _sorted_list(self.exc_lanes),
            "exc_lane_pfx": list(self.exc_lane_pfx),
            "exc_lane_re": [r.pattern for r in self.exc_lane_re],
            "drop_internal": self.drop_internal,
            "lane_pos_specs": self._lane_pos_specs,   # raw specs are JSON-serializable
            "edge_pos_specs": self._edge_pos_specs,
            "rects": self._rects,
            "circles": self._circles,
        }

    # ---------- optional: build from net.xml by edge 'type' ----------
    @classmethod
    def from_net_xml(
        cls,
        net_xml_path: str,
        include_edge_types: Optional[Iterable[str]] = ("highway.motorway",),
        exclude_edge_types: Optional[Iterable[str]] = ("highway.motorway_link",),
        drop_internal: bool = True,
    ) -> "LaneSelector":
        """
        Build include/exclude sets from net.xml edge 'type' fields (OSM imports).
        """
        inc_edges, exc_edges = set(), set()
        try:
            tree = ET.parse(net_xml_path)
            root = tree.getroot()
            for e in root.findall(".//edge"):
                if e.get("function") == "internal":
                    continue
                eid = e.get("id")
                etype = e.get("type")
                if not eid:
                    continue
                if include_edge_types and etype in set(include_edge_types):
                    inc_edges.add(eid)
                if exclude_edge_types and etype in set(exclude_edge_types):
                    exc_edges.add(eid)
        except Exception as ex:
            print(f"[LaneSelector] WARN: failed to parse net.xml '{net_xml_path}': {ex}")
        return cls(include_edges=inc_edges, exclude_edges=exc_edges, drop_internal=drop_internal)


def _file_sig(paths: List[str]) -> Tuple[str, int]:
    """Build a simple signature (hash, count) from existing files based on mtime+size."""
    h = hashlib.sha1()
    n = 0
    for p in paths:
        if p and os.path.isfile(p):
            st = os.stat(p)
            h.update(str(st.st_mtime).encode())
            h.update(str(st.st_size).encode())
            n += 1
    return h.hexdigest(), n

def _abs_or_join(base_dir: str, path: str) -> str:
    """
    Return an absolute path. If `path` is already absolute, keep it;
    otherwise join it with `base_dir`.
    """
    if not path:
        return path
    return path if os.path.isabs(path) else os.path.join(base_dir, path)

def _selector_signature(lane_selector) -> dict | None:
    """
    Build a JSON-serializable signature of the LaneSelector so that changes
    to filtering rules invalidate the cache. Tries to use a dedicated method
    if present; otherwise extracts common attributes.
    """
    if lane_selector is None:
        return None

    # If you implemented a custom signature on the class, use it.
    if hasattr(lane_selector, "to_signature_dict"):
        try:
            return lane_selector.to_signature_dict()
        except Exception:
            pass

    # Fallback: read typical attributes (keep it stable & sorted for hashing)
    def _sorted_list(v):
        v = v or []
        try:
            return sorted(list(v))
        except Exception:
            return list(v)

    sig = {
        "drop_internal": bool(getattr(lane_selector, "drop_internal", False)),
        "exclude_lanes": _sorted_list(getattr(lane_selector, "exclude_lanes", [])),
        "exclude_lane_prefixes": _sorted_list(getattr(lane_selector, "exclude_lane_prefixes", [])),
        "exclude_lane_regexes": _sorted_list(getattr(lane_selector, "exclude_lane_regexes", [])),
        "include_lanes": _sorted_list(getattr(lane_selector, "include_lanes", [])),
        "include_lane_prefixes": _sorted_list(getattr(lane_selector, "include_lane_prefixes", [])),
        "include_lane_regexes": _sorted_list(getattr(lane_selector, "include_lane_regexes", [])),
        # Position windows / spatial masks (keep as-is; must be JSON-serializable in your LaneSelector)
        "edge_pos_windows": list(getattr(lane_selector, "edge_pos_windows", [])),
        "lane_pos_windows": list(getattr(lane_selector, "lane_pos_windows", [])),
        "xy_rects": list(getattr(lane_selector, "xy_rects", [])),
        "xy_circles": list(getattr(lane_selector, "xy_circles", [])),
    }
    return sig

def load_or_build_metrics(
    file_dir: str,
    urban_mode: bool = True,
    lane_selector=None,
    strict_vehicle_filter: str = "sample",
    penetration_tag: str | None = None,
):
    """
    Load metrics cache if present and valid; otherwise build from scratch.

    Cache invalidation (signature) is based on:
      - Input files' mtime + size (FCD, and detector/TLS files when urban_mode=True)
      - Urban mode flag
      - LaneSelector filtering rules
      - strict_vehicle_filter mode (e.g., 'sample')

    Parameters
    ----------
    file_dir : str
        Directory that contains SUMO output files (FCD, stats, etc.)
    urban_mode : bool, default True
        If True, include detector/TLS files in cache signature and enable urban metrics.
    lane_selector : LaneSelector | None
        Sample-level filter. Pass the same object you use in the dashboard/main.
    strict_vehicle_filter : {'sample','vehicle','none'}, default 'sample'
        Filtering mode. 'sample' means drop only the samples on excluded lanes/windows.

    Returns
    -------
    TrafficMetrics
        A metrics object with parsed data; cached when possible.
    """
    # Build penetration tag for filenames (default kept for backward compatibility)
    pen_tag = penetration_tag if penetration_tag is not None else "p0"

    # Cache file name (use penetration tag so caches for different penetrations don't collide)
    pkl = os.path.join(file_dir, f"metrics_{pen_tag}.pkl")

    # ---- Build file list for signature (only existing files affect the hash) ----
    # FCD path (change to your actual file name if different)
    fcd_path = os.path.join(file_dir, f"fcd_{pen_tag}.xml")
    files_for_sig = [fcd_path]

    if urban_mode:
        # Use absolute paths as-is; join relative ones to file_dir.
        det_path = _abs_or_join(file_dir, "sumo_scenarios/roosevelt/detector_output.xml")
        tls_path = _abs_or_join(file_dir, "sumo_scenarios/roosevelt/tls.xml")
        files_for_sig.extend([det_path, tls_path])

    # File signature (mtime + size)
    file_sig, _ = _file_sig(files_for_sig)

    # Config signature (urban flag + filtering configuration)
    cfg = {
        "urban_mode": bool(urban_mode),
        "strict_vehicle_filter": str(strict_vehicle_filter or "sample"),
        "lane_selector": _selector_signature(lane_selector),
    }
    cfg_json = json.dumps(cfg, sort_keys=True, ensure_ascii=False, default=str)
    cfg_sig = hashlib.sha1(cfg_json.encode("utf-8")).hexdigest()

    # Final signature combines files + config
    cur_sig = f"{file_sig}|{cfg_sig}"

    # ---- Try loading cache ----
    if os.path.isfile(pkl):
        try:
            with open(pkl, "rb") as f:
                obj = dill.load(f)
            old_sig = getattr(obj, "cache_sig", None)
            if old_sig == cur_sig:
                print("[CACHE] Loaded metrics.pkl (inputs & filters unchanged).")
                return obj
            else:
                print("[CACHE] metrics.pkl found but signature changed -> rebuild.")
        except Exception as e:
            print(f"[CACHE] Failed to load metrics.pkl, rebuilding. Reason: {e}")

    # ---- Rebuild metrics from scratch ----
    # NOTE: TrafficMetrics __init__ should accept lane_selector and strict_vehicle_filter.
    m = TrafficMetrics(
        file_dir=file_dir,
        save_metrics=False,
        urban_mode=urban_mode,
        lane_selector=lane_selector,
        strict_vehicle_filter=strict_vehicle_filter,
        cache_sig=cur_sig,
        penetration_tag=pen_tag,
    )
    with open(pkl, "wb") as f:
        dill.dump(m, f)
    print(f"[CACHE] Built metrics and saved {os.path.basename(pkl)}")

    return m

EPS = 1e-9  # for half-open intervals [start, end)

def _coalesce_tls_states(time_states):
    """
    Input: list of (t, state) sorted by time; usually 0.1s step.
    Output: list of (t0, t1, state) using half-open intervals [t0, t1).
    """
    if not time_states:
        return []
    out = []
    t0, s0 = time_states[0]
    for t, s in time_states[1:]:
        if s != s0:
            out.append((t0, t, s0))
            t0, s0 = t, s
    # infer last step to close the last segment
    last_t = time_states[-1][0]
    step = (time_states[-1][0] - time_states[-2][0]) if len(time_states) >= 2 else 0.1
    if step <= 0: step = 0.1
    out.append((t0, last_t + step, s0))
    return out

def _build_interval_index(intervals):
    """Build arrays for binary search: starts, ends, states."""
    starts = [a for a, _, _ in intervals]
    ends   = [b for _, b, _ in intervals]
    states = [s for _, _, s in intervals]
    return (starts, ends, states)

def _state_at_time(interval_index, t):
    """Return the state string at time t, or None if not covered by any interval."""
    starts, ends, states = interval_index
    i = bisect_right(starts, t + EPS) - 1
    if 0 <= i < len(starts) and starts[i] - EPS <= t < ends[i] - EPS:
        return states[i]
    return None

def _is_green_char(ch):
    # Keep the same rule already used for "green"
    return ch in ('G', 'g')

def _clipLessThanArray(metric, threshold):    
    metric[metric > threshold] = np.nan
    return metric

def _clipMoreThanArray(metric, threshold):
    metric[metric < threshold] = np.nan
    return metric

def _clipLessThan(metric, threshold):    
    clipped_metric = [nan if x < threshold else x for x in metric]
    all_nan = all(isnan(x) for x in clipped_metric)
    if not all_nan:
        return clipped_metric
    else:
        return metric

def _clipMoreThan(metric, threshold):
    clipped_metric = [nan if x > threshold else x for x in metric]
    all_nan = all(isnan(x) for x in clipped_metric)
    if not all_nan:
        return clipped_metric
    else:
        return metric


class TrafficMetrics:
    def __init__(self, file_dir, save_metrics=True, urban_mode=False,
                 lane_selector=None, strict_vehicle_filter="sample", cache_sig=None,
                 penetration_tag: str | None = None):
        """
        Initialize TrafficMetrics.

        Behavior
        --------
        - If `urban_mode=True`: attempt to load detector/TLS/additional mapping files
          and compute signal-related urban metrics.
        - If `urban_mode=False`: skip urban signal metrics.

        Only comments/docstrings were updated for clarity; runtime behavior is unchanged.
        """
        self.file_dir = file_dir
        self.save_metrics = save_metrics
        self.is_sanitizing = True  
        self.lane_selector = lane_selector
        self.strict_vehicle_filter = strict_vehicle_filter

        # ===== Low memory defaults =====
        self.BUILD_LANE_GRID   = True 
        self.LANE_GRID_STRIDE  = 1     # 0.1 s → 0.5 s 
        self.STORE_SPEED_TRAJ  = True  # Default does not store the entire speed track
        self.WRITE_FUEL_IN_FCD = False  # Do not write back to FCD to avoid whole tree memory

        # --- AUDIT counters ---
        from collections import defaultdict as _dd
        self._audit = _dd(int)   # keys: samples_total, samples_drop_selector, safety_kept, safety_skip, ...

        # --- raw (ungated) lane-change for PET ---
        self.positions_all = {"hdv": _dd(list), "cav": _dd(list)}
        self.timestep_lane_positions_all = defaultdict(lambda: defaultdict(dict))
        self.vehicle_lane_history_all = {"hdv": _dd(list), "cav": _dd(list)}
        self._prev_lane_raw = {}  # veh_id -> last seen lane (raw)
        self.xy_traj = {"hdv": defaultdict(list), "cav": defaultdict(list)}

        # --- PET tunables (highway-friendly defaults) ---
        self.PET_EPS       = getattr(self, "PET_EPS", 0.10)   # s
        self.PET_T_WINDOW  = getattr(self, "PET_T_WINDOW", 25.0)
        self.PET_DT_TOL    = getattr(self, "PET_DT_TOL", 0.12)

        # --- SAFETY gate decoupled from POS (important) ---
        self.SAFETY_USE_POS_GATE = getattr(self, "SAFETY_USE_POS_GATE", False)


        # ----------- Basic inputs -----------
        # Allow caller to specify a penetration tag (e.g., 'p0.1', 'p0.2').
        # Default to 'p0.1' for backward compatibility with loader defaults.
        self.penetration_tag = penetration_tag if penetration_tag is not None else "p0.1"
        fcd_path = os.path.join(file_dir, f"fcd_{self.penetration_tag}.xml")
        stats_path = os.path.join(file_dir, f"stats_{self.penetration_tag}.xml")

        if os.path.isfile(fcd_path):
            self.fcd_file = fcd_path
        else:
            raise FileNotFoundError(f"FCD file not found at {fcd_path}")

        if os.path.isfile(stats_path):
            self.stats_path = stats_path
            self.parse_stats(stats_path)

        # Analysis gates and defaults (overridable by setting same-named attributes)
        self.POS_MIN_ANALYSIS    = getattr(self, "POS_MIN_ANALYSIS", 100.0)
        self.TT_USE_POS_GATE     = getattr(self, "TT_USE_POS_GATE",  True)
        self.ENERGY_USE_POS_GATE = getattr(self, "ENERGY_USE_POS_GATE", True)

        self.TT_MIN = 10.0
        self.TT_MAX = None
        self.V_MAX  = 60.0
        self.ACC_MIN, self.ACC_MAX = -8.0, 8.0
        self.HW_V_EPS = 0.1  # protection for headway denominator

        # Containers
        self.speeds = {"hdv": [], "cav": []}
        self.accelerations = {"hdv": [], "cav": []}
        self.travel_times = {"hdv": {}, "cav": {}}
        self.positions = {"hdv": {}, "cav": {}}
        self.time_headways = {"hdv": [], "cav": []}  
        self.space_gaps = {"hdv": [], "cav": []}  
        self.ttc_values = {"hdv": [], "cav": []} 
        self.lane_change_frequency = {"hdv": [], "cav": []} 
        self.vehicle_lane_history = {"hdv": defaultdict(list), "cav": defaultdict(list)}
 
        self.drac_values = {"hdv": [], "cav": []}
        self.simulation_duration = 0
        self.current_lane = {}
        self.timesteps = []
        self.num_hdvs_per_timestep = []
        self.num_cavs_per_timestep = []
        self.per_trip_fuel_consumption = {"hdv": [], "cav": []}
        self.timestep_lane_occupancy = defaultdict(dict)
        self.vehicle_info = {}
        self.pet_list = {"hdv": [], "cav": []}
        self.accepted_gaps = {"hdv": [], "cav": []} 
        self.timestep_lane_positions = defaultdict(lambda: defaultdict(dict))
        self.first_seen = {"hdv": {}, "cav": {}}
        self.speed_traj = {"hdv": defaultdict(list), "cav": defaultdict(list)}  
        
        self.parse_xml()
        self.compute_pet()
        self.compute_gap_acceptance()

        # Histogram visualization ranges
        self.SPEED_MIN_HIST = getattr(self, "SPEED_MIN_HIST", 0.5)  # remove near-zero spawn/entry noise
        self.SPEED_MAX_HIST = getattr(self, "SPEED_MAX_HIST", 30.0) # visibility upper bound (m/s)
        self.ACC_MAX_HIST   = getattr(self, "ACC_MAX_HIST",   6.0)  # visible acceleration bound (abs)

        self._build_vis_arrays()

        # Data sanitization for small/invalid values
        self._sanitize_small_values() 
        self._sanitize_travel_times()
        self._sanitize_speeds_for_hist()
        self._cap_large_values()
        self._finalize_lane_change_frequency()
        self._build_delay_stats(free_flow_speed_hdv=30.0, free_flow_speed_cav=30.0)

        # ----------- Urban‑mode specific -----------
        if urban_mode:
            mapping_file = os.path.join(file_dir, "sumo_scenarios/roosevelt/detector_mapping.json")
            self.detector_file = os.path.join(file_dir, "sumo_scenarios/roosevelt/detector_output.xml")
            self.tls_file = os.path.join(file_dir, "sumo_scenarios/roosevelt/tls.xml")
            self.add_file = os.path.join(file_dir, "sumo_scenarios/roosevelt/sensors.add.xml") 

            if os.path.exists(mapping_file):
                with open(mapping_file, "r") as f:
                    self.detector_mapping = json.load(f)
            else:
                self.detector_mapping = {}
                print(f"[Warning] Mapping file not found: {mapping_file}")
            
            if os.path.isfile(self.detector_file) and os.path.isfile(self.tls_file):
                self.det2lane = self.parse_add_detectors_to_map(self.add_file)
                self.lane_map = self.load_lane_tls_mapping_json(mapping_file)
                self.detector_mapping = self._compose_detector_mapping(self.det2lane, self.lane_map)

                self.parse_detector_data(self.detector_file)
                self.parse_tls_data(self.tls_file)
                self._run_city_signal_metrics_if_applicable()
            else:
                print("[Urban] Detector/TLS file not found, skipping signal metrics.")
                self.detector_file, self.tls_file, self.add_file = None, None, None
        else:
            print("[Init] Urban mode disabled, skipping detector/tls/add files.")
            self.detector_file, self.tls_file, self.add_file = None, None, None
   
        # ----------- Cache signature -----------
        if cache_sig is None:
            cur_sig, _ = _file_sig([self.fcd_file, getattr(self, "detector_file", None), getattr(self, "tls_file", None)])
            self.cache_sig = cur_sig
        else:
            self.cache_sig = cache_sig

    def _cap_large_values(self):

        TTC_MAX_KEEP = float(getattr(self, "TTC_MAX_KEEP", 30.0))  # s
        HW_MAX_KEEP  = float(getattr(self, "HW_MAX_KEEP",  8.0))   # s
        SG_X_MAX     = float(getattr(self, "SG_X_MAX",    80.0))   # m

        def _clip_max(arr, m):
            return [x for x in arr if (x is not None and np.isfinite(x) and x <= m)]

        for k in ("hdv", "cav"):
            if hasattr(self, "ttc_values") and k in self.ttc_values:
                self.ttc_values[k] = _clip_max(self.ttc_values[k], TTC_MAX_KEEP)
            if hasattr(self, "time_headways") and k in self.time_headways:
                self.time_headways[k] = _clip_max(self.time_headways[k], HW_MAX_KEEP)
            if hasattr(self, "space_gaps") and k in self.space_gaps:
                self.space_gaps[k] = _clip_max(self.space_gaps[k], SG_X_MAX)
     
    def parse_xml(self):
        """
        Low-memory streaming parsing of FCD:
        - Iterate over <timestep>/<vehicle> with ElementTree.iterparse
        - Keep only small per-step structures in memory
        - Store BOTH raw(*_all) and gated containers (after LaneSelector)
        """

        import xml.etree.ElementTree as ET
        import numpy as np
        from collections import defaultdict

        # ===== Tunable gates & switches =====
        POS_MIN_ANALYSIS    = float(getattr(self, "POS_MIN_ANALYSIS", 100.0))
        TT_USE_POS_GATE     = bool(getattr(self, "TT_USE_POS_GATE", True))
        ENERGY_USE_POS_GATE = bool(getattr(self, "ENERGY_USE_POS_GATE", True))
        SAFETY_USE_POS_GATE = bool(getattr(self, "SAFETY_USE_POS_GATE", False))

        V_MAX   = float(getattr(self, "V_MAX", 60.0))
        ACC_MIN = float(getattr(self, "ACC_MIN", -8.0))
        ACC_MAX = float(getattr(self, "ACC_MAX",  8.0))
        HW_V_EPS= float(getattr(self, "HW_V_EPS", 1e-4))

        BUILD_LANE_GRID   = bool(getattr(self, "BUILD_LANE_GRID", bool(getattr(self, "urban_mode", False))))
        LANE_GRID_STRIDE  = int(getattr(self, "LANE_GRID_STRIDE", 1))
        STORE_SPEED_TRAJ  = bool(getattr(self, "STORE_SPEED_TRAJ", True))
        STORE_XY_TRAJ     = bool(getattr(self, "STORE_XY_TRAJ", True))
        WRITE_FUEL_IN_FCD = bool(getattr(self, "WRITE_FUEL_IN_FCD", False))

        # ===== Containers (ensure existence) =====
        self.timesteps = []
        self.num_hdvs_per_timestep = []
        self.num_cavs_per_timestep = []

        if not hasattr(self, "positions"):     self.positions = {"hdv": {}, "cav": {}}
        if not hasattr(self, "positions_all"): self.positions_all = {"hdv": defaultdict(list), "cav": defaultdict(list)}
        if not hasattr(self, "speed_traj"):
            self.speed_traj = {"hdv": defaultdict(list), "cav": defaultdict(list)} if STORE_SPEED_TRAJ else {"hdv": {}, "cav": {}}
        if not hasattr(self, "xy_traj"):
            self.xy_traj = {"hdv": defaultdict(list), "cav": defaultdict(list)} if STORE_XY_TRAJ else {"hdv": {}, "cav": {}}

        self.speeds        = {"hdv": [], "cav": []}
        self.accelerations = {"hdv": [], "cav": []}
        self.space_gaps    = {"hdv": [], "cav": []}
        self.time_headways = {"hdv": [], "cav": []}
        self.ttc_values    = {"hdv": [], "cav": []}
        self.drac_values   = {"hdv": [], "cav": []}

        if not hasattr(self, "vehicle_lane_history"):     self.vehicle_lane_history     = {"hdv": defaultdict(list), "cav": defaultdict(list)}
        if not hasattr(self, "vehicle_lane_history_all"): self.vehicle_lane_history_all = {"hdv": defaultdict(list), "cav": defaultdict(list)}
        if not hasattr(self, "_prev_lane_raw"):           self._prev_lane_raw = {}

        self.timestep_lane_occupancy = {}    # gated
        self.timestep_lane_positions = {}    # gated
        if not hasattr(self, "timestep_lane_positions_all"):
            self.timestep_lane_positions_all = defaultdict(lambda: defaultdict(dict))  # raw

        self.first_seen     = {"hdv": {}, "cav": {}}
        vehicle_last_seen   = {"hdv": {}, "cav": {}}

        if not hasattr(self, "vehicle_info") or not isinstance(self.vehicle_info, dict):
            self.vehicle_info = {}
        if not hasattr(self, "current_lane") or not isinstance(self.current_lane, dict):
            self.current_lane = {}

        vehicle_fuel_consumption = {"hdv": {}, "cav": {}}
        vehicle_distance         = {"hdv": {}, "cav": {}}

        # Per-step state
        cur_time  = None
        prev_time = None
        dt_step   = 0.1
        step_idx  = -1
        first_time = None
        last_time  = None

        # per-step containers: raw & gated
        lane_occ_g  = None
        lane_pos_g  = None
        lane_pos_all = None
        present_hdv_g = None
        present_cav_g = None

        # ==== streaming parse ====
        context = ET.iterparse(self.fcd_file, events=("start", "end"))
        context = iter(context)
        try:
            _ev, root = next(context)  # for periodic root.clear()
        except StopIteration:
            raise RuntimeError(f"Empty or invalid FCD XML: {self.fcd_file}")

        CLEAR_ROOT_EVERY = 64

        for event, elem in context:
            tag = elem.tag.split('}')[-1]

            # -- timestep start
            if tag == "timestep" and event == "start":
                cur_time = float(elem.get("time"))
                step_idx += 1
                if first_time is None:
                    first_time = cur_time
                if prev_time is not None:
                    dt_step = max(1e-9, cur_time - prev_time)
                else:
                    dt_step = 0.1
                last_time = cur_time

                from collections import defaultdict as _dd
                lane_occ_g   = _dd(set)
                lane_pos_g   = _dd(dict)
                lane_pos_all = _dd(dict)
                present_hdv_g = set()
                present_cav_g = set()

            # -- vehicle end
            elif tag == "vehicle" and event == "end":
                try:
                    vid   = elem.get("id")
                    vtype = (elem.get("type") or "hdv").lower()
                    lane  = elem.get("lane") or ""
                    pos   = float(elem.get("pos"))
                    x     = float(elem.get("x"))
                    y     = float(elem.get("y", "nan"))
                    v_raw = float(elem.get("speed"))
                    a_raw = float(elem.get("acceleration"))
                    lgap  = float(elem.get("leaderGap", "0"))
                    ls_raw= float(elem.get("leaderSpeed", "-1"))
                except Exception:
                    elem.clear()
                    continue

                # ---------- RAW: prior to any filtering ----------
                # Vehicle type registration (raw also needs it, otherwise downstream _all cannot find the type)
                if vid not in self.vehicle_info:
                    self.vehicle_info[vid] = {"type": vtype}

                self.positions_all.setdefault(vtype, {}).setdefault(vid, []).append((cur_time, pos))
                if lane and not lane.startswith(":J"):
                    lane_pos_all[lane][vid] = pos

                prev_raw = self._prev_lane_raw.get(vid)
                if prev_raw is None:
                    self._prev_lane_raw[vid] = lane
                elif lane != prev_raw:
                    self.vehicle_lane_history_all[vtype][vid].append((cur_time, prev_raw, lane))
                    self._prev_lane_raw[vid] = lane

                # ---------- Sample-level lane selector（gated pipeline） ----------
                if (self.lane_selector is not None) and (self.strict_vehicle_filter == "sample"):
                    if not self.lane_selector.allow(lane, pos=pos, x=x, y=y):
                        elem.clear()
                        continue  # Skip gated statistics, but RAW has already been recorded.

                # gated presence sets
                (present_hdv_g if vtype == "hdv" else present_cav_g).add(vid)

                # clamp/clean
                v  = min(max(v_raw, 0.0), V_MAX)
                a  = min(max(a_raw, ACC_MIN), ACC_MAX)
                ls = None if ls_raw < 0 else min(max(ls_raw, 0.0), V_MAX)
                lg = max(0.0, lgap)

                # first/last seen（gated TT)
                if vid not in self.first_seen[vtype]:
                    if TT_USE_POS_GATE:
                        if pos >= POS_MIN_ANALYSIS:
                            self.first_seen[vtype][vid] = cur_time
                    else:
                        self.first_seen[vtype][vid] = cur_time
                vehicle_last_seen[vtype][vid] = cur_time

                # trajectories (gated)
                self.positions.setdefault(vtype, {}).setdefault(vid, []).append((cur_time, pos))
                if STORE_SPEED_TRAJ:
                    self.speed_traj[vtype][vid].append((cur_time, v))
                if STORE_XY_TRAJ:
                    self.xy_traj.setdefault(vtype, {}).setdefault(vid, []).append((cur_time, x, y))

                # unified gates
                passed_pos_gate = (pos >= POS_MIN_ANALYSIS)
                passed_safety   = (pos >= POS_MIN_ANALYSIS) if SAFETY_USE_POS_GATE else True

                # distributions & safety (gated)
                if passed_safety:
                    self.speeds[vtype].append(v)
                    self.accelerations[vtype].append(a)
                    self.space_gaps[vtype].append(lg)
                    self.time_headways[vtype].append(min(999.0, lg / max(HW_V_EPS, v)))
                    if (ls is not None) and (v > ls):
                        ttc = lg / max(HW_V_EPS, (v - ls))
                        if np.isfinite(ttc) and ttc >= 0:
                            self.ttc_values[vtype].append(ttc)
                    if lg > 1.0 and (ls is not None):
                        rel_v = v - ls
                        drac = (rel_v * rel_v) / (2.0 * lg)
                        if np.isfinite(drac):
                            self.drac_values[vtype].append(drac)

                # lane grid (gated)
                if lane and not lane.startswith(":J"):
                    lane_occ_g[lane].add(vid)
                    lane_pos_g[lane][vid] = pos
                    # gated lane-change
                    prev_lane = self.current_lane.get(vid)
                    if prev_lane is None:
                        self.current_lane[vid] = lane
                    elif lane != prev_lane:
                        self.current_lane[vid] = lane
                        if passed_safety:
                            self.vehicle_lane_history[vtype][vid].append((cur_time, prev_lane, lane))

                # fuel/distance (gated， dt_step)
                gps = max(0.0, self.fuel_rate_g_per_s(v, a, vtype))
                if (not ENERGY_USE_POS_GATE) or passed_pos_gate:
                    vehicle_fuel_consumption[vtype][vid] = vehicle_fuel_consumption[vtype].get(vid, 0.0) + gps * dt_step
                    vehicle_distance[vtype][vid]         = vehicle_distance[vtype].get(vid, 0.0) + v * dt_step

                elem.clear()

            # -- timestep end
            elif tag == "timestep" and event == "end":
                # time axis + presence (gated)
                self.timesteps.append(cur_time)
                self.num_hdvs_per_timestep.append(len(present_hdv_g or ()))
                self.num_cavs_per_timestep.append(len(present_cav_g or ()))

                if BUILD_LANE_GRID and (step_idx % LANE_GRID_STRIDE == 0):
                    # gated snapshots
                    self.timestep_lane_occupancy[cur_time] = {ln: set(vs) for ln, vs in (lane_occ_g or {}).items()}
                    self.timestep_lane_positions[cur_time] = {ln: dict(vmap) for ln, vmap in (lane_pos_g or {}).items()}
                    # RAW snapshots
                    self.timestep_lane_positions_all[cur_time] = {ln: dict(vmap) for ln, vmap in (lane_pos_all or {}).items()}

                elem.clear()
                prev_time = cur_time

                if (step_idx % CLEAR_ROOT_EVERY) == 0:
                    try:
                        root.clear()
                    except Exception:
                        pass

        # ==== Finalize ====
        self.simulation_duration = 0.0 if first_time is None else (last_time - first_time)
        self.num_hdvs = len(self.first_seen["hdv"])
        self.num_cavs = len(self.first_seen["cav"])

        # travel times
        self.travel_times = {"hdv": {}, "cav": {}}
        for vt in ("hdv", "cav"):
            for vid, t0 in self.first_seen[vt].items():
                t1 = vehicle_last_seen[vt].get(vid)
                if t1 is not None:
                    self.travel_times[vt][vid] = t1 - t0

        # fuel stats
        GRAMS_PER_GALLON = 2791.0
        per_trip = {"hdv": list(vehicle_fuel_consumption["hdv"].values()),
                    "cav": list(vehicle_fuel_consumption["cav"].values())}
        self.per_trip_fuel_consumption = per_trip

        total_fuel_g = {"hdv": sum(per_trip["hdv"]), "cav": sum(per_trip["cav"])}
        total_miles  = {"hdv": 0.0, "cav": 0.0}
        for vt in ("hdv", "cav"):
            for _, dist_m in vehicle_distance[vt].items():
                total_miles[vt] += dist_m * 0.000621371

        avg_fuel = {"hdv": (np.mean(per_trip["hdv"]) if per_trip["hdv"] else 0.0),
                    "cav": (np.mean(per_trip["cav"]) if per_trip["cav"] else 0.0)}
        fe_gpm   = {"hdv": (total_fuel_g["hdv"]/GRAMS_PER_GALLON)/total_miles["hdv"] if total_miles["hdv"] > 0 else 0.0,
                    "cav": (total_fuel_g["cav"]/GRAMS_PER_GALLON)/total_miles["cav"] if total_miles["cav"] > 0 else 0.0}

    # Correct formula: mpg = total_miles / (total_fuel_g / GRAMS_PER_GALLON)
        fe_mpg = {"hdv": (total_miles["hdv"]*GRAMS_PER_GALLON)/total_fuel_g["hdv"] if total_fuel_g["hdv"] > 0 else 0.0,
            "cav": (total_miles["cav"]*GRAMS_PER_GALLON)/total_fuel_g["cav"] if total_fuel_g["cav"] > 0 else 0.0}


        savings  = ((fe_gpm["cav"] - fe_gpm["hdv"])/fe_gpm["hdv"]*100.0) if fe_gpm["hdv"] > 0 else 0.0

        fe_mpg_savings = ((fe_mpg["cav"] - fe_mpg["hdv"])/fe_mpg["hdv"]*100.0) if fe_mpg["hdv"] > 0 else 0.0

        if not hasattr(self, "simulation_stats"):
            self.simulation_stats = {}
        self.simulation_stats["fuel"] = {
            "hdv_total_fuel_g": round(total_fuel_g["hdv"], 2),
            "cav_total_fuel_g": round(total_fuel_g["cav"], 2),
            "hdv_avg_fuel_per_vehicle_g": round(avg_fuel["hdv"], 2),
            "cav_avg_fuel_per_vehicle_g": round(avg_fuel["cav"], 2),
            "hdv_fuel_efficiency_gpm": round(fe_gpm["hdv"], 4),
            "cav_fuel_efficiency_gpm": round(fe_gpm["cav"], 4),
            "hdv_fuel_efficiency_mpg": round(fe_mpg["hdv"], 4),
            "cav_fuel_efficiency_mpg": round(fe_mpg["cav"],4),
            "cav_fuel_savings_percent": round(savings, 2),
            "cav_fuel_efficiency_mpg_savings_percent": round(fe_mpg_savings, 2),
        }

        if WRITE_FUEL_IN_FCD:
            print("[WARN] WRITE_FUEL_IN_FCD=True is not supported in streaming mode (would re-materialize the tree).")
 
    def _finalize_lane_change_frequency(self):
        """
        Aggregate lane change frequency arrays:
        - self.lane_change_frequency       ← Based on gated vehicle_lane_history
        - self.lane_change_frequency_raw ← Based on RAW vehicle_lane_history_all
        If gated is empty but RAW contains data, fall back to lane_change_frequency using RAW results to ensure visualization data availability.
        """
        def _count(hist):
            out = {"hdv": [], "cav": []}
            for vt in ("hdv", "cav"):
                for vid, changes in (hist.get(vt, {}) or {}).items():
                    out[vt].append(len(changes))
            return out

        # 1) Calculate lane change frequency for both raw and gated data
        lc_g = _count(getattr(self, "vehicle_lane_history", {}))
        lc_r = _count(getattr(self, "vehicle_lane_history_all", {}))

        self.lane_change_frequency      = lc_g
        self.lane_change_frequency_raw  = lc_r

        # 2) If gated is empty but raw is not, use raw as fallback (does not change your metrics, only ensures visualization availability)
        def _sum(d): return sum((d.get("hdv") or [])) + sum((d.get("cav") or []))
        if _sum(lc_g) == 0 and _sum(lc_r) > 0:
            self.lane_change_frequency = lc_r

    def _build_delay_stats(self, free_flow_speed_hdv=30.0, free_flow_speed_cav=30.0):
        """
        Calculate delays based on travel_times (already computed in parse_xml) 
        + distance from velocity trajectory integration, then write to 
        
        self.simulation_stats['delay']. The field structure remains consistent with the previous version, 
        allowing seamless integration with your existing visualizations.
        """

        
        def _distance_from_speed(vt, vid, t0, t1):
            seq = (getattr(self, "speed_traj", {}).get(vt, {}) or {}).get(vid, [])
            if not seq or len(seq) < 2:
                return None
            seq = sorted(((float(t), float(v)) for (t, v) in seq), key=lambda z: z[0])
            dist = 0.0
            for (ti, vi), (tj, vj) in zip(seq[:-1], seq[1:]):
                a = max(ti, t0); b = min(tj, t1)
                if b <= a:
                    continue
                
                dist += max(0.0, vi) * (b - a)
            return dist

        totals_delay = {"hdv": 0.0, "cav": 0.0}
        totals_miles = {"hdv": 0.0, "cav": 0.0}
        vehicle_delays = {"hdv": {}, "cav": {}}

        for vt in ("hdv", "cav"):
            ff = free_flow_speed_cav if vt == "cav" else free_flow_speed_hdv
            t_first = self.first_seen.get(vt, {})
            tt_map  = self.travel_times.get(vt, {})

            for vid, travel_time in tt_map.items():
                
                t0 = t_first.get(vid, None)
                if t0 is None:
                    continue
                t1 = t0 + float(travel_time)

                dist_m = _distance_from_speed(vt, vid, t0, t1)
                if (dist_m is None) or not np.isfinite(dist_m):
                    traj = (self.positions.get(vt, {}) or {}).get(vid, [])
                    if traj:

                        dist_m = max(0.0, float(traj[-1][1]))
                    else:
                        dist_m = 0.0

                miles = dist_m * 0.000621371
                ff_time = dist_m / max(1e-6, ff)  # s
                delay = max(0.0, float(travel_time) - ff_time)

                vehicle_delays[vt][vid] = delay
                totals_delay[vt] += delay
                totals_miles[vt] += miles

        delay_per_mile = {
            "hdv": (totals_delay["hdv"] / totals_miles["hdv"]) if totals_miles["hdv"] > 0 else 0.0,
            "cav": (totals_delay["cav"] / totals_miles["cav"]) if totals_miles["cav"] > 0 else 0.0,
        }

        if not hasattr(self, "simulation_stats") or not isinstance(self.simulation_stats, dict):
            self.simulation_stats = {}
        self.simulation_stats["delay"] = {
            "hdv_total_delay": round(totals_delay["hdv"], 1),
            "hdv_total_miles": round(totals_miles["hdv"], 2),
            "hdv_delay_per_mile": round(delay_per_mile["hdv"], 1),
            "cav_total_delay": round(totals_delay["cav"], 1),
            "cav_total_miles": round(totals_miles["cav"], 2),
            "cav_delay_per_mile": round(delay_per_mile["cav"], 1),
            "vehicle_delays": vehicle_delays,   
        }

    def _sanitize_small_values(self):
        """
        Remove numeric artifacts such as zeros/unrealistically small values.
        Thresholds are conservative defaults; adjust if needed.
        """
        # ---- Conservative physical lower bounds (robust defaults) ----
        SG_X_MAX = 80.0            # m   (kept as a reference upper cap for spacing if needed)
        MIN_SPACE_GAP_M   = 1.0    # m   space gaps < 0.5 m are typically artifacts
        MIN_HEADWAY_S     = 0.2    # s   headway < 0.2 s is unrealistic
        MIN_PET_S         = 0.10   # s   PET < 0.1 s is usually interpolation jitter
        MIN_TTC_S         = 0.05   # s   extremely small TTC often comes from numeric noise

        def _flt(arr, min_val):
            return [x for x in arr if (x is not None and not np.isnan(x) and x >= min_val)]

        # Clean space gap / headway / PET / TTC for both HDV and CAV
        for k in ("hdv", "cav"):
            # Space gap
            if hasattr(self, "space_gaps") and k in self.space_gaps:
                self.space_gaps[k] = _flt(self.space_gaps[k], MIN_SPACE_GAP_M)

            # Time headway
            if hasattr(self, "time_headways") and k in self.time_headways:
                self.time_headways[k] = _flt(self.time_headways[k], MIN_HEADWAY_S)

            # PET
            if hasattr(self, "pet_list") and k in self.pet_list:
                self.pet_list[k] = _flt(self.pet_list[k], MIN_PET_S)

            # TTC
            if hasattr(self, "ttc_values") and k in self.ttc_values:
                self.ttc_values[k] = _flt(self.ttc_values[k], MIN_TTC_S)

        # Record thresholds for reproducibility
        if not hasattr(self, "sanitization_thresholds"):
            self.sanitization_thresholds = {}
        self.sanitization_thresholds.update({
            "min_space_gap_m": MIN_SPACE_GAP_M,
            "min_headway_s": MIN_HEADWAY_S,
            "min_pet_s": MIN_PET_S,
            "min_ttc_s": MIN_TTC_S,
        })
    
    def _sanitize_travel_times(self):
        def _clean(d):
            out = {}
            for vid, tt in d.items():
                try:
                    val = float(tt)
                except Exception:
                    continue
                if val < getattr(self, "TT_MIN", 0.0):
                    continue
                tt_max = getattr(self, "TT_MAX", None)
                if tt_max is not None and val > tt_max:
                    continue
                out[vid] = val
            return out

        self.travel_times["hdv"] = _clean(self.travel_times.get("hdv", {}))
        self.travel_times["cav"] = _clean(self.travel_times.get("cav", {}))

    def _find_crossing_time(self, pos_A, trajectory):
        """Find linear‑interpolated time when a trajectory crosses position `pos_A`."""
        if not trajectory:
            return None
        times = [t for t, p in trajectory]
        positions = [p for t, p in trajectory]

        # Binary search for crossing point
        low, high = 0, len(positions) - 1
        while low <= high:
            mid = (low + high) // 2
            if positions[mid] < pos_A:
                low = mid + 1
            else:
                high = mid - 1

        if low == 0 or low >= len(positions):
            return None

        i = low - 1
        if positions[i] <= pos_A <= positions[i+1]:
            t1, p1 = times[i], positions[i]
            t2, p2 = times[i+1], positions[i+1]
            delta_p = p2 - p1
            if delta_p == 0:
                return None
            delta_t = t2 - t1
            return t1 + (pos_A - p1) * delta_t / delta_p
        return None

    def _build_vis_arrays(self):
        """Build filtered arrays for visualization (histograms)."""

        vmin = float(getattr(self, "SPEED_MIN_HIST", 0.5))
        vmax = getattr(self, "SPEED_MAX_HIST", None)
        amax = getattr(self, "ACC_MAX_HIST", None)

        def _keep_v(v):
            if not np.isfinite(v): return False
            if v < vmin: return False
            if (vmax is not None) and (v > vmax): return False
            return True

        def _keep_a(a):
            if not np.isfinite(a): return False
            if (amax is not None) and (abs(a) > amax): return False
            return True

        # Speed histogram arrays: only threshold on speed
        self.speeds_vis = {
            "hdv": [v for v in (self.speeds.get("hdv", []) or []) if _keep_v(v)],
            "cav": [v for v in (self.speeds.get("cav", []) or []) if _keep_v(v)],
        }

        # Acceleration histogram arrays: drop extreme spikes; optionally exclude near‑idle noise
        acc_hdv = [a for a in (self.accelerations.get("hdv", []) or []) if _keep_a(a)]
        acc_cav = [a for a in (self.accelerations.get("cav", []) or []) if _keep_a(a)]
        self.accel_vis = {"hdv": acc_hdv, "cav": acc_cav}

    def debug_pet_pipeline(self, distance_threshold=50.0, dt_tolerance=0.1, max_print=5):
        
        src = getattr(self, "vehicle_lane_history_all", self.vehicle_lane_history)
        n_lc_hdv = sum(len(v) for v in src.get('hdv', {}).values())
        n_lc_cav = sum(len(v) for v in src.get('cav', {}).values())

        n_lc_hdv = sum(len(v) for v in self.vehicle_lane_history.get('hdv', {}).values())
        n_lc_cav = sum(len(v) for v in self.vehicle_lane_history.get('cav', {}).values())
        print(f"[PET/DEBUG] lane-changes (gated) HDV={n_lc_hdv}  CAV={n_lc_cav}")


        counts = dict(changes=0, no_target=0, far=0, no_cross=0, kept=0)
        times = list(getattr(self, "timesteps", []))
        if len(times) < 2:
            print("[PET/DEBUG] no timesteps.")
            return
        dt = times[1] - times[0]

        def nearest_time_idx(t):
            i = bisect.bisect_left(times, t)
            cand = []
            if 0 <= i < len(times): cand.append((abs(times[i]-t), i))
            if i-1 >= 0: cand.append((abs(times[i-1]-t), i-1))
            if i+1 < len(times): cand.append((abs(times[i+1]-t), i+1))
            cand.sort()
            if cand and cand[0][0] <= dt_tolerance:
                return cand[0][1]
            return None

        for vt in ["hdv", "cav"]:
            for vid, changes in src.get(vt, {}).items():
                for (t0, old_lane, new_lane) in changes:
                    counts["changes"] += 1
                    # Target Vehicle Set (on the new_lane near t0)
                    idx = nearest_time_idx(t0)
                    if idx is None:
                        counts["no_target"] += 1
                        continue
                    t_near = times[idx]
                    lane_dict = self.timestep_lane_positions.get(t_near, {})
                    target_veh = set(lane_dict.get(new_lane, {}).keys()) - {vid}
                    if not target_veh:
                        counts["no_target"] += 1
                        continue

                    # location of vehicle A 
                    a_traj = self.positions[vt].get(vid, [])
                    # location of vehicle A at t_near
                    a_pos = None
                    for (tt, pp) in a_traj:
                        if abs(tt - t_near) <= dt_tolerance:
                            a_pos = pp; break
                    if a_pos is None:
                        counts["no_target"] += 1
                        continue

                    # Scan the target vehicles to see which ones qualify as PET.
                    found_one = False
                    for bid in target_veh:
                        btype = self.vehicle_info.get(bid, {}).get("type")
                        if not btype: 
                            continue
                        b_traj = self.positions.get(btype, {}).get(bid, [])
                        # location of vehicle B at t_near
                        b_pos = None
                        for (tt, pp) in b_traj:
                            if abs(tt - t_near) <= dt_tolerance:
                                b_pos = pp; break
                        if (b_pos is None) or (abs(b_pos - a_pos) > distance_threshold):
                            continue
                        # Linear interpolation to find B at the time of a_pos
                        ct = self._find_crossing_time(a_pos, b_traj)
                        if ct is None or ct < t0 - 1e-9:
                            continue
                        pet = ct - t0
                        if 0.1 <= pet <= 25.0:
                            counts["kept"] += 1
                            found_one = True
                            break
                    if not found_one:
                        
                        counts["no_cross"] += 1

        print("[PET/DEBUG] funnel:", counts)

    def compute_pet(self, distance_threshold=60.0):
        """
        Compute PET (Post-Encroachment Time).
        Uses RAW(_all) grids/trajectories when available; falls back to gated.
        """
        import bisect
        times = list(getattr(self, "timesteps", []))
        if len(times) < 2:
            self.pet_list = {"hdv": [], "cav": []}
            return

        dt = times[1] - times[0]
        base_tol = float(getattr(self, "PET_DT_TOL", 0.08))
        dt_tol   = max(base_tol, 0.6 * float(dt))
        eps_pet  = float(getattr(self, "PET_EPS", 0.20))
        t_win    = float(getattr(self, "PET_T_WINDOW", 15.0))
        dist_th  = float(distance_threshold)

        # helpers
        def nearest_idx(t):
            j = bisect.bisect_left(times, t)
            cands = []
            if 0 <= j < len(times):   cands.append((abs(times[j]  - t), j))
            if j-1 >= 0:              cands.append((abs(times[j-1]- t), j-1))
            if j+1 < len(times):      cands.append((abs(times[j+1]- t), j+1))
            cands.sort(key=lambda z: z[0])
            if not cands or cands[0][0] > dt_tol:
                return None
            return cands[0][1]

        pos_all_grid = getattr(self, "timestep_lane_positions_all", None)
        pos_grid_g   = getattr(self, "timestep_lane_positions", {})
        occ_grid_g   = getattr(self, "timestep_lane_occupancy", {})

        def a_traj(vt, vid):
            # prefer RAW, fallback gated
            pa = (getattr(self, "positions_all", {}).get(vt, {}) or {}).get(vid, [])
            if pa: return pa
            return (getattr(self, "positions", {}).get(vt, {}) or {}).get(vid, [])

        self.pet_list = {"hdv": [], "cav": []}
        for vt in ("hdv", "cav"):
            src_changes = (getattr(self, "vehicle_lane_history_all", {}) or {}).get(vt, {})
            for vid, changes in (src_changes or {}).items():
                for (t0, old_lane, new_lane) in changes:
                    if not new_lane or new_lane.startswith(":J"):
                        continue

                    # 1) target set, prefer RAW grid (exact), else RAW nearest neighbor, then fallback gated
                    candidates = set()
                    if pos_all_grid and t0 in pos_all_grid:
                        candidates = set(pos_all_grid[t0].get(new_lane, {}).keys())
                    if not candidates:
                        k = nearest_idx(t0)
                        if (pos_all_grid is not None) and (k is not None):
                            tN = times[k]
                            candidates = set(pos_all_grid.get(tN, {}).get(new_lane, {}).keys())
                    if not candidates:
                        # fall back to gated occupancy / positions
                        occ = occ_grid_g.get(t0, {})
                        candidates = set(occ.get(new_lane, set()))
                        if not candidates:
                            k = nearest_idx(t0)
                            if k is not None:
                                tN = times[k]
                                candidates = set(pos_grid_g.get(tN, {}).get(new_lane, {}).keys())
                    candidates -= {vid}
                    if not candidates:
                        continue

                    # 2) Get A's position (prefer RAW)
                    a_pos = None
                    ATR = a_traj(vt, vid)
                    for (tt, pp) in ATR:
                        if abs(tt - t0) <= dt_tol:
                            a_pos = pp; break
                    if a_pos is None and ATR:
                        # Nearest neighbor interpolation
                        i = nearest_idx(t0)
                        if i is not None:
                            # Simple nearest neighbor (more detailed linear interpolation can be done)
                            for (tt, pp) in ATR:
                                if abs(tt - times[i]) <= dt_tol:
                                    a_pos = pp; break
                    if a_pos is None:
                        continue

                    best_pet = None
                    for b_id in candidates:
                        btype = (self.vehicle_info.get(b_id, {}) or {}).get("type")
                        if not btype:
                            continue
                        BTR = a_traj(btype, b_id)
                        if not BTR:
                            continue
                        # proximity
                        b_pos = None
                        for (tt, pp) in BTR:
                            if abs(tt - t0) <= dt_tol:
                                b_pos = pp; break
                        if b_pos is None or abs(b_pos - a_pos) > dist_th:
                            continue
                        ct = self._find_crossing_time(a_pos, BTR)
                        if ct is None or ct < t0 - 1e-9:
                            continue
                        pet = ct - t0
                        if (pet < eps_pet) or (pet > t_win):
                            continue
                        if (best_pet is None) or (pet < best_pet):
                            best_pet = pet

                    if best_pet is not None:
                        self.pet_list[vt].append(best_pet)
        # —— Data sanitization ——     
        if getattr(self, "is_sanitizing", True):
            for v_type in ("hdv", "cav"):
                arr = np.asarray(self.pet_list[v_type], dtype=float)
                arr = arr[np.isfinite(arr)]
                if arr.size == 0:
                    self.pet_list[v_type] = []
                    continue
                hi = np.percentile(arr, 99.5)
                cap = min(max(15.0, hi), 30.0)
                arr = arr[(arr >= 0.1) & (arr <= cap)]
                self.pet_list[v_type] = arr.tolist()

    def compute_gap_acceptance(self, distance_threshold=60.0, max_gap=30.0):
            """
            Compute the accepted merging gap (in seconds).
            The results are stored in self.accepted_gaps['hdv'] and self.accepted_gaps['cav'].

            - Preferred case: Gap = t_lag - t_lead
            (the time difference between the nearest following and leading vehicle in the target lane
            passing the merge point)
            - Fallback case: If no leader but a following vehicle exists,
            Gap = t_lag - t_merge (the time between the following vehicle reaching the merge point
            and the merge event itself)
            """
            times = list(getattr(self, "timesteps", []))
            if len(times) < 2:
                self.accepted_gaps = {"hdv": [], "cav": []}
                return

            import bisect
            dt_tol = float(getattr(self, "PET_DT_TOL", 0.12))
            eps_gap = float(getattr(self, "PET_EPS", 0.10))   # Consistent with PET minimum threshold

            _lane_grid = getattr(self, "timestep_lane_positions_all", self.timestep_lane_positions)
            _pos_src   = getattr(self, "positions_all", self.positions)
            src_changes = getattr(self, "vehicle_lane_history_all", self.vehicle_lane_history)

            def nearest_idx(t):
                """Find the index of the timestamp closest to t within tolerance dt_tol."""
                i = bisect.bisect_left(times, t)
                cand = []
                if 0 <= i < len(times): cand.append((abs(times[i]-t), i))
                if i-1 >= 0:           cand.append((abs(times[i-1]-t), i-1))
                if i+1 < len(times):   cand.append((abs(times[i+1]-t), i+1))
                cand.sort(key=lambda z: z[0])
                return cand[0][1] if cand and cand[0][0] <= dt_tol else None

            self.accepted_gaps = {"hdv": [], "cav": []}

            for vt in ("hdv", "cav"):
                for vid, changes in (src_changes.get(vt, {}) or {}).items():
                    for (t_merge, old_lane, new_lane) in changes:
                        if not new_lane or new_lane.startswith(":J"):
                            continue

                        k = nearest_idx(t_merge)
                        if k is None:
                            continue
                        tN = times[k]
                        lanes = _lane_grid.get(tN, {})
                        vehs_on_new = lanes.get(new_lane, {})
                        if not vehs_on_new:
                            continue

                        # Position of the merging vehicle A (near tN)
                        a_pos = None
                        for (tt, pp) in _pos_src.get(vt, {}).get(vid, []):
                            if abs(tt - tN) <= dt_tol:
                                a_pos = pp
                                break
                        if a_pos is None:
                            continue

                        # Find the nearest leader and lag vehicle in the target lane
                        ahead, behind = [], []
                        for bid, bpos in vehs_on_new.items():
                            if bid == vid:
                                continue
                            if bpos >= a_pos:
                                ahead.append((bpos - a_pos, bid))
                            else:
                                behind.append((a_pos - bpos, bid))
                        ahead.sort()
                        behind.sort()
                        leader_id = ahead[0][1] if ahead else None
                        lag_id    = behind[0][1] if behind else None

                        def traj_of(bid):
                            """Retrieve the trajectory of vehicle bid."""
                            btype = (self.vehicle_info.get(bid, {}) or {}).get("type")
                            if not btype:
                                return None
                            return _pos_src.get(btype, {}).get(bid, [])

                        t_lead = None
                        t_lag  = None
                        if leader_id:
                            tr = traj_of(leader_id)
                            if tr:
                                t_lead = self._find_crossing_time(a_pos, tr)
                        if lag_id:
                            tr = traj_of(lag_id)
                            if tr:
                                t_lag = self._find_crossing_time(a_pos, tr)

                        gap = None
                        # Preferred case: both leader and lag exist
                        if (t_lead is not None) and (t_lag is not None) and (t_lead < t_merge <= t_lag + 1e-9):
                            gap = t_lag - t_lead
                        # Fallback: only lag vehicle exists
                        elif (t_lag is not None) and (t_lag >= t_merge):
                            gap = t_lag - t_merge

                        # Validate and record the accepted gap
                        if gap is not None and (gap >= eps_gap) and (gap <= max_gap) and (abs(gap) < 1e6):
                            self.accepted_gaps[vt].append(float(gap))
 
    def parse_stats(self, stats_file):
        """Parse stats.xml into `self.simulation_stats` (flat tag->attribs dict)."""
        print("Parsing stats file: ", stats_file)
        tree = ET.parse(stats_file)
        root = tree.getroot()
        
        stats = {}
        for element in root:
            tag = element.tag
            stats[tag] = {k: self._try_convert(v) for k, v in element.attrib.items()}
        
        self.simulation_stats = stats

    def fuel_rate_g_per_s(self, speed_mps: float, accel_mps2: float, veh_type: str = "hdv") -> float:
        """
        Physics‑based simple fuel model (g/s), numerically stable.

        Inputs
        ------
        speed_mps : float
            Vehicle speed [m/s]
        accel_mps2 : float
            Longitudinal acceleration [m/s^2]
        veh_type : str
            Vehicle type key (e.g., "hdv"/"cav"). Currently uses a unified parameter set.

        Notes
        -----
        - No negative fuel; includes an idle floor; handles low‑speed gracefully.
        - Traction power = inertia + rolling + aerodynamic; negative traction (braking) contributes no fuel
          beyond the idle floor.
        """
        #******* (math type: traction power -> fuel rate)

        # ---- Vehicle/physics params (tunable) ----
        rho = 1.225           # air density (kg/m^3)
        g   = 9.81
        # Unified parameters; if you need per‑type, set different CdA/mass/Crr by veh_type.
        mass = 1500.0         # kg (typical passenger car)
        CdA  = 0.65           # m^2 (drag coefficient * frontal area)
        Crr  = 0.012          # rolling resistance coefficient (asphalt)
        P_aux = 1000.0        # W (auxiliary loads: HVAC/electronics)
        # Idle fuel (g/s) and minimal speed threshold
        IDLE_GPS  = 0.20      # ~0.20 g/s ≈ 0.72 kg/h
        V_EPS     = 0.1       # [m/s] treat below as near idle
        # Engine BSFC (g/kWh), typical 240–280
        BSFC = 260.0

        v = max(0.0, float(speed_mps))
        a = float(accel_mps2)

        # Traction power (W): inertia + rolling + aero; negative traction doesn't increase fuel
        P_inertia = mass * a * v
        P_roll    = Crr * mass * g * v
        P_drag    = 0.5 * rho * CdA * v * v * v
        P_trac    = P_inertia + P_roll + P_drag

        # Positive mechanical demand + auxiliaries
        P_mech = max(0.0, P_trac) + P_aux

        # Very low speed / load -> revert to idle fuel
        if v < V_EPS and abs(a) < 0.1:
            return IDLE_GPS

        # Power -> fuel (g/s): (kW) * (g/kWh) / 3600
        gps = (P_mech / 1000.0) * (BSFC / 3600.0)

        # Enforce idle floor and non‑negativity
        gps = max(IDLE_GPS, gps)
        return float(gps)

    def _try_convert(self, value):
        """Try int/float conversion; otherwise return original string."""
        try:
            return float(value) if '.' in value else int(value)
        except ValueError:
            return value


    def _sanitize_speeds_for_hist(self):
        """Prepare speed arrays for histograms (filter NaNs and near‑zero speeds)."""
        vmin = getattr(self, "SPEED_MIN_HIST", 0.5)
        self.speeds_vis = {
            "hdv": [v for v in self.speeds.get("hdv", []) if np.isfinite(v) and v >= vmin],
            "cav": [v for v in self.speeds.get("cav", []) if np.isfinite(v) and v >= vmin],
        }

    def speed_dist(self):
        """Return mean/std of speeds for each vehicle type."""
        return {
            "hdv": [np.mean(self.speeds["hdv"]), np.std(self.speeds["hdv"])],
            "cav": [np.mean(self.speeds["cav"]), np.std(self.speeds["cav"])],
        }

    def acceleration_dist(self):
        """Return mean/std of accelerations for each vehicle type."""
        return {
            "hdv": [np.mean(self.accelerations["hdv"]), np.std(self.accelerations["hdv"])],
            "cav": [np.mean(self.accelerations["cav"]), np.std(self.accelerations["cav"])],
        }

    def total_delay(self, free_flow_speed):
        """
        Compute total delay for all vehicles (by type).

        Definition
        ----------
        Delay = (actual travel time) - (free‑flow time at `free_flow_speed`).

        Parameters
        ----------
        free_flow_speed : float
            Assumed maximum speed [m/s] for free‑flow time.

        Returns
        -------
        dict
            {
              "total_delay": {"hdv": ..., "cav": ...},
              "delay_per_mile": {"hdv": ..., "cav": ...},
              "vehicle_delays": {"hdv": {...}, "cav": {...}},
              "total_miles": {"hdv": ..., "cav": ...}
            }
        """
        total_delay = {"hdv": 0, "cav": 0}
        total_miles = {"hdv": 0, "cav": 0}
        vehicle_delays = {"hdv": {}, "cav": {}}
        
        for vehicle_type in ["hdv", "cav"]:
            for vehicle_id, travel_time in self.travel_times[vehicle_type].items():
                # Get final position (meters)
                if not self.positions[vehicle_type].get(vehicle_id):
                    continue
                
                final_pos = self.positions[vehicle_type][vehicle_id][-1][1]
                
                # Distance in miles
                distance_miles = final_pos * 0.000621371
                total_miles[vehicle_type] += distance_miles
                
                # Free‑flow time
                free_flow_time = final_pos / free_flow_speed
                
                # Delay for this vehicle (non‑negative)
                delay = travel_time - free_flow_time
                vehicle_delays[vehicle_type][vehicle_id] = max(0, delay)
                total_delay[vehicle_type] += max(0, delay)
        
        delay_per_mile = {
            "hdv": total_delay["hdv"] / total_miles["hdv"] if total_miles["hdv"] > 0 else 0,
            "cav": total_delay["cav"] / total_miles["cav"] if total_miles["cav"] > 0 else 0
        }
        
        return {
            "total_delay": total_delay,
            "delay_per_mile": delay_per_mile,
            "vehicle_delays": vehicle_delays,
            "total_miles": total_miles
        }

    def calculate_throughput(self):
        """
        Approx throughput per lane (veh/s) based on unique vehicles observed per lane
        over the whole simulation divided by simulation duration.
        """
        from collections import defaultdict
        seen = defaultdict(set)  # lane -> set(vehicle_id)
        for t, lanes in self.timestep_lane_occupancy.items():
            for lane_id, veh_ids in lanes.items():
                seen[lane_id].update(veh_ids)
        if self.simulation_duration <= 0:
            return {}
        return {lane: len(veh_ids) / self.simulation_duration for lane, veh_ids in seen.items()}
    
    def parse_detector_data(self, detector_file=None):
        """
        Parse detector XML (typical <interval> entries).
        Produces:
        - self.detector_data: list of dicts {lane, begin, end, occ, nVeh}
        - self.detector_by_lane: mapping lane -> list(interval dict), sorted by begin
        """
        if detector_file is None:
            detector_file = getattr(self, 'detector_file', None)
        self.detector_data = []
        self.detector_by_lane = defaultdict(list)

        if detector_file is None or not os.path.isfile(detector_file):
            print(f"[Warning] Detector file not found: {detector_file}")
            return

        tree = ET.parse(detector_file)
        root = tree.getroot()
        for interval in root.findall(".//interval"):
            try:

                det_id = interval.attrib.get("id")          # detector id (e.g., e1_...)
                lane_id = interval.attrib.get("lane") or det_id  # some dumps put lane id here
                begin = float(interval.attrib.get("begin", 0.0))
                end = float(interval.attrib.get("end", begin))
                occ = float(interval.attrib.get("occupancy", 0.0))
                # different detector outputs may use different attribute names:
                nVeh = int(float(interval.attrib.get("nVehEntered", interval.attrib.get("nVeh", 0))))
            except Exception:
                continue
            rec = {"id": det_id, "lane": lane_id, "begin": begin, "end": end, "occ": occ, "nVeh": nVeh}
            self.detector_data.append(rec)
            self.detector_by_lane[lane_id].append(rec)

        # sort by begin time
        for lane in self.detector_by_lane:
            self.detector_by_lane[lane].sort(key=lambda x: x["begin"])

    def parse_tls_data(self, tls_file=None):
        """
        Parse TLS/netstate dump or static tlLogic files.

        Outputs
        -------
        - self.tls_time_states: dict tls_id -> list of (time, state)
        - self.tls_intervals:   dict tls_id -> list of (t0, t1, state)  [accelerated]
        - self.tls_interval_index: dict tls_id -> (starts, ends, states) [accelerated]
        - self.tls_programs: dict tls_id -> list of {'state':..., 'duration':...} (if static)
        """
        if tls_file is None:
            tls_file = getattr(self, 'tls_file', None)

        self.tls_time_states = defaultdict(list)
        self.tls_programs = {}
        self.tls_intervals = {}
        self.tls_interval_index = {}

        if tls_file is None or not os.path.isfile(tls_file):
            print(f"[Warning] TLS file not found: {tls_file}")
            return

        tree = ET.parse(tls_file)
        root = tree.getroot()

        def tag_endswith(elem, name):
            return elem.tag.split('}')[-1] == name

        # --- Case A: timestep-based dump (tls.xml) ---
        timesteps = root.findall(".//timestep")
        if len(timesteps) > 0:
            for ts in timesteps:
                t = float(ts.attrib.get("time", 0.0))

                # Normal children (<tls> in netstate-dump; sometimes <tl> / <tlLogic>)
                for child in ts:
                    tag = child.tag.split('}')[-1]
                    if tag in ("tlLogic", "tl", "tls"):
                        tls_id = (child.attrib.get("id")
                                or child.attrib.get("tlsID")
                                or child.attrib.get("name"))
                        state = child.attrib.get("state")
                        if state is None:
                            ph = child.find(".//phase")
                            if ph is not None and "state" in ph.attrib:
                                state = ph.attrib["state"]
                        if tls_id and state is not None:
                            self.tls_time_states[tls_id].append((t, state))

            # Coalesce & index for each tls_id
            for tls_id, seq in self.tls_time_states.items():
                seq.sort(key=lambda x: x[0])
                intervals = _coalesce_tls_states(seq)
                self.tls_intervals[tls_id] = intervals
                self.tls_interval_index[tls_id] = _build_interval_index(intervals)

        else:
            # --- Case B: static tlLogic in net.xml ---
            for tl in root.findall(".//tlLogic"):
                tls_id = tl.attrib.get("id")
                phases = []
                for ph in tl.findall("phase"):
                    st = ph.attrib.get("state")
                    dur = float(ph.attrib.get("duration", 0.0))
                    if st is not None:
                        phases.append({"state": st, "duration": dur})
                if tls_id and phases:
                    self.tls_programs[tls_id] = phases

        # Sort time series (for backward compatibility)
        for tls_id in list(self.tls_time_states.keys()):
            self.tls_time_states[tls_id].sort(key=lambda x: x[0])

        total = sum(len(v) for v in self.tls_time_states.values())
        print(f"[Urban Signals][DEBUG] tls time-states captured: n_tls={len(self.tls_time_states)}, samples={total}")
        # Extra debug for intervals
        total_intervals = sum(len(v) for v in self.tls_intervals.values())
        if total_intervals:
            print(f"[Urban Signals][DEBUG] tls intervals built: n_tls={len(self.tls_intervals)}, intervals={total_intervals}")

    def _has_detector_data(self):
        """
        Quick check for detector data availability.
        Returns True only if `parse_detector_data` succeeded and valid interval records exist.
        """
        return hasattr(self, "detector_data") and isinstance(self.detector_data, list) and len(self.detector_data) > 0
    
    def _is_city_signalized_corridor(self):
        """
        Determine whether the dataset appears to be an urban signalized corridor/intersection case.

        Criteria:
        - Detector intervals (loop/magnet/virtual detector output) are available.
        - TLS has valid green windows (static or time-series).
        """
        if not self._has_detector_data():
            return False
        if not self._has_tls_green_windows():
            return False
        return True

    def _has_tls_green_windows(self):
        """
        Quick check for TLS green windows (either static program or time series).
        Uses `_get_green_windows()` to verify at least one green window is present.
        """
        greens, _, _, _ = self._get_green_windows(tls_id=None)
        return len(greens) > 0

    def _get_green_windows(self, tls_id=None, movement_col=None):
        """
        Build green-light windows.

        If movement_col is None:
            - Static program: any phase containing 'g'/'G' -> a green window of its duration.
            - Time-series: any consecutive interval with 'g'/'G' -> a green window.
        If movement_col is not None:
            - Only treat a window as green if that column is green.

        Returns
        -------
        green_windows : list[(start, end)]
        is_static : bool
            True if static (cycle-based), False if time-series.
        cycle_len : float or None
            Cycle length if static, else None.
        chosen_tls : str or None
            TLS ID used when multiple programs are present.
        """
        green_windows = []

        # --- Static program ---
        if self.tls_programs:
            chosen = tls_id or next(iter(self.tls_programs.keys()))
            phases = self.tls_programs.get(chosen, [])
            t = 0.0
            for ph in phases:
                dur = float(ph["duration"])
                st = (ph["state"] or "")
                is_green_phase = False
                if movement_col is None:
                    is_green_phase = any(_is_green_char(c) for c in st)
                else:
                    if movement_col < len(st) and _is_green_char(st[movement_col]):
                        is_green_phase = True
                if is_green_phase:
                    green_windows.append((t, t + dur))
                t += dur
            return green_windows, True, t, chosen

        # --- Time-series ---
        if self.tls_intervals:
            chosen = tls_id or next(iter(self.tls_intervals.keys()))
            intervals = self.tls_intervals.get(chosen, [])
            cur_start = None

            def interval_has_green(state):
                if movement_col is None:
                    return any(_is_green_char(c) for c in state)
                if movement_col < len(state):
                    return _is_green_char(state[movement_col])
                return False

            for a, b, s in intervals:
                if interval_has_green(s):
                    if cur_start is None:
                        cur_start = a
                else:
                    if cur_start is not None:
                        green_windows.append((cur_start, a))
                        cur_start = None
            if cur_start is not None and intervals:
                green_windows.append((cur_start, intervals[-1][1]))
            return green_windows, False, None, chosen

        # Fallback to raw samples if no intervals
        if self.tls_time_states:
            chosen = tls_id or next(iter(self.tls_time_states.keys()))
            series = self.tls_time_states.get(chosen, [])
            for i in range(len(series) - 1):
                t0, st0 = series[i]
                t1, _ = series[i + 1]
                if movement_col is None:
                    is_green = any(_is_green_char(c) for c in (st0 or ""))
                else:
                    is_green = (movement_col < len(st0)) and _is_green_char(st0[movement_col])
                if is_green:
                    green_windows.append((t0, t1))
            return green_windows, False, None, chosen

        return [], False, None, None

    def _debug_city_signal_requirements(self):
        """Print why the dataset is treated as non‑urban‑signal and summarize data scale."""
        has_det = hasattr(self, "detector_data") and isinstance(self.detector_data, list) and len(self.detector_data) > 0
        det_count = len(self.detector_data) if has_det else 0

        greens, is_static, cycle_len, chosen_tls = self._get_green_windows(tls_id=None)
        has_tls_green = len(greens) > 0

        print("[Urban Signals][DEBUG] detector_data: ", "OK" if has_det else "MISSING/EMPTY",
            f"(intervals={det_count})")
        print("[Urban Signals][DEBUG] tls green windows: ", "OK" if has_tls_green else "MISSING",
            f"(tls_id={chosen_tls}, n_greens={len(greens)}, is_static={is_static}, cycle_len={cycle_len})")

        return has_det, has_tls_green
    
    def tls_state(self, tls_id, t):
        """
        Get the signal state string at time `t` for `tls_id`.
        Prefer accelerated interval index; fall back to linear search on raw samples.
        """
        idx = self.tls_interval_index.get(tls_id)
        if idx is not None:
            return _state_at_time(idx, t)
        # fallback (linear)
        seq = self.tls_time_states.get(tls_id, [])
        last_state = None
        for tt, st in seq:
            if tt - EPS <= t:
                last_state = st
            else:
                break
        return last_state

    def is_green(self, tls_id, col_idx, t):
        """Return True if movement column `col_idx` is green at time `t` for `tls_id`."""
        s = self.tls_state(tls_id, t)
        if s is None or col_idx >= len(s):
            return False
        return _is_green_char(s[col_idx])

    def _run_city_signal_metrics_if_applicable(self, allow_partial=True):
        """
        Only compute the following metrics when the dataset qualifies as an “urban signal” case:
        - PAoG / GOR / Approach Delay (intersection approach metric) / Spillback / TTS
          (+ CAV/HDV splits with backfilled dashboard keys)

        If not qualified, print the reason. With `allow_partial=True`, run partial metrics that
        require only detector data (e.g., spillback), even without TLS.
        """
        has_det, has_tls_green = self._debug_city_signal_requirements()

        if has_det and has_tls_green:
            print("[Urban Signals] Detector + TLS detected. Computing PAOG / GOR / ApproachDelay / Spillback / TTS ...")

            # --- PAOG ---
            try:
                self.compute_paog()
            except Exception as e:
                print(f"[Urban Signals][WARN] compute_paog failed: {e}")

            # --- GOR (overall) ---
            try:
                self.compute_gor()
            except Exception as e:
                print(f"[Urban Signals][WARN] compute_gor failed: {e}")

            # --- Approach Delay (intersection approach metric) ---
            try:
                self.compute_approach_delay(free_flow_speed_hdv=13.0, free_flow_speed_cav=15.0)
            except Exception as e:
                print(f"[Urban Signals][WARN] compute_approach_delay failed: {e}")

            # --- Spillback ---
            try:
                self.compute_queue_spillback()
            except Exception as e:
                print(f"[Urban Signals][WARN] compute_queue_spillback failed: {e}")

            # --- TTS (overall) ---
            try:
                self.compute_time_to_service()
            except Exception as e:
                print(f"[Urban Signals][WARN] compute_time_to_service failed: {e}")

            # ========== NEW: by‑type versions and dashboard backfilling ==========
            # GOR by type
            try:
                if hasattr(self, "compute_gor"):
                    self.compute_gor(tls_id=None, debug=False)

                    # Backfill by-type results into common keys for dashboards
                    if not hasattr(self, "gor_values") or not isinstance(self.gor_values, dict):
                        self.gor_values = {}
                    by_type = {}
                    if hasattr(self, "gor_type") and isinstance(self.gor_type, dict):
                        by_type = {
                            "cav": self.gor_type.get("cav"),
                            "hdv": self.gor_type.get("hdv"),
                            "all": self.gor_type.get("all"),
                        }
                    self.gor_values["by_type"] = by_type
                    if by_type:
                        self.gor_values["cav"] = by_type.get("cav")
                        self.gor_values["hdv"] = by_type.get("hdv")
                else:
                    print("[Urban Signals][INFO] compute_gor_by_type_fcd not found; skipping by-type GOR.")
            except Exception as e:
                print(f"[Urban Signals][WARN] compute_gor_by_type_fcd failed: {e}")

            # TTS by type
            try:
                if hasattr(self, "compute_time_to_service"):
                    self.compute_time_to_service(tls_id=None, debug=False)

                    if not hasattr(self, "time_to_service_values") or not isinstance(self.time_to_service_values, dict):
                        self.time_to_service_values = {}
                    if hasattr(self, "time_to_service_split") and isinstance(self.time_to_service_split, dict):
                        self.time_to_service_values["by_type"] = {
                            "cav": self.time_to_service_split.get("cav"),
                            "hdv": self.time_to_service_split.get("hdv"),
                            "all": self.time_to_service_split.get("all"),
                        }
                else:
                    print("[Urban Signals][INFO] compute_time_to_service not found; skipping by-type TTS.")
            except Exception as e:
                print(f"[Urban Signals][WARN] compute_time_to_service failed: {e}")

            try:
                print("[PIPELINE] GOR(all) =", self.gor_values.get("all") if hasattr(self, "gor_values") else None)
                print("[PIPELINE] GOR(by_type) =", self.gor_values.get("by_type") if hasattr(self, "gor_values") else None)
                print("[PIPELINE] TTS(all) =", getattr(self, "time_to_service_values", None))
                print("[PIPELINE] TTS(by_type) =", (self.time_to_service_values.get("by_type")
                                                if isinstance(getattr(self, "time_to_service_values", None), dict) else None))
            except Exception as e:
                print(f"[PIPELINE][WARN] print summary failed: {e}")

        else:
            if not has_det and not has_tls_green:
                print("[Urban Signals] Detector intervals missing/empty AND no TLS green windows. Skipping all city-signal metrics.")
            elif not has_det:
                print("[Urban Signals] Detector intervals missing/empty. Skipping city-signal metrics.")
            else:  # not has_tls_green
                print("[Urban Signals] No TLS green windows detected. Skipping city-signal metrics.")

            if allow_partial and has_det and not has_tls_green:
                print("[Urban Signals] Running partial metrics that only need Detector: Spillback.")
                try:
                    self.compute_queue_spillback()
                except Exception as e:
                    print(f"[Urban Signals][WARN] partial compute_queue_spillback failed: {e}")
   
    # ------------------------------
    # 1. PAOG (Percent Arrivals On Green)
    # =========================
    def parse_add_detectors_to_map(self, add_xml_path):
        """
        Read SUMO additional (add.xml) to build mapping: detector_id -> lane_id.
        Returns: dict like {"W060_straight1": "277219207#0_2", ...}
        """
        import xml.etree.ElementTree as ET
        det2lane = {}
        if not add_xml_path or not os.path.isfile(add_xml_path):
            print(f"[Mapping] WARN: add.xml not found: {add_xml_path}")
            return det2lane

        tree = ET.parse(add_xml_path)
        root = tree.getroot()
        # e1Detector
        for e in root.findall(".//e1Detector"):
            did = e.attrib.get("id")
            lane = e.attrib.get("lane")
            if did and lane:
                det2lane[did] = lane
        # Also support generic <detector> tags if present
        for e in root.findall(".//detector"):
            did = e.attrib.get("id")
            lane = e.attrib.get("lane")
            if did and lane and did not in det2lane:
                det2lane[did] = lane

        print(f"[Mapping] det2lane loaded: {len(det2lane)} pairs")
        return det2lane
    
    def load_lane_tls_mapping_json(self, json_path):
        """
        Load lane->TLS mapping JSON where keys are SUMO lane IDs.
        Value schema accepts either {"tls_id": "...", "groups": [...]} or {"tls_id": "...", "phase_pos": [...]}.

        Returns
        -------
        dict
            lane_id -> {"tls_id": str, "groups": [int, ...]}
        """
        import json
        lane_map = {}
        if not json_path or not os.path.isfile(json_path):
            print(f"[Mapping] WARN: lane mapping JSON not found: {json_path}")
            return lane_map
        with open(json_path, "r") as f:
            raw = json.load(f)
        for lane_id, val in raw.items():
            if not isinstance(val, dict):
                continue
            tls_id = val.get("tls_id")
            groups = val.get("groups")
            if groups is None:
                groups = val.get("phase_pos")  # backward‑compat field
            lane_map[lane_id] = {"tls_id": tls_id, "groups": groups}
        print(f"[Mapping] lane->TLS loaded: {len(lane_map)} lanes")
        return lane_map

    def debug_list_fcd_lane_keys(self, sample_times=5, max_keys=50):
        """
        Print a sample of lane keys observed in self.timestep_lane_positions.
        Helps verify the exact lane‑id strings that the FCD parser produced.
        """
        if not hasattr(self, "timestep_lane_positions"):
            print("[FCD/DEBUG] timestep_lane_positions missing.")
            return
        times = list(getattr(self, "timesteps", []))
        if not times:
            print("[FCD/DEBUG] timesteps empty.")
            return

        seen = set()
        for t in times[:sample_times]:
            lane_dict = self.timestep_lane_positions.get(t, {})
            for lane_id in lane_dict.keys():
                seen.add(lane_id)
                if len(seen) >= max_keys:
                    break
            if len(seen) >= max_keys:
                break

        # for i, k in enumerate(sorted(seen)):
        #     if i >= max_keys:
        #         print("  ...")
        #         break
        #     print("  ", k)
    
    def debug_check_mapped_lanes_in_fcd(self, detector_mapping, max_print_per_lane=3):
        """
        For each mapped detector, check if its lane_id appears in FCD at all,
        and print a few (time, vehicle_id) samples if present.
        """
        if detector_mapping is None and hasattr(self, "detector_mapping"):
            detector_mapping = self.detector_mapping
        if not isinstance(detector_mapping, dict) or not detector_mapping:
            print("[MAP/DEBUG] detector_mapping missing or empty.")
            return
        if not hasattr(self, "timestep_lane_positions"):
            print("[MAP/DEBUG] FCD grid missing.")
            return

        print(f"[MAP/DEBUG] Checking {len(detector_mapping)} mapped detectors against FCD lanes...")
        found = 0
        missing = 0
        times = list(getattr(self, "timesteps", []))
        for det_id, mp in detector_mapping.items():
            lane_id = mp.get("lane_id") or mp.get("sumo_lane") or mp.get("lane")
            if not lane_id:
                print(f"  - det={det_id}: mapping has no lane_id")
                missing += 1
                continue

            # Scan early portion quickly (first ~20s)
            printed = 0
            seen_any = False
            for t in times[:200]:
                lane_dict = self.timestep_lane_positions.get(t, {})
                vehs = lane_dict.get(lane_id, {})
                if vehs:
                    if not seen_any:
                        # print some examples
                        seen_any = True
                    for v in list(vehs.keys())[:max_print_per_lane]:
                        print(f"     t={t:.2f}  veh={v}")
                    printed += 1
                    if printed >= 1:
                        break

            if seen_any:
                found += 1
            else:
                missing += 1

        print(f"[MAP/DEBUG] Summary: found={found}, missing={missing} (quick sample).")

    def debug_probe_lane_window(self, lane_id, t0=0.0, t1=60.0, max_hits=10):
        """
        Probe a specific lane_id within [t0,t1] and print (time, vehicle ids) up to max_hits.
        Use this when PAoG reports 'no vehicles on lane=...' but you expect otherwise.
        """
        if not hasattr(self, "timestep_lane_positions"):
            return
        times = list(getattr(self, "timesteps", []))
        if not times:
            return
        import bisect
        i0 = bisect.bisect_left(times, t0 - 1e-9)
        i1 = bisect.bisect_right(times, t1 + 1e-9)
        hits = 0
        for ti in range(i0, i1):
            t = times[ti]
            lane_dict = self.timestep_lane_positions.get(t, {})
            vehs = lane_dict.get(lane_id, {})
            if vehs:
                # print a small sample
                for v in list(vehs.keys())[:5]:
                    print(f"[FCD/PROBE] t={t:.2f} lane={lane_id} veh={v}")
                hits += 1
                if hits >= max_hits:
                    break
        if hits == 0:
            print(f"[FCD/PROBE] no vehicles found on lane={lane_id} within [{t0},{t1}].")

    def _compose_detector_mapping(self, det2lane, lane_map):
        """
        Build detector->mapping using:
          det2lane: detector_id -> lane_id (from add.xml)
          lane_map: lane_id -> {tls_id, groups}
        Returns:
          detector_id -> {"sumo_lane": lane_id, "tls_id": ..., "groups": [...]}
        """
        out = {}
        n_ok, n_miss = 0, 0
        for det_id, lane_id in (det2lane or {}).items():
            rec = {"sumo_lane": lane_id, "tls_id": None, "groups": None}
            if lane_map and lane_id in lane_map:
                rec["tls_id"] = lane_map[lane_id].get("tls_id")
                rec["groups"] = lane_map[lane_id].get("groups")
                n_ok += 1
            else:
                n_miss += 1
            out[det_id] = rec
        print(f"[Mapping] composed: {len(out)} dets (tls-linked={n_ok}, no-tls={n_miss})")
        return out

 
    def compute_paog(
        self,
        pre_stop_window=8.0,
        stop_speed_thresh=0.5,
        detector_mapping=None,
        dt_tolerance=1e-6,
        count_yellow_as_green=False,
        debug=True,
        debug_samples_per_lane=2,
        allow_edge_fallback=True,
    ):
        """
        Compute PAoG (Percent of Arrivals on Green) per detector interval using FCD + TLS + mapping.

        Main idea
        ---------
        1) For each detector interval, find each vehicle's first‑appearance time on the mapped SUMO lane_id.
        2) If TLS series + mapping exist, classify arrival as on‑green if required signal groups are green
           at t_arr (optionally count 'y' as green).
        3) Otherwise fallback to a brief speed/stop heuristic.
        4) If the exact FCD lane key is not found, optionally fall back to an *edge‑level* merge
           (e.g., use all FCD keys whose "edge part" equals the mapped lane's edge).

        Returns
        -------
        dict: detector_id -> list[dict]
            Each dict includes: begin, end, paog, n_total, n_cav, n_hdv, paog_cav, paog_hdv, n_detected
        """
        import bisect, re

        # ---- resolve mapping param ----
        if detector_mapping is None and hasattr(self, "detector_mapping"):
            detector_mapping = self.detector_mapping
        have_mapping = isinstance(detector_mapping, dict) and len(detector_mapping) > 0

        # ---- prechecks ----
        if not hasattr(self, "timesteps") or not hasattr(self, "timestep_lane_positions"):
            print("[PAoG] ERROR: FCD timesteps / lane grid missing.")
            self.paog_by_detector = {}
            return {}
        if not getattr(self, "detector_data", None):
            print("[PAoG] ERROR: detector_data missing/empty.")
            self.paog_by_detector = {}
            return {}

        times = list(self.timesteps)
        if not times:
            print("[PAoG] ERROR: empty FCD time axis.")
            self.paog_by_detector = {}
            return {}
        tmin, tmax = times[0], times[-1]
        time_to_idx = {t: i for i, t in enumerate(times)}

        have_tls_series = bool(getattr(self, "tls_time_states", None))
        can_use_tls = have_tls_series and have_mapping

        # ---- tiny helpers ----
        def is_cav(veh_id: str) -> bool:
            return "_cav" in (veh_id or "").lower()

        _tls_axes = {}
        def tls_state_at(tls_id, series, t_query):
            # series: [(t, state_str), ...]
            if tls_id not in _tls_axes:
                _tls_axes[tls_id] = [tt for tt, _ in series]
            axis = _tls_axes[tls_id]
            j = bisect.bisect_right(axis, t_query) - 1
            return None if j < 0 else series[j][1]

        green_set = {'g', 'G'}
        if count_yellow_as_green:
            green_set |= {'y', 'Y'}

        def on_green(state_str, groups):
            if state_str is None:
                return False
            s = state_str
            if not groups:
                return any(ch in green_set for ch in s)
            for gi in groups:
                if gi < 0 or gi >= len(s): return False
                if s[gi] not in green_set: return False
            return True

        # Extract the “edge part” by stripping the trailing "_<laneIndex>"
        # e.g., "277219207#0_2" -> "277219207#0"
        lane_idx_pat = re.compile(r"(.+)_\d+$")
        def edge_part(lane_id: str) -> str:
            m = lane_idx_pat.match(lane_id or "")
            return m.group(1) if m else (lane_id or "")

        # ---- DEBUG: show how FCD lanes are keyed ----
        if debug:
            # collect a sample of FCD keys from first few timesteps
            fcd_keys_sample = set()
            for t in times[:min(len(times), 30)]:  # up to the first ~3s if dt=0.1
                ln = self.timestep_lane_positions.get(t, {})
                fcd_keys_sample.update(ln.keys())
                if len(fcd_keys_sample) >= 50:
                    break
            sample_show = sorted(list(fcd_keys_sample))[:20]
            # print(f"[PAoG/DEBUG] FCD keys sample (first timesteps): {sample_show}")
            # quick check: do we see lane-indexed keys (ending with _d)?
            has_lane_index = any(lane_idx_pat.match(k) for k in fcd_keys_sample)
            print(f"[PAoG/DEBUG] FCD lane style: {'lane-indexed' if has_lane_index else 'edge-level or mixed'}")

            # mapping status
            print(f"[PAoG/DEBUG] mapping(detector->lane/tls) size={len(detector_mapping) if have_mapping else 0}")
            print(f"[PAoG/DEBUG] FCD window=[{tmin}..{tmax}] dt≈{(times[1]-times[0]) if len(times)>1 else 'NA'}")
            tls_ids = list(self.tls_time_states.keys()) if have_tls_series else []
            print(f"[PAoG/DEBUG] TLS time-series: {have_tls_series} (n={len(tls_ids)})")

        # ---- iterate detector intervals with time clamping ----
        results = {}
        lane_dbg_count = {}
        skipped_no_overlap = 0
        clamped_intervals = 0

        for d in self.detector_data:
            det_id = d.get("id") or d.get("detector_id") or d.get("lane")
            if not det_id:
                if debug: print("[PAoG/DEBUG] skip(detector without id):", d)
                continue

            begin = float(d["begin"]); end = float(d["end"])
            if end < tmin or begin > tmax:
                skipped_no_overlap += 1
                continue
            c_begin, c_end = max(begin, tmin), min(end, tmax)
            if (c_begin != begin) or (c_end != end):
                clamped_intervals += 1

            # mapping lookup
            mapped = detector_mapping.get(det_id, {}) if have_mapping else {}
            lane_id = mapped.get("sumo_lane") or mapped.get("lane_id") or d.get("lane_id") or d.get("sumo_lane")
            tls_id  = mapped.get("tls_id")
            groups  = mapped.get("groups") or mapped.get("phase_pos")

            if debug:
                cnt = lane_dbg_count.get(det_id, 0)
                if cnt < 1:
                    # print a one‑time mapping line if needed
                    lane_dbg_count[det_id] = cnt + 1

            # collect first‑appearance times on this lane (with lane/edge fallback)
            seen_first = {}
            i0 = bisect.bisect_left(times, c_begin - 1e-9)
            i1 = bisect.bisect_right(times, c_end   + 1e-9)

            epart = edge_part(lane_id or "")
            used_edge_fallback_once = False

            for ti in range(i0, i1):
                t = times[ti]
                lane_dict = self.timestep_lane_positions.get(t, {})
                veh_pos_map = lane_dict.get(lane_id, None)

                # fallback: merge all FCD keys whose edge_part equals lane_id's edge
                if veh_pos_map is None and allow_edge_fallback and epart:
                    merged = {}
                    for k, vmap in lane_dict.items():
                        if edge_part(k) == epart:
                            for vid, pos in vmap.items():
                                merged[vid] = pos
                    if merged:
                        veh_pos_map = merged
                        used_edge_fallback_once = True

                if not veh_pos_map:
                    continue

                for vid in veh_pos_map.keys():
                    if vid not in seen_first:
                        seen_first[vid] = t

            n_total = len(seen_first)
            if n_total == 0:
                results.setdefault(det_id, []).append({
                    "begin": c_begin, "end": c_end, "paog": None,
                    "n_total": 0, "n_cav": 0, "n_hdv": 0,
                    "paog_cav": None, "paog_hdv": None,
                    "n_detected": d.get("nVeh", None),
                })
                continue

            # Counters
            cav_total = hdv_total = 0
            cav_green = hdv_green = 0
            series = self.tls_time_states.get(tls_id) if (can_use_tls and tls_id) else None

            for vid, t_arr in seen_first.items():
                if series is not None:
                    st = tls_state_at(tls_id, series, t_arr)
                    ok = on_green(st, groups)
                else:
                    # fallback speed/stop heuristic
                    idx = time_to_idx.get(t_arr, None)
                    if idx is None or idx == 0:
                        speed_est = 1.0
                    else:
                        pos_t = self.timestep_lane_positions.get(t_arr, {}).get(lane_id, {}).get(vid)
                        t_prev = times[idx - 1]
                        pos_prev = self.timestep_lane_positions.get(t_prev, {}).get(lane_id, {}).get(vid)
                        if pos_t is None or pos_prev is None:
                            speed_est = 1.0
                        else:
                            dtp = max(1e-6, (t_arr - t_prev))
                            speed_est = (pos_t - pos_prev) / dtp

                    stopped_recently = False
                    t_start = t_arr - pre_stop_window
                    j0 = bisect.bisect_left(times, t_start - 1e-9)
                    j1 = idx if idx is not None else j0
                    for j in range(j0, j1 + 1):
                        tt = times[j]
                        pos_j = self.timestep_lane_positions.get(tt, {}).get(lane_id, {}).get(vid)
                        if j > 0:
                            tt_prev = times[j - 1]
                            pos_j_prev = self.timestep_lane_positions.get(tt_prev, {}).get(lane_id, {}).get(vid)
                        else:
                            pos_j_prev = None
                        if pos_j is not None and pos_j_prev is not None:
                            dtj = tt - times[j - 1]
                            vj = (pos_j - pos_j_prev) / max(1e-6, dtj)
                            if vj <= stop_speed_thresh:
                                stopped_recently = True
                                break
                    ok = (not stopped_recently) and (speed_est is None or speed_est > stop_speed_thresh)

                if is_cav(vid):
                    cav_total += 1
                    if ok: cav_green += 1
                else:
                    hdv_total += 1
                    if ok: hdv_green += 1

            paog_all = (cav_green + hdv_green) / n_total if n_total > 0 else None
            paog_cav = (cav_green / cav_total) if cav_total > 0 else None
            paog_hdv = (hdv_green / hdv_total) if hdv_total > 0 else None

            results.setdefault(det_id, []).append({
                "begin": c_begin, "end": c_end,
                "paog": paog_all,
                "n_total": n_total,
                "n_cav": cav_total, "n_hdv": hdv_total,
                "paog_cav": paog_cav, "paog_hdv": paog_hdv,
                "n_detected": d.get("nVeh", None),
            })

        if debug:
            print(f"[PAoG/DEBUG] Summary: skipped_no_overlap={skipped_no_overlap}, "
                f"clamped_intervals={clamped_intervals}, detectors={len(results)}")

        self.paog_by_detector = results
        return results

    # 2. GOR (Green Occupancy Ratio)
    # =========================

    def _resolve_lane_key(self, d):
        """
        Safely parse the “lane” field in a detector interval record into a SUMO lane key.

        Rules
        -----
        - If d["lane"] is already a SUMO lane (e.g., '435381830#0_2'), return it unchanged.
        - If it is a detector ID (e.g., 'W040_left') and appears in self.det2lane, return the mapped SUMO lane.
        - Otherwise, return the input as-is.
        """
        ln = d.get("lane") or d.get("id") or d.get("detector_id")
        if hasattr(self, "det2lane") and isinstance(self.det2lane, dict):
            return self.det2lane.get(ln, ln)
        return ln

    def compute_gor(self, tls_id=None, detector_filter=None, count_yellow_as_green=False, clip=True, debug=False):
        """
        Compute type‑split Green Occupancy using FCD presence during green windows.

        For each detector interval overlapping a green window, sample timesteps and check
        if any CAV/HDV are present on that lane; accumulate presence time and divide by
        total green time.

        Produces
        --------
        self.gor_type = {
            "all_green_time": total_green_time_considered,
            "cav": gor_cav,   # in [0,1] or None
            "hdv": gor_hdv,   # in [0,1] or None
        }

        Note
        ----
        This does NOT use detector 'occ' directly (which is not type‑split);
        it uses FCD lane occupancy instead.
        """
        # Preconditions
        if not getattr(self, "detector_data", None):
            print("[GOR-byType] Detector data not available.")
            self.gor_type = {"all_green_time": 0.0, "cav": None, "hdv": None}
            return

        if not (hasattr(self, "timesteps") and hasattr(self, "timestep_lane_positions")):
            print("[GOR-byType] FCD grids not available.")
            self.gor_type = {"all_green_time": 0.0, "cav": None, "hdv": None}
            return

        green_windows, _, _, chosen_tls = self._get_green_windows(tls_id)
        if not green_windows:
            print("[GOR-byType] No TLS green windows available.")
            self.gor_type = {"all_green_time": 0.0, "cav": None, "hdv": None}
            return

        # Optional lane filtering (use the 'lane' carried in detector_data)
        allowed = set(detector_filter) if detector_filter else None

        # Helper: CAV / HDV classifier from vehicle id
        def is_cav(veh_id: str) -> bool:
            return "_cav" in (veh_id or "").lower()

        times = list(self.timesteps)
        if len(times) < 2:
            print("[GOR-byType] FCD timesteps insufficient.")
            self.gor_type = {"all_green_time": 0.0, "cav": None, "hdv": None}
            return

        import bisect
        dt = times[1] - times[0]

        cav_time = 0.0
        hdv_time = 0.0
        green_time_total = 0.0

        for d in self.detector_data:
            lane = self._resolve_lane_key(d)
            if allowed is not None and lane not in allowed:
                continue

            int_s = float(d["begin"]); int_e = float(d["end"])
            if int_e <= int_s:
                continue

            # Intersect detector interval with each green window
            for gs, ge in green_windows:
                s = max(int_s, gs); e = min(int_e, ge)
                if s >= e:
                    continue

                # Integrate on discrete FCD timesteps
                i0 = bisect.bisect_left(times, s - 1e-9)
                i1 = bisect.bisect_left(times, e - 1e-9)  # half‑open on the right
                if i1 <= i0:
                    continue

                for i in range(i0, i1):
                    t = times[i]
                    lane_dict = self.timestep_lane_positions.get(t, {})
                    vehs = lane_dict.get(lane, {})
                    if not vehs:
                        # no vehicles this sample; contribute only to denominator
                        green_time_total += dt
                        continue
                    has_cav = any(is_cav(vid) for vid in vehs.keys())
                    has_hdv = any((not is_cav(vid)) for vid in vehs.keys())
                    if has_cav: cav_time += dt
                    if has_hdv: hdv_time += dt
                    green_time_total += dt

        gor_cav = (cav_time / green_time_total) if green_time_total > 0 else None
        gor_hdv = (hdv_time / green_time_total) if green_time_total > 0 else None

        if clip:
            if gor_cav is not None:
                gor_cav = max(0.0, min(1.0, gor_cav))
            if gor_hdv is not None:
                gor_hdv = max(0.0, min(1.0, gor_hdv))

        self.gor_type = {
            "all_green_time": green_time_total,
            "cav": gor_cav,
            "hdv": gor_hdv
        }
        if debug:
            print(f"[GOR-byType] tls_id={chosen_tls} green_time={green_time_total:.1f}s cav={gor_cav} hdv={gor_hdv}")
    
    # ------------------------------
    # 3. Approach Delay
    # ------------------------------
    def compute_approach_delay(self, free_flow_speed_hdv=13.0, free_flow_speed_cav=15.0):
        """
        Compute average approach delay per vehicle.
        Delay = actual elapsed time − free‑flow time (based on full‑trajectory approximation).
        """
        self.approach_delay_values = {"hdv": 0.0, "cav": 0.0}
        for v_type in ["hdv", "cav"]:
            delays = []
            for veh_id, traj in self.positions[v_type].items():
                if not traj: continue
                (t0, p0) = traj[0]; (t1, p1) = traj[-1]
                travel_time = max(0.0, t1 - t0)
                dist = max(0.0, p1 - p0)
                v_ff = free_flow_speed_hdv if v_type == "hdv" else free_flow_speed_cav
                ff_time = dist / max(v_ff, 0.1)
                delays.append(max(0.0, travel_time - ff_time))
            self.approach_delay_values[v_type] = float(np.mean(delays)) if delays else 0.0

        print(f"Average approach delay - HDV: {self.approach_delay_values['hdv']:.2f}s, "
              f"CAV: {self.approach_delay_values['cav']:.2f}s")

    # =========================
    # 4. Queue Spillback Occurrence
    # =========================

    def compute_queue_spillback(self, occ_threshold=0.8):
        """
        Compute queue spillback metrics.
        A spillback is recorded when detector occupancy exceeds a threshold
        for a continuous period with zero entries, indicating vehicles are backed up.
        """

        self.spillback_events = []   # full event log
        current_event = {}           # ongoing event (if any)

        for d in self.detector_data:
            lane = d["lane"]
            occ = d["occ"] / 100.0   # convert % to fraction
            nVeh = d["nVeh"]
            begin = d["begin"]
            end = d["end"]

            if occ >= occ_threshold and nVeh == 0:
                # spillback condition triggered
                if not current_event:
                    # start a new event
                    current_event = {
                        "lane": lane,
                        "start": begin,
                        "end": end,
                        "max_occ": occ,
                    }
                else:
                    # extend ongoing event
                    current_event["end"] = end
                    current_event["max_occ"] = max(current_event["max_occ"], occ)
            else:
                # spillback ended
                if current_event:
                    duration = current_event["end"] - current_event["start"]
                    current_event["duration"] = duration
                    self.spillback_events.append(current_event)
                    current_event = {}

        # close last open event if exists
        if current_event:
            duration = current_event["end"] - current_event["start"]
            current_event["duration"] = duration
            self.spillback_events.append(current_event)

        # summary stats
        if self.spillback_events:
            durations = [e["duration"] for e in self.spillback_events]
            self.spillback_summary = {
                "count": len(self.spillback_events),
                "max_duration": max(durations),
                "avg_duration": sum(durations) / len(durations),
            }
        else:
            self.spillback_summary = {
                "count": 0,
                "max_duration": 0.0,
                "avg_duration": 0.0,
            }

        # print log for debugging
        print("[Spillback Events]")
        for e in self.spillback_events:
            print(f"Lane={e['lane']} | Start={e['start']:.1f}s | End={e['end']:.1f}s | "
                f"Duration={e['duration']:.1f}s | MaxOcc={e['max_occ']:.2f}")

    # =========================
    # 5. Time to Service 
    # =========================
    def compute_time_to_service(self, tls_id=None, detector_filter=None, count_yellow_as_green=False, debug=False):
        """
        Compute Time To Service (TTS) split by CAV/HDV using FCD-based arrivals:
        - For each detector interval and mapped lane, find each vehicle's first‑seen
          time within that interval (arrival).
        - TTS = time from arrival to the next green start (or 0 if arriving during green).
        - Aggregate TTS per vehicle type.

        Sets
        ----
        self.time_to_service_split = {
            "cav": {"avg": ..., "p50": ..., "p95": ..., "n": N_cav},
            "hdv": {"avg": ..., "p50": ..., "p95": ..., "n": N_hdv},
            "all": {"avg": ..., "p50": ..., "p95": ..., "n": N_all},
            "tls_id": <chosen tls>
        }
        """
        # Checks
        if not getattr(self, "detector_data", None):
            print("[TTS-byType] Missing detector data.")
            self.time_to_service_split = {}
            return
        if not (hasattr(self, "timesteps") and hasattr(self, "timestep_lane_positions")):
            print("[TTS-byType] Missing FCD timesteps/lanes.")
            self.time_to_service_split = {}
            return

        green_windows, is_static, cycle_len, chosen_tls = self._get_green_windows(tls_id)
        if not green_windows:
            print("[TTS-byType] Missing TLS green windows.")
            self.time_to_service_split = {}
            return

        # Precompute green starts for fast lookup
        green_starts = [gs for gs, ge in green_windows]
        times = list(self.timesteps)
        time_to_idx = {t: i for i, t in enumerate(times)}

        # Helpers
        def is_cav(veh_id: str) -> bool:
            return "_cav" in (veh_id or "").lower()

        def next_green_start(t_arr: float) -> float:
            """Return the next green start time >= t_arr; if none, return None."""
            idx = bisect.bisect_left(green_starts, t_arr)
            if idx < len(green_starts):
                return green_starts[idx]
            return None  # no future green start in the window

        # Collect TTS per type
        tts_all = []
        tts_cav = []
        tts_hdv = []

        # (optional) only consider certain lanes
        allowed = set(detector_filter) if detector_filter else None

        # Use detector_data's lane field directly to look up in FCD
        for d in self.detector_data:
            lane = self._resolve_lane_key(d)
            if allowed is not None and lane not in allowed:
                continue
            t0 = float(d["begin"]); t1 = float(d["end"])
            if t1 <= t0:
                continue

            # First‑appearance times within this interval
            i0 = bisect.bisect_left(times, t0 - 1e-9)
            i1 = bisect.bisect_right(times, t1 + 1e-9)
            first_seen = {}  # veh_id -> arrival time in this interval

            for i in range(i0, i1):
                t = times[i]
                vehs = self.timestep_lane_positions.get(t, {}).get(lane, {})
                if not vehs:
                    continue
                for vid in vehs.keys():
                    if vid not in first_seen:
                        first_seen[vid] = t

            if not first_seen:
                continue

            # Compute TTS per vehicle
            for vid, t_arr in first_seen.items():
                # Arriving during green => TTS=0
                in_green = any(gs <= t_arr < ge for gs, ge in green_windows)
                if in_green:
                    tts = 0.0
                else:
                    ng = next_green_start(t_arr)
                    tts = max(0.0, (ng - t_arr)) if ng is not None else 0.0

                tts_all.append(tts)
                if is_cav(vid):
                    tts_cav.append(tts)
                else:
                    tts_hdv.append(tts)

        def stats(arr):
            if not arr:
                return {"avg": None, "p50": None, "p95": None, "n": 0}
            a = np.array(arr, dtype=float)
            return {
                "avg": float(a.mean()),
                "p50": float(np.percentile(a, 50)),
                "p95": float(np.percentile(a, 95)),
                "n": int(a.size),
            }

        self.time_to_service_split = {
            "cav": stats(tts_cav),
            "hdv": stats(tts_hdv),
            "all": stats(tts_all),
            "tls_id": chosen_tls,
        }

        if debug:
            print(f"[TTS-byType] tls={chosen_tls}  all(avg={self.time_to_service_split['all']['avg']}) "
                f"CAV(avg={self.time_to_service_split['cav']['avg']}) "
                f"HDV(avg={self.time_to_service_split['hdv']['avg']})")

    def audit_summary(self):
        def _n(d): return sum(len(v) for v in d.values()) if isinstance(d, dict) else 0
        print("\n=== AUDIT SUMMARY ===")
        print(f"samples_total           : {self._audit.get('samples_total', 0)}")
        print(f"samples_drop_selector   : {self._audit.get('samples_drop_selector', 0)}")
        print(f"safety_kept             : {self._audit.get('safety_kept', 0)}")
        print(f"safety_skip             : {self._audit.get('safety_skip', 0)}")
        print(f"lane_changes_gated      : HDV={_n(self.vehicle_lane_history.get('hdv', {}))}  "
            f"CAV={_n(self.vehicle_lane_history.get('cav', {}))}")
        print(f"lane_changes_RAW        : HDV={_n(self.vehicle_lane_history_all.get('hdv', {}))}  "
            f"CAV={_n(self.vehicle_lane_history_all.get('cav', {}))}")
        print(f"PET funnel (changes/kept/no_target/far/no_cross) : "
            f"{self._audit.get('pet_changes',0)}/"
            f"{self._audit.get('pet_kept',0)}/"
            f"{self._audit.get('pet_no_target',0)}/"
            f"{self._audit.get('pet_far',0)}/"
            f"{self._audit.get('pet_no_cross',0)}")
        print(f"TTC/DRAC/Headway counts : "
            f"TTC={len(self.ttc_values.get('hdv',[]))+len(self.ttc_values.get('cav',[]))}, "
            f"DRAC={len(self.drac_values.get('hdv',[]))+len(self.drac_values.get('cav',[]))}, "
            f"HW={len(self.time_headways.get('hdv',[]))+len(self.time_headways.get('cav',[]))}")
        print("======================\n")
            
    def debug_array_sizes(self):
        def _sz(d): 
            if isinstance(d, dict): 
                return {k: (len(v) if isinstance(v, list) else len(v) if hasattr(v,'__len__') else 'NA') for k,v in d.items()}
            return 'NA'
        print("speeds:", {k:len(v) for k,v in self.speeds.items()})
        print("accel :", {k:len(v) for k,v in self.accelerations.items()})
        print("gap   :", {k:len(v) for k,v in self.space_gaps.items()})
        print("hw    :", {k:len(v) for k,v in self.time_headways.items()})
        print("ttc   :", {k:len(v) for k,v in self.ttc_values.items()})
        print("drac  :", {k:len(v) for k,v in self.drac_values.items()})
        print("PET   :", {k:len(v) for k,v in self.pet_list.items()})
        print("travel_times veh:", {k:len(v) for k,v in self.travel_times.items()})

# ## ---------------------------------------------------------- ##

# def main():
#     # Handle input arguments
#     parser = argparse.ArgumentParser('Python SUMO data analysis and visualization tool')
#     parser.add_argument('--scenario_folder',
#         help='Directory path to search for output file to plot: ["sumo_scenarios", "dev/sumo_scenarios"]. Default "sumo_scenarios"',
#         default="sumo_scenarios", nargs="?", type=str)
#     parser.add_argument('--scenario',
#         help='Select SUMO scenario to analyze from with <scenario_folder>/: ["onramp", "i24"]. Default "onramp"',
#         default="onramp", nargs="?", type=str)
#     parser.add_argument('--file',
#         help='File to plot from <scenario_folder>/<scenario>/output/. Default "fcd.xml"',
#         default="fcd.xml", nargs="?", type=str)
#     parser.add_argument('--plotting',
#         help='Flag to create plots from output file. Default true.', 
#             default=True, action='store_true')
#     parser.add_argument('--saving',
#         help='Flag to save plots from output file. Default true.', 
#             default=True, action='store_true')
#     args = parser.parse_args()


if __name__ == "__main__":
    print("not implemented")

    # main()
    # ax = vis.visualize_fcd("sumo_scenarios/onramp/output/fcd.xml", lanes=None, color_by="fuel_gps") # test function
    # ax.figure.savefig("test.png", dpi=300)