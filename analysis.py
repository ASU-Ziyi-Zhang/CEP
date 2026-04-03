"""
Author: Ziyi Zhang Yanbing Wang
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
   - Implemented urban metrics: throughput, average delay, queue length, v/c ratio,
     intersection approach delay, and queue spillback detection, with optional partial
     execution when only detector data is available.
   - Built green-window extraction and lane-occupancy grids to support the above metrics.

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

# Bump this version whenever metric computation logic changes (e.g. queue
# rewrite, new output schema) so that stale .pkl caches are automatically
# invalidated even though the *input* files haven't changed.
_METRICS_CODE_VERSION = 2  # v2: cycle-based queue with summary_rows/cycle_records

# When False, suppress [DEBUG] / informational prints; keep CACHE/EXPORT/WARN/ERROR.
_VERBOSE = False


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

def _infer_scenario_root(file_dir: str) -> str:
    """
    Infer the scenario root directory from an analysis input directory.
    Typical input is <scenario>/output, but callers may also pass <scenario>.
    """
    if not file_dir:
        return file_dir
    norm = os.path.normpath(file_dir)
    if os.path.basename(norm).lower() == "output":
        return os.path.dirname(norm)
    return norm

def _first_existing(paths: List[str]) -> str | None:
    """Return the first existing path from a candidate list."""
    for path in paths:
        if path and os.path.isfile(path):
            return path
    return None

def _discover_urban_support_files(file_dir: str, penetration_tag: str | None = None) -> dict:
    """
    Discover detector/TLS/additional/mapping files for the current scenario.
    """
    scenario_dir = _infer_scenario_root(file_dir)
    scenario_name = os.path.basename(scenario_dir.rstrip("\\/"))

    detector_candidates = []
    if penetration_tag:
        detector_candidates.extend([
            os.path.join(file_dir, f"detector_output_{penetration_tag}.xml"),
            os.path.join(scenario_dir, f"detector_output_{penetration_tag}.xml"),
        ])
    detector_candidates.extend([
        os.path.join(file_dir, "detector_output.xml"),
        os.path.join(scenario_dir, "detector_output.xml"),
    ])
    detector_file = _first_existing(detector_candidates)
    mapping_file = _first_existing([
        os.path.join(scenario_dir, "detector_mapping.json"),
        os.path.join(file_dir, "detector_mapping.json"),
    ])
    add_file = _first_existing([
        os.path.join(scenario_dir, "sensors.add.xml"),
        os.path.join(scenario_dir, "detectors.add.xml"),
    ])
    signal_file = _first_existing([
        os.path.join(scenario_dir, "signal.add.xml"),
    ])
    tls_candidates = []
    if penetration_tag:
        tls_candidates.extend([
            os.path.join(file_dir, f"tls_{penetration_tag}.xml"),
            os.path.join(scenario_dir, f"tls_{penetration_tag}.xml"),
        ])
    tls_candidates.extend([
        os.path.join(file_dir, "tls.xml"),
        os.path.join(scenario_dir, "tls.xml"),
    ])
    tls_file = _first_existing(tls_candidates)
    net_file = _first_existing([
        os.path.join(scenario_dir, f"{scenario_name}.net.xml"),
    ])

    if net_file is None and os.path.isdir(scenario_dir):
        net_candidates = sorted(
            os.path.join(scenario_dir, name)
            for name in os.listdir(scenario_dir)
            if name.endswith(".net.xml")
        )
        net_file = net_candidates[0] if net_candidates else None

    if tls_file is None:
        tls_file = net_file

    return {
        "scenario_dir": scenario_dir,
        "scenario_name": scenario_name,
        "detector_file": detector_file,
        "mapping_file": mapping_file,
        "add_file": add_file,
        "signal_file": signal_file,
        "tls_file": tls_file,
        "net_file": net_file,
    }


TRAFFIC_LIGHT_SCENARIOS = {"chi_clinton_canal", "roosevelt", "roosevelt_simple"}
DEFAULT_TRAFFIC_LIGHT_WARMUP_S = 350.0


def _default_evaluation_start_s(file_dir: str) -> float:
    """
    Match main.py behavior:
    - traffic-light scenarios use a 350 s warm-up
    - highway / merging scenarios evaluate from 0 s
    """
    scenario_dir = _infer_scenario_root(file_dir)
    scenario_name = os.path.basename(scenario_dir.rstrip("\\/")).lower()
    if scenario_name in TRAFFIC_LIGHT_SCENARIOS:
        return DEFAULT_TRAFFIC_LIGHT_WARMUP_S
    return 0.0

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


def _parse_xml_tree_resilient(xml_path: str, label: str = "XML") -> ET.ElementTree:
    """
    Parse an XML file, stripping embedded NUL bytes if needed.

    This makes the post-processing pipeline more tolerant of partially
    corrupted SUMO outputs from prior runs while still surfacing truly
    malformed XML.
    """
    try:
        return ET.parse(xml_path)
    except ET.ParseError:
        with open(xml_path, "rb") as f:
            raw = f.read()
        nul_count = raw.count(b"\x00")
        if nul_count <= 0:
            raise
        cleaned = raw.replace(b"\x00", b"")
        root = ET.fromstring(cleaned)
        print(f"[Warning] Removed {nul_count} NUL bytes from {label} file: {xml_path}")
        return ET.ElementTree(root)

def load_or_build_metrics(
    file_dir: str,
    urban_mode: bool = True,
    lane_selector=None,
    strict_vehicle_filter: str = "sample",
    penetration_tag: str | None = None,
    evaluation_start_s: float | None = None,
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
    eval_start_s = _default_evaluation_start_s(file_dir) if evaluation_start_s is None else float(evaluation_start_s)

    # Cache file name (use penetration tag so caches for different penetrations don't collide)
    pkl = os.path.join(file_dir, f"metrics_{pen_tag}.pkl")

    # ---- Build file list for signature (only existing files affect the hash) ----
    # FCD path (change to your actual file name if different)
    fcd_path = os.path.join(file_dir, f"fcd_{pen_tag}.xml")
    files_for_sig = [fcd_path]

    if urban_mode:
        urban_files = _discover_urban_support_files(file_dir, pen_tag)
        files_for_sig.extend([
            urban_files.get("detector_file"),
            urban_files.get("tls_file"),
            urban_files.get("add_file"),
            urban_files.get("mapping_file"),
            urban_files.get("net_file"),
        ])

    # File signature (mtime + size)
    file_sig, _ = _file_sig(files_for_sig)

    # Config signature (urban flag + filtering configuration + code version)
    cfg = {
        "urban_mode": bool(urban_mode),
        "strict_vehicle_filter": str(strict_vehicle_filter or "sample"),
        "lane_selector": _selector_signature(lane_selector),
        "evaluation_start_s": eval_start_s,
        "code_version": _METRICS_CODE_VERSION,
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
        evaluation_start_s=eval_start_s,
    )
    with open(pkl, "wb") as f:
        dill.dump(m, f)
    print(f"[CACHE] Built metrics and saved {os.path.basename(pkl)}")

    return m

EPS = 1e-9  # for half-open intervals [start, end)


def _build_time_windows(t_min, t_max, window_s=300.0):
    """Generate fixed-width (start, end) time-window tuples."""
    if t_max <= t_min or window_s <= 0:
        return []
    windows = []
    t = t_min
    while t < t_max - 1e-9:
        windows.append((t, min(t + window_s, t_max)))
        t += window_s
    return windows

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
                 penetration_tag: str | None = None, evaluation_start_s: float | None = None):
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
        self.evaluation_start_s = (
            _default_evaluation_start_s(file_dir)
            if evaluation_start_s is None else float(evaluation_start_s)
        )

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
            if isinstance(getattr(self, "simulation_stats", None), dict):
                self.simulation_stats.setdefault("evaluation", {})
                self.simulation_stats["evaluation"]["start_time_s"] = self.evaluation_start_s

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
            urban_files = _discover_urban_support_files(file_dir, self.penetration_tag)
            self.urban_support_files = urban_files
            mapping_file = urban_files.get("mapping_file")
            self.detector_file = urban_files.get("detector_file")
            self.tls_file = urban_files.get("tls_file")
            self.add_file = urban_files.get("add_file")
            self.net_file = urban_files.get("net_file")

            if mapping_file and os.path.exists(mapping_file):
                with open(mapping_file, "r") as f:
                    self.detector_mapping = json.load(f)
            else:
                self.detector_mapping = {}
                print(f"[Warning] Mapping file not found: {mapping_file}")
            
            if self.detector_file and os.path.isfile(self.detector_file) and self.tls_file and os.path.isfile(self.tls_file):
                self.det2lane = self.parse_add_detectors_to_map(self.add_file)
                self.lane_map = self.load_lane_tls_mapping_json(mapping_file)
                if self.lane_map:
                    self.detector_mapping = self._compose_detector_mapping(self.det2lane, self.lane_map)
                elif self.net_file:
                    self.lane_map = self.build_lane_tls_mapping_from_net(self.net_file)
                    self.detector_mapping = self.build_detector_mapping_from_net(self.det2lane, self.net_file)
                else:
                    self.detector_mapping = self._compose_detector_mapping(self.det2lane, self.lane_map)

                self.parse_detector_data(self.detector_file)
                self.parse_tls_data(self.tls_file)
                self._run_city_signal_metrics_if_applicable()
            else:
                print("[Urban] Detector/TLS file not found, skipping signal metrics.")
                self.detector_file, self.tls_file, self.add_file, self.net_file = None, None, None, None
        else:
            print("[Init] Urban mode disabled, skipping detector/tls/add files.")
            self.detector_file, self.tls_file, self.add_file, self.net_file = None, None, None, None
   
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
        skip_timestep = False

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
                skip_timestep = cur_time < self.evaluation_start_s
                if skip_timestep:
                    continue
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
                if skip_timestep:
                    elem.clear()
                    continue
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
                if skip_timestep:
                    elem.clear()
                    prev_time = cur_time
                    continue
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
                        dist_m = max(0.0, float(traj[-1][1]) - float(traj[0][1]))
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

        tree = _parse_xml_tree_resilient(detector_file, label="detector")
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
            if end <= self.evaluation_start_s:
                continue
            begin = max(begin, self.evaluation_start_s)
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

        tree = _parse_xml_tree_resilient(tls_file, label="TLS")
        root = tree.getroot()

        def tag_endswith(elem, name):
            return elem.tag.split('}')[-1] == name

        # --- Case A: timestep-based dump (tls.xml) ---
        timesteps = root.findall(".//timestep")
        if len(timesteps) > 0:
            for ts in timesteps:
                t = float(ts.attrib.get("time", 0.0))
                if t < self.evaluation_start_s:
                    continue

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

        # --- Case B: tlsStates / tlsState dump generated by SaveTLSStates ---
        elif root.tag.split('}')[-1] == "tlsStates" or len(root.findall(".//tlsState")) > 0:
            for item in root.findall(".//tlsState"):
                try:
                    t = float(item.attrib.get("time", 0.0))
                except Exception:
                    continue
                if t < self.evaluation_start_s:
                    continue
                tls_id = item.attrib.get("id") or item.attrib.get("tlsID") or item.attrib.get("name")
                state = item.attrib.get("state")
                if tls_id and state is not None:
                    self.tls_time_states[tls_id].append((t, state))

            for tls_id, seq in self.tls_time_states.items():
                seq.sort(key=lambda x: x[0])
                intervals = _coalesce_tls_states(seq)
                self.tls_intervals[tls_id] = intervals
                self.tls_interval_index[tls_id] = _build_interval_index(intervals)

        else:
            # --- Case C: static tlLogic in net.xml ---
            for tl in root.findall(".//tlLogic"):
                tls_id = tl.attrib.get("id")
                program_id = tl.attrib.get("programID")
                phases = []
                for ph in tl.findall("phase"):
                    st = ph.attrib.get("state")
                    dur = float(ph.attrib.get("duration", 0.0))
                    if st is not None:
                        phases.append({"state": st, "duration": dur})
                if tls_id and phases:
                    # Prefer program 0 when multiple static programs exist for the same TLS.
                    if tls_id not in self.tls_programs or program_id == "0":
                        self.tls_programs[tls_id] = phases

        # Sort time series (for backward compatibility)
        for tls_id in list(self.tls_time_states.keys()):
            self.tls_time_states[tls_id].sort(key=lambda x: x[0])

        if _VERBOSE:
            total = sum(len(v) for v in self.tls_time_states.values())
            print(f"[Urban Signals][DEBUG] tls time-states captured: n_tls={len(self.tls_time_states)}, samples={total}")
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
        groups = None if movement_col is None else [movement_col]
        return self._get_green_windows_for_groups(
            tls_id=tls_id,
            groups=groups,
            count_yellow_as_green=False,
        )

    def _get_green_windows_for_groups(self, tls_id=None, groups=None, count_yellow_as_green=False):
        """
        Build green windows for a specific TLS and optional signal-group list.

        If `groups` is provided, the window is green only when all listed signal
        groups are green simultaneously. If not provided, any green in the state
        string qualifies.
        """
        green_windows = []
        green_set = {'g', 'G'}
        if count_yellow_as_green:
            green_set |= {'y', 'Y'}

        if groups is not None:
            try:
                groups = [int(g) for g in groups]
            except Exception:
                groups = None

        def state_is_green(state):
            if state is None:
                return False
            s = str(state)
            if groups:
                for gi in groups:
                    if gi < 0 or gi >= len(s) or s[gi] not in green_set:
                        return False
                return True
            return any(ch in green_set for ch in s)

        # --- Static program ---
        if self.tls_programs:
            chosen = tls_id or next(iter(self.tls_programs.keys()))
            phases = self.tls_programs.get(chosen, [])
            t = 0.0
            for ph in phases:
                dur = float(ph["duration"])
                if state_is_green(ph.get("state")):
                    green_windows.append((t, t + dur))
                t += dur
            return green_windows, True, t, chosen

        # --- Time-series ---
        if self.tls_intervals:
            chosen = tls_id or next(iter(self.tls_intervals.keys()))
            intervals = self.tls_intervals.get(chosen, [])
            cur_start = None
            for a, b, s in intervals:
                if state_is_green(s):
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
                if state_is_green(st0):
                    green_windows.append((t0, t1))
            return green_windows, False, None, chosen

        return [], False, None, None

    def _resolve_detector_control(self, detector_record, detector_mapping=None):
        """
        Resolve detector metadata into lane / tls / groups.
        """
        if detector_mapping is None and hasattr(self, "detector_mapping"):
            detector_mapping = self.detector_mapping

        det_id = (
            detector_record.get("id")
            or detector_record.get("detector_id")
            or detector_record.get("lane")
        )
        mapped = detector_mapping.get(det_id, {}) if isinstance(detector_mapping, dict) else {}
        lane_id = (
            mapped.get("sumo_lane")
            or mapped.get("lane_id")
            or detector_record.get("lane_id")
            or detector_record.get("sumo_lane")
            or self._resolve_lane_key(detector_record)
        )
        tls_id = mapped.get("tls_id")
        groups = mapped.get("groups")
        if groups is None:
            groups = mapped.get("phase_pos")
        if groups is not None and not isinstance(groups, (list, tuple, set)):
            groups = [groups]
        if groups is not None:
            try:
                groups = [int(g) for g in groups]
            except Exception:
                groups = None
        return det_id, lane_id, tls_id, groups

    def _lane_vehicle_map_at_time(self, t, lane_id, allow_edge_fallback=True):
        """
        Return vehicle->position map for a lane at time `t`.
        Falls back to merging same-edge keys when exact lane keys are absent.
        """
        lane_dict = getattr(self, "timestep_lane_positions", {}).get(t, {})
        vehs = lane_dict.get(lane_id)
        if vehs is not None:
            return vehs
        if not allow_edge_fallback or not lane_id:
            return {}

        import re
        m = re.match(r"(.+)_\d+$", str(lane_id))
        edge_id = m.group(1) if m else str(lane_id)
        merged = {}
        for key, vmap in lane_dict.items():
            mk = re.match(r"(.+)_\d+$", str(key))
            key_edge = mk.group(1) if mk else str(key)
            if key_edge == edge_id:
                for vid, pos in vmap.items():
                    merged[vid] = pos
        return merged

    def _detector_intersection_id(self, det_id, tls_id=None):
        """
        Return a stable intersection identifier for a detector.
        Prefers tls_id (e.g. 'Canal', 'Clinton') over detector-prefix
        so that queue / throughput / delay all use the same ID scheme.
        """
        if tls_id:
            return str(tls_id)
        det_id = str(det_id or "")
        if det_id and "_" in det_id:
            return det_id.split("_", 1)[0]
        return det_id or "unknown"

    def _lane_edge_id(self, lane_id):
        m = re.match(r"(.+)_\d+$", str(lane_id or ""))
        return m.group(1) if m else str(lane_id or "")

    def _edge_direction_labels_from_net(self):
        """
        Infer EB/WB/NB/SB labels from edge geometry in net.xml.
        """
        out = {}
        net_xml_path = getattr(self, "net_file", None)
        if not net_xml_path or not os.path.isfile(net_xml_path):
            return out

        try:
            tree = ET.parse(net_xml_path)
            root = tree.getroot()
        except Exception:
            return out

        def _dir_from_shape(shape_text):
            if not shape_text:
                return None, None
            pts = []
            for token in str(shape_text).strip().split():
                try:
                    x_str, y_str = token.split(",")
                    pts.append((float(x_str), float(y_str)))
                except Exception:
                    continue
            if len(pts) < 2:
                return None, None
            x0, y0 = pts[0]
            x1, y1 = pts[-1]
            dx = x1 - x0
            dy = y1 - y0
            if abs(dx) >= abs(dy):
                return ("EB", "eastbound") if dx >= 0 else ("WB", "westbound")
            return ("NB", "northbound") if dy >= 0 else ("SB", "southbound")

        for edge in root.findall(".//edge"):
            edge_id = edge.attrib.get("id")
            if not edge_id or edge_id.startswith(":"):
                continue
            shape_text = edge.attrib.get("shape")
            if not shape_text:
                lane = edge.find("./lane")
                if lane is not None:
                    shape_text = lane.attrib.get("shape")
            short_label, long_label = _dir_from_shape(shape_text)
            if short_label:
                out[edge_id] = {
                    "direction_short": short_label,
                    "direction_long": long_label,
                }
        return out

    def _infer_cycle_length_from_intervals(self, tls_id):
        """
        Infer fixed cycle length from repeated TLS interval states when no static
        tlLogic program is available.
        """
        intervals = (getattr(self, "tls_intervals", {}) or {}).get(tls_id, [])
        if not intervals or len(intervals) < 4:
            return None

        first_state = intervals[0][2]
        first_dur = float(intervals[0][1] - intervals[0][0])
        starts = []
        for a, b, s in intervals:
            dur = float(b - a)
            if s == first_state and abs(dur - first_dur) <= 1e-6:
                starts.append(float(a))

        if len(starts) >= 2:
            diffs = [starts[i + 1] - starts[i] for i in range(len(starts) - 1) if starts[i + 1] > starts[i]]
            if diffs:
                return float(np.median(diffs))

        # Fallback: use repeated state string regardless of first duration.
        state_starts = defaultdict(list)
        for a, _, s in intervals:
            state_starts[str(s)].append(float(a))
        candidate_diffs = []
        for starts in state_starts.values():
            if len(starts) >= 2:
                candidate_diffs.extend(
                    starts[i + 1] - starts[i] for i in range(len(starts) - 1) if starts[i + 1] > starts[i]
                )
        if candidate_diffs:
            return float(np.median(candidate_diffs))
        return None

    def _debug_city_signal_requirements(self):
        """Print why the dataset is treated as non‑urban‑signal and summarize data scale."""
        has_det = hasattr(self, "detector_data") and isinstance(self.detector_data, list) and len(self.detector_data) > 0
        det_count = len(self.detector_data) if has_det else 0

        greens, is_static, cycle_len, chosen_tls = self._get_green_windows(tls_id=None)
        has_tls_green = len(greens) > 0

        if _VERBOSE:
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
        Only compute the following metrics when the dataset qualifies as an urban signal case:
        - Spillback / Throughput / Average Delay / Queue Length / v/c Ratio

        If not qualified, print the reason. With `allow_partial=True`, run partial metrics that
        require only detector data (e.g., spillback, throughput), even without TLS.
        """
        has_det, has_tls_green = self._debug_city_signal_requirements()

        if has_det and has_tls_green:
            if _VERBOSE:
                print("[Urban Signals] Detector + TLS detected. Computing Throughput / AvgDelay / QueueLength / v/c / Spillback ...")

            # --- Spillback ---
            try:
                self.compute_queue_spillback()
            except Exception as e:
                print(f"[Urban Signals][WARN] compute_queue_spillback failed: {e}")

            # --- Throughput ---
            try:
                self.compute_throughput()
            except Exception as e:
                print(f"[Urban Signals][WARN] compute_throughput failed: {e}")

            # --- Average Delay (intersection-level) ---
            try:
                self.compute_average_delay()
            except Exception as e:
                print(f"[Urban Signals][WARN] compute_average_delay failed: {e}")

            # --- Queue Length ---
            try:
                self.compute_queue_length()
            except Exception as e:
                import traceback
                print(f"[Urban Signals][WARN] compute_queue_length failed: {e}")
                traceback.print_exc()

            # --- v/c Ratio ---
            try:
                self.compute_vc_ratio()
            except Exception as e:
                print(f"[Urban Signals][WARN] compute_vc_ratio failed: {e}")

        else:
            if not has_det and not has_tls_green:
                print("[Urban Signals] Detector intervals missing/empty AND no TLS green windows. Skipping all city-signal metrics.")
            elif not has_det:
                print("[Urban Signals] Detector intervals missing/empty. Skipping city-signal metrics.")
            else:  # not has_tls_green
                print("[Urban Signals] No TLS green windows detected. Skipping city-signal metrics.")

            if allow_partial and has_det and not has_tls_green:
                print("[Urban Signals] Running partial metrics (Spillback, Throughput).")
                try:
                    self.compute_queue_spillback()
                except Exception as e:
                    print(f"[Urban Signals][WARN] partial compute_queue_spillback failed: {e}")
                try:
                    self.compute_throughput()
                except Exception as e:
                    print(f"[Urban Signals][WARN] partial compute_throughput failed: {e}")
   
    # ------------------------------
    # Detector / TLS Mapping Helpers
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

        if _VERBOSE:
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
        if _VERBOSE:
            print(f"[Mapping] lane->TLS loaded: {len(lane_map)} lanes")
        return lane_map

    def build_lane_tls_mapping_from_net(self, net_xml_path):
        """
        Build lane -> {tls_id, groups} mapping directly from SUMO net.xml.

        Each controlled connection contributes its incoming lane
        `<from>_<fromLane>` and the associated `linkIndex` group.
        """
        lane_map = {}
        if not net_xml_path or not os.path.isfile(net_xml_path):
            print(f"[Mapping] WARN: net.xml not found for auto lane->TLS mapping: {net_xml_path}")
            return lane_map

        tree = ET.parse(net_xml_path)
        root = tree.getroot()

        ambiguous = 0
        for conn in root.findall(".//connection"):
            tls_id = conn.attrib.get("tl")
            link_index = conn.attrib.get("linkIndex")
            from_edge = conn.attrib.get("from")
            from_lane = conn.attrib.get("fromLane")

            if not tls_id or link_index is None or from_edge is None or from_lane is None:
                continue

            try:
                group_idx = int(link_index)
            except Exception:
                continue

            lane_id = f"{from_edge}_{from_lane}"
            rec = lane_map.setdefault(lane_id, {"tls_id": tls_id, "groups": []})
            if rec["tls_id"] != tls_id:
                ambiguous += 1
                continue
            if group_idx not in rec["groups"]:
                rec["groups"].append(group_idx)

        for rec in lane_map.values():
            rec["groups"].sort()

        if _VERBOSE:
            print(
                f"[Mapping] lane->TLS auto-built from net: {len(lane_map)} lanes"
                + (f" (ambiguous={ambiguous})" if ambiguous else "")
            )
        return lane_map

    def build_detector_mapping_from_net(self, det2lane, net_xml_path):
        """
        Build detector -> {sumo_lane, tls_id, groups} mapping from detector ids,
        lane ids, and SUMO net.xml controlled connections.

        When detector ids contain movement hints such as `_left`, `_right`, or
        `_straight`, prefer connection groups whose SUMO `dir` matches that hint.
        """
        out = {}
        if not det2lane:
            return out
        if not net_xml_path or not os.path.isfile(net_xml_path):
            print(f"[Mapping] WARN: net.xml not found for auto detector mapping: {net_xml_path}")
            return self._compose_detector_mapping(det2lane, {})

        tree = ET.parse(net_xml_path)
        root = tree.getroot()
        lane_conns = defaultdict(list)
        for conn in root.findall(".//connection"):
            tls_id = conn.attrib.get("tl")
            link_index = conn.attrib.get("linkIndex")
            from_edge = conn.attrib.get("from")
            from_lane = conn.attrib.get("fromLane")
            direction = conn.attrib.get("dir")

            if not tls_id or link_index is None or from_edge is None or from_lane is None:
                continue

            try:
                group_idx = int(link_index)
            except Exception:
                continue

            lane_id = f"{from_edge}_{from_lane}"
            lane_conns[lane_id].append({
                "tls_id": tls_id,
                "group": group_idx,
                "dir": direction,
            })

        def desired_dirs(det_id: str):
            det_id = (det_id or "").lower()
            if "_left" in det_id:
                return {"l", "L"}
            if "_right" in det_id:
                return {"r", "R"}
            if "_straight" in det_id:
                return {"s"}
            return set()

        n_matched = 0
        for det_id, lane_id in det2lane.items():
            conns = lane_conns.get(lane_id, [])
            wanted = desired_dirs(det_id)

            selected = [c for c in conns if c.get("dir") in wanted] if wanted else list(conns)
            if not selected:
                selected = list(conns)

            tls_id = selected[0]["tls_id"] if selected else None
            groups = sorted({c["group"] for c in selected if c["tls_id"] == tls_id}) if tls_id else None
            if selected:
                n_matched += 1

            out[det_id] = {
                "sumo_lane": lane_id,
                "tls_id": tls_id,
                "groups": groups,
            }

        if _VERBOSE:
            print(f"[Mapping] detector->TLS auto-built from net: {len(out)} dets (matched={n_matched})")
        return out

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
        if _VERBOSE:
            print(f"[Mapping] composed: {len(out)} dets (tls-linked={n_ok}, no-tls={n_miss})")
        return out


    # ------------------------------
    # Urban Macro Metrics
    # =========================

    def _get_lane_intersection_map(self):
        """
        Map approach lane_id -> intersection_id (tls_id).

        Uses tls_id from detector_mapping as intersection_id.
        Falls back to 'unknown' when TLS info is unavailable.
        """
        lane_to_int = {}
        detector_mapping = getattr(self, "detector_mapping", None)
        if isinstance(detector_mapping, dict) and detector_mapping:
            for mp in detector_mapping.values():
                lane_id = mp.get("sumo_lane") or mp.get("lane_id")
                tls_id = mp.get("tls_id")
                if lane_id:
                    lane_to_int[lane_id] = tls_id or "unknown"
        elif hasattr(self, "det2lane") and isinstance(self.det2lane, dict):
            lane_map = getattr(self, "lane_map", None)
            for v in self.det2lane.values():
                if v:
                    if isinstance(lane_map, dict) and v in lane_map:
                        lane_to_int[v] = lane_map[v].get("tls_id") or "unknown"
                    else:
                        lane_to_int[v] = "unknown"
        else:
            for d in getattr(self, "detector_data", []):
                lane_id = self._resolve_lane_key(d)
                if lane_id:
                    lane_to_int.setdefault(lane_id, "unknown")
        return lane_to_int

    def _resolve_lane_key(self, d):
        """
        Safely parse the "lane" field in a detector interval record into a SUMO lane key.

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

    # ------------------------------
    # Approach Delay — per approach lane / intersection, time-windowed
    # ------------------------------
    def compute_approach_delay(self, free_flow_speed_hdv=13.0, free_flow_speed_cav=15.0,
                               window_s=300.0):
        """
        Compute approach delay per approach lane, grouped by intersection.

        Each detector-mapped approach lane is treated as a separate approach.
        Delay = actual traversal time - free-flow time (type-specific ff speed).

        Sets
        ----
        self.approach_delay_values = {
            "timeseries": list[dict],   # per intersection per window
            "summary": dict,            # per intersection aggregates
            # Global backward-compatible:
            "hdv": float, "cav": float,
        }
        """
        _empty = {"timeseries": [], "summary": {}, "hdv": 0.0, "cav": 0.0}
        self.approach_delay_values = _empty

        lane_int_map = self._get_lane_intersection_map()
        approach_lanes = set(lane_int_map.keys())

        if not approach_lanes or not getattr(self, "timesteps", None):
            print("Average approach delay - HDV: 0.00s, CAV: 0.00s")
            return

        times = list(self.timesteps)
        t_min = times[0] if times else 0.0
        t_max = times[-1] if times else 0.0

        # Track traversals with approach (lane) and intersection context
        traversals = {}  # (lane_id, veh_id) -> {t0, p0, t1, p1, vtype, int_id}
        for t in self.timesteps:
            lane_dict = self.timestep_lane_positions.get(t, {})
            for lane_id in approach_lanes:
                vehs = lane_dict.get(lane_id)
                if vehs is None:
                    vehs = self._lane_vehicle_map_at_time(t, lane_id, allow_edge_fallback=True)
                if not vehs:
                    continue
                int_id = lane_int_map.get(lane_id, "unknown")
                for veh_id, pos in vehs.items():
                    v_type = self.vehicle_info.get(veh_id, {}).get("type")
                    if v_type not in ("hdv", "cav"):
                        continue
                    key = (lane_id, veh_id)
                    rec = traversals.get(key)
                    if rec is None:
                        traversals[key] = {
                            "t0": float(t), "p0": float(pos),
                            "t1": float(t), "p1": float(pos),
                            "vtype": v_type, "int_id": int_id,
                            "lane_id": lane_id,
                        }
                    else:
                        rec["t1"] = float(t)
                        rec["p1"] = float(pos)

        # Compute delay per traversal, assign to window
        windows = _build_time_windows(t_min, t_max, window_s)
        if not windows:
            windows = [(t_min, t_max)]

        min_distance_m = 10.0
        win_data = defaultdict(lambda: {"delays_cav": [], "delays_hdv": []})
        global_delays = {"cav": [], "hdv": []}

        for rec in traversals.values():
            travel_time = max(0.0, rec["t1"] - rec["t0"])
            dist = max(0.0, rec["p1"] - rec["p0"])
            if dist < min_distance_m or travel_time <= 0.0:
                continue
            v_ff = free_flow_speed_hdv if rec["vtype"] == "hdv" else free_flow_speed_cav
            ff_time = dist / max(v_ff, 0.1)
            delay = max(0.0, travel_time - ff_time)

            w_idx = min(int((rec["t1"] - t_min) / window_s), len(windows) - 1) if window_s > 0 else 0
            win_data[(rec["int_id"], w_idx)][f"delays_{rec['vtype']}"].append(delay)
            global_delays[rec["vtype"]].append(delay)

        # Build timeseries
        timeseries = []
        for (int_id, w_idx), wd in sorted(win_data.items()):
            ws, we = windows[min(w_idx, len(windows) - 1)]
            all_d = wd["delays_cav"] + wd["delays_hdv"]
            timeseries.append({
                "intersection_id": int_id,
                "window_start": ws,
                "window_end": we,
                "approach_delay_all": float(np.mean(all_d)) if all_d else 0.0,
                "approach_delay_cav": float(np.mean(wd["delays_cav"])) if wd["delays_cav"] else 0.0,
                "approach_delay_hdv": float(np.mean(wd["delays_hdv"])) if wd["delays_hdv"] else 0.0,
                "n_all": len(all_d),
                "n_cav": len(wd["delays_cav"]),
                "n_hdv": len(wd["delays_hdv"]),
            })

        # Per-intersection summary
        int_s = defaultdict(lambda: {"d_cav": [], "d_hdv": []})
        for row in timeseries:
            s = int_s[row["intersection_id"]]
            if row["n_cav"] > 0:
                s["d_cav"].append(row["approach_delay_cav"])
            if row["n_hdv"] > 0:
                s["d_hdv"].append(row["approach_delay_hdv"])

        summary = {}
        for int_id, s in int_s.items():
            all_d = s["d_cav"] + s["d_hdv"]
            summary[int_id] = {
                "intersection_id": int_id,
                "approach_delay_all": float(np.mean(all_d)) if all_d else 0.0,
                "approach_delay_cav": float(np.mean(s["d_cav"])) if s["d_cav"] else 0.0,
                "approach_delay_hdv": float(np.mean(s["d_hdv"])) if s["d_hdv"] else 0.0,
            }

        # Global (backward-compatible)
        self.approach_delay_values = {
            "timeseries": timeseries,
            "summary": summary,
            "hdv": float(np.mean(global_delays["hdv"])) if global_delays["hdv"] else 0.0,
            "cav": float(np.mean(global_delays["cav"])) if global_delays["cav"] else 0.0,
        }

        if _VERBOSE:
            print(f"Average approach delay - HDV: {self.approach_delay_values['hdv']:.2f}s, "
                  f"CAV: {self.approach_delay_values['cav']:.2f}s, "
                  f"{len(summary)} intersections, {len(timeseries)} window-records")

    # =========================
    # Queue Spillback Occurrence
    # =========================

    def compute_queue_spillback(self, occ_threshold=0.8):
        """
        Compute queue spillback metrics.
        A spillback is recorded when detector occupancy exceeds a threshold
        for a continuous period with zero entries, indicating vehicles are backed up.
        """

        self.spillback_events = []   # full event log
        by_detector = defaultdict(list)
        for d in self.detector_data:
            det_key = d.get("id") or d.get("detector_id") or d.get("lane")
            by_detector[det_key].append(d)

        for det_key, recs in by_detector.items():
            current_event = None
            recs = sorted(recs, key=lambda x: (float(x.get("begin", 0.0)), float(x.get("end", 0.0))))

            for d in recs:
                lane_label = d.get("lane") or det_key
                occ = float(d["occ"]) / 100.0   # convert % to fraction
                nVeh = int(d["nVeh"])
                begin = float(d["begin"])
                end = float(d["end"])

                if occ >= occ_threshold and nVeh == 0:
                    if current_event is None:
                        current_event = {
                            "lane": lane_label,
                            "detector_id": det_key,
                            "start": begin,
                            "end": end,
                            "max_occ": occ,
                        }
                    else:
                        current_event["end"] = end
                        current_event["max_occ"] = max(current_event["max_occ"], occ)
                else:
                    if current_event is not None:
                        duration = current_event["end"] - current_event["start"]
                        current_event["duration"] = duration
                        self.spillback_events.append(current_event)
                        current_event = None

            if current_event is not None:
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

        if _VERBOSE:
            print("[Spillback Events]")
            for e in self.spillback_events:
                print(f"Lane={e['lane']} | Start={e['start']:.1f}s | End={e['end']:.1f}s | "
                    f"Duration={e['duration']:.1f}s | MaxOcc={e['max_occ']:.2f}")

    # =========================
    # Throughput (Discharged Vehicles) — intersection-level, time-windowed
    # =========================
    def compute_throughput(self, window_s=300.0):
        """
        Compute intersection throughput from detector data.

        Aggregates nVeh per intersection per time window, split by vehicle type
        (when FCD type info is available).

        Sets
        ----
        self.throughput_values = {
            "timeseries": list[dict],   # per intersection per window
            "summary": dict,            # per intersection aggregates
            # Global backward-compatible fields:
            "total_vehicles": int,
            "total_hours": float,
            "vehicles_per_hour": float,
            "cav_vehicles": int, "hdv_vehicles": int,
            "cav_vehicles_per_hour": float, "hdv_vehicles_per_hour": float,
        }
        """
        _empty = {
            "timeseries": [], "summary": {},
            "total_vehicles": 0, "total_hours": 0.0,
            "vehicles_per_hour": 0.0,
            "cav_vehicles": 0, "hdv_vehicles": 0,
            "cav_vehicles_per_hour": 0.0, "hdv_vehicles_per_hour": 0.0,
        }
        if not getattr(self, "detector_data", None):
            print("[Throughput] No detector data available.")
            self.throughput_values = _empty
            return

        detector_mapping = getattr(self, "detector_mapping", None)
        has_fcd = hasattr(self, "timesteps") and hasattr(self, "timestep_lane_positions")

        def _det_intersection(det_id):
            if isinstance(detector_mapping, dict):
                mp = detector_mapping.get(det_id, {})
                return mp.get("tls_id") or "unknown"
            return "unknown"

        # Aggregate by (intersection_id, begin, end)
        interval_map = defaultdict(lambda: {"nVeh": 0, "cav": 0, "hdv": 0})

        for d in self.detector_data:
            begin = float(d["begin"])
            end = float(d["end"])
            nVeh = int(d.get("nVeh", 0))
            det_id = d.get("id") or d.get("detector_id") or d.get("lane")
            int_id = _det_intersection(det_id)

            interval_map[(int_id, begin, end)]["nVeh"] += nVeh

            # FCD-based type split
            if has_fcd and nVeh > 0:
                det_id_r, lane_id, _, _ = self._resolve_detector_control(d, detector_mapping)
                if lane_id:
                    times = list(self.timesteps)
                    i0 = bisect.bisect_left(times, begin - 1e-9)
                    i1 = bisect.bisect_right(times, end + 1e-9)
                    seen_vehs = set()
                    for i in range(i0, i1):
                        t = times[i]
                        vehs = self._lane_vehicle_map_at_time(t, lane_id, allow_edge_fallback=True)
                        if vehs:
                            seen_vehs.update(vehs.keys())
                    for vid in seen_vehs:
                        if "_cav" in (vid or "").lower():
                            interval_map[(int_id, begin, end)]["cav"] += 1
                        else:
                            interval_map[(int_id, begin, end)]["hdv"] += 1

        if not interval_map:
            self.throughput_values = _empty
            return

        # Global time range
        all_begins = [k[1] for k in interval_map]
        all_ends = [k[2] for k in interval_map]
        t_min = min(all_begins)
        t_max = max(all_ends)

        # Build fixed time windows
        windows = _build_time_windows(t_min, t_max, window_s)
        if not windows:
            windows = [(t_min, t_max)]

        # Aggregate per (intersection_id, window_idx)
        window_agg = defaultdict(lambda: {"nVeh": 0, "cav": 0, "hdv": 0})
        for (int_id, begin, end), counts in interval_map.items():
            mid = (begin + end) / 2.0
            w_idx = min(int((mid - t_min) / window_s), len(windows) - 1) if window_s > 0 else 0
            window_agg[(int_id, w_idx)]["nVeh"] += counts["nVeh"]
            window_agg[(int_id, w_idx)]["cav"] += counts["cav"]
            window_agg[(int_id, w_idx)]["hdv"] += counts["hdv"]

        # Build timeseries
        timeseries = []
        for (int_id, w_idx), counts in sorted(window_agg.items()):
            ws, we = windows[min(w_idx, len(windows) - 1)]
            window_hours = max((we - ws) / 3600.0, 1e-9)
            timeseries.append({
                "intersection_id": int_id,
                "window_start": ws,
                "window_end": we,
                "vehicles": counts["nVeh"],
                "vehicles_per_hour": counts["nVeh"] / window_hours,
                "cav": counts["cav"],
                "hdv": counts["hdv"],
            })

        # Per-intersection summary
        int_agg = defaultdict(lambda: {"total_veh": 0, "total_cav": 0, "total_hdv": 0, "vph_list": []})
        for row in timeseries:
            a = int_agg[row["intersection_id"]]
            a["total_veh"] += row["vehicles"]
            a["total_cav"] += row["cav"]
            a["total_hdv"] += row["hdv"]
            a["vph_list"].append(row["vehicles_per_hour"])

        summary = {}
        for int_id, a in int_agg.items():
            summary[int_id] = {
                "intersection_id": int_id,
                "total_vehicles": a["total_veh"],
                "total_cav": a["total_cav"],
                "total_hdv": a["total_hdv"],
                "mean_vehicles_per_hour": float(np.mean(a["vph_list"])) if a["vph_list"] else 0.0,
                "peak_vehicles_per_hour": float(max(a["vph_list"])) if a["vph_list"] else 0.0,
            }

        # Global summary (backward-compatible)
        total_veh = sum(a["total_veh"] for a in int_agg.values())
        total_cav = sum(a["total_cav"] for a in int_agg.values())
        total_hdv = sum(a["total_hdv"] for a in int_agg.values())
        total_hours = max(0.0, (t_max - t_min) / 3600.0)

        self.throughput_values = {
            "timeseries": timeseries,
            "summary": summary,
            "total_vehicles": total_veh,
            "total_hours": total_hours,
            "vehicles_per_hour": total_veh / total_hours if total_hours > 0 else 0.0,
            "cav_vehicles": total_cav,
            "hdv_vehicles": total_hdv,
            "cav_vehicles_per_hour": total_cav / total_hours if total_hours > 0 else 0.0,
            "hdv_vehicles_per_hour": total_hdv / total_hours if total_hours > 0 else 0.0,
        }

        if _VERBOSE:
            print(f"[Throughput] total={total_veh} veh, {self.throughput_values['vehicles_per_hour']:.0f} veh/hr "
                  f"(CAV={total_cav}, HDV={total_hdv}) over {total_hours:.2f} hr, "
                  f"{len(summary)} intersections, {len(timeseries)} window-records")

    # =========================
    # Average Delay (intersection-level, pooled) — time-windowed
    # =========================
    def compute_average_delay(self, free_flow_speed=13.0, stop_speed_thresh=2.0, window_s=300.0):
        """
        Compute pooled intersection-level average delay using FCD data.

        For each vehicle traversing any approach lane:
          delay = actual_travel_time - free_flow_time
        Also computes stopped delay (time at speed < stop_speed_thresh).

        Results are grouped by intersection_id and time window.

        Sets
        ----
        self.average_delay_values = {
            "timeseries": list[dict],   # per intersection per window
            "summary": dict,            # per intersection aggregates
            # Global backward-compatible fields:
            "avg_delay_all": float, "avg_delay_cav": float, "avg_delay_hdv": float,
            "avg_stopped_delay_all": float, ...,
            "n_all": int, "n_cav": int, "n_hdv": int,
        }
        """
        _empty = {
            "timeseries": [], "summary": {},
            "avg_delay_all": 0.0, "avg_delay_cav": 0.0, "avg_delay_hdv": 0.0,
            "avg_stopped_delay_all": 0.0, "avg_stopped_delay_cav": 0.0,
            "avg_stopped_delay_hdv": 0.0,
            "n_all": 0, "n_cav": 0, "n_hdv": 0,
        }
        self.average_delay_values = _empty

        lane_int_map = self._get_lane_intersection_map()
        approach_lanes = set(lane_int_map.keys())

        if not approach_lanes or not getattr(self, "timesteps", None):
            print("[AvgDelay] No approach lanes or FCD data.")
            return

        times = list(self.timesteps)
        dt = times[1] - times[0] if len(times) > 1 else 0.1

        # Track each vehicle's traversal with intersection context
        # key = (lane_id, veh_id)
        traversals = {}

        for t in times:
            lane_dict = self.timestep_lane_positions.get(t, {})
            for lane_id in approach_lanes:
                vehs = lane_dict.get(lane_id)
                if vehs is None:
                    vehs = self._lane_vehicle_map_at_time(t, lane_id, allow_edge_fallback=True)
                if not vehs:
                    continue
                int_id = lane_int_map.get(lane_id, "unknown")
                for veh_id, pos in vehs.items():
                    key = (lane_id, veh_id)
                    rec = traversals.get(key)
                    if rec is None:
                        vtype = self.vehicle_info.get(veh_id, {}).get("type")
                        if vtype not in ("hdv", "cav"):
                            vtype = "cav" if "_cav" in (veh_id or "").lower() else "hdv"
                        traversals[key] = {
                            "t0": float(t), "p0": float(pos),
                            "t1": float(t), "p1": float(pos),
                            "stopped_time": 0.0, "prev_pos": float(pos),
                            "vtype": vtype, "int_id": int_id,
                        }
                    else:
                        speed = abs(float(pos) - rec["prev_pos"]) / max(dt, 1e-6)
                        if speed < stop_speed_thresh:
                            rec["stopped_time"] += dt
                        rec["t1"] = float(t)
                        rec["p1"] = float(pos)
                        rec["prev_pos"] = float(pos)

        # Compute delay per traversal and assign to time window
        min_distance_m = 10.0
        t_min = times[0] if times else 0.0
        t_max = times[-1] if times else 0.0
        windows = _build_time_windows(t_min, t_max, window_s)
        if not windows:
            windows = [(t_min, t_max)]

        # Collect per (int_id, w_idx): lists of delays
        win_data = defaultdict(lambda: {"delays_cav": [], "delays_hdv": [],
                                         "stopped_cav": [], "stopped_hdv": []})
        global_delays = {"cav": [], "hdv": []}
        global_stopped = {"cav": [], "hdv": []}

        for rec in traversals.values():
            travel_time = max(0.0, rec["t1"] - rec["t0"])
            dist = max(0.0, rec["p1"] - rec["p0"])
            if dist < min_distance_m or travel_time <= 0.0:
                continue
            ff_time = dist / max(free_flow_speed, 0.1)
            delay = max(0.0, travel_time - ff_time)
            vtype = rec["vtype"]
            int_id = rec["int_id"]

            # Assign to window by departure time
            w_idx = min(int((rec["t1"] - t_min) / window_s), len(windows) - 1) if window_s > 0 else 0
            wd = win_data[(int_id, w_idx)]
            wd[f"delays_{vtype}"].append(delay)
            wd[f"stopped_{vtype}"].append(rec["stopped_time"])
            global_delays[vtype].append(delay)
            global_stopped[vtype].append(rec["stopped_time"])

        # Build timeseries
        timeseries = []
        for (int_id, w_idx), wd in sorted(win_data.items()):
            ws, we = windows[min(w_idx, len(windows) - 1)]
            all_d = wd["delays_cav"] + wd["delays_hdv"]
            all_s = wd["stopped_cav"] + wd["stopped_hdv"]
            timeseries.append({
                "intersection_id": int_id,
                "window_start": ws,
                "window_end": we,
                "avg_delay_all": float(np.mean(all_d)) if all_d else 0.0,
                "avg_delay_cav": float(np.mean(wd["delays_cav"])) if wd["delays_cav"] else 0.0,
                "avg_delay_hdv": float(np.mean(wd["delays_hdv"])) if wd["delays_hdv"] else 0.0,
                "avg_stopped_delay_all": float(np.mean(all_s)) if all_s else 0.0,
                "n_all": len(all_d),
                "n_cav": len(wd["delays_cav"]),
                "n_hdv": len(wd["delays_hdv"]),
            })

        # Per-intersection summary
        int_summary = defaultdict(lambda: {"delays_all": [], "delays_cav": [], "delays_hdv": [],
                                            "stopped_all": []})
        for row in timeseries:
            s = int_summary[row["intersection_id"]]
            if row["n_all"] > 0:
                s["delays_all"].append(row["avg_delay_all"])
            if row["n_cav"] > 0:
                s["delays_cav"].append(row["avg_delay_cav"])
            if row["n_hdv"] > 0:
                s["delays_hdv"].append(row["avg_delay_hdv"])
            if row["n_all"] > 0:
                s["stopped_all"].append(row["avg_stopped_delay_all"])

        summary = {}
        for int_id, s in int_summary.items():
            summary[int_id] = {
                "intersection_id": int_id,
                "avg_delay_all": float(np.mean(s["delays_all"])) if s["delays_all"] else 0.0,
                "avg_delay_cav": float(np.mean(s["delays_cav"])) if s["delays_cav"] else 0.0,
                "avg_delay_hdv": float(np.mean(s["delays_hdv"])) if s["delays_hdv"] else 0.0,
                "avg_stopped_delay_all": float(np.mean(s["stopped_all"])) if s["stopped_all"] else 0.0,
            }

        # Global (backward-compatible)
        all_delays = global_delays["cav"] + global_delays["hdv"]
        all_stopped = global_stopped["cav"] + global_stopped["hdv"]

        self.average_delay_values = {
            "timeseries": timeseries,
            "summary": summary,
            "avg_delay_all": float(np.mean(all_delays)) if all_delays else 0.0,
            "avg_delay_cav": float(np.mean(global_delays["cav"])) if global_delays["cav"] else 0.0,
            "avg_delay_hdv": float(np.mean(global_delays["hdv"])) if global_delays["hdv"] else 0.0,
            "avg_stopped_delay_all": float(np.mean(all_stopped)) if all_stopped else 0.0,
            "avg_stopped_delay_cav": float(np.mean(global_stopped["cav"])) if global_stopped["cav"] else 0.0,
            "avg_stopped_delay_hdv": float(np.mean(global_stopped["hdv"])) if global_stopped["hdv"] else 0.0,
            "n_all": len(all_delays),
            "n_cav": len(global_delays["cav"]),
            "n_hdv": len(global_delays["hdv"]),
        }

        if _VERBOSE:
            print(f"[AvgDelay] all={self.average_delay_values['avg_delay_all']:.2f}s "
                  f"(CAV={self.average_delay_values['avg_delay_cav']:.2f}s, "
                  f"HDV={self.average_delay_values['avg_delay_hdv']:.2f}s) "
                  f"n={self.average_delay_values['n_all']}, "
                  f"{len(summary)} intersections, {len(timeseries)} window-records")

    # =========================
    # Queue Count (queued vehicles) — per intersection, time-windowed
    # =========================
    def compute_queue_length(self, stop_speed_thresh=2.0, window_s=300.0):
        """
        Estimate queue by approach as the maximum queued vehicles during red for
        each signal cycle, then summarize over all cycles.

        The output keeps the existing `queue_length_values` container, but queue
        is no longer defined on fixed 5-minute windows. Instead:
          - cycle_records: one row per intersection + approach + cycle
          - summary_rows: one row per intersection + approach with Average/Max Queue
        """
        _empty = {
            "timeseries": [],
            "summary": {},
            "summary_rows": [],
            "cycle_records": [],
            "avg_queue_vehicles": 0.0,
            "max_queue_vehicles": 0,
            "avg_queue_cav": 0.0,
            "avg_queue_hdv": 0.0,
        }
        self.queue_length_values = _empty

        detector_mapping = getattr(self, "detector_mapping", None)
        if not isinstance(detector_mapping, dict) or not detector_mapping:
            print("[QueueCount] detector_mapping missing; skipping queue metric.")
            return
        if not getattr(self, "timesteps", None):
            print("[QueueCount] No FCD timesteps available.")
            return
        edge_dirs = self._edge_direction_labels_from_net()

        approach_specs = {}
        _skip_no_lane = 0
        _skip_no_tls = 0
        _skip_no_groups = 0
        for det_id, mp in detector_mapping.items():
            lane_id = mp.get("sumo_lane") or mp.get("lane_id")
            tls_id = mp.get("tls_id")
            groups = mp.get("groups")
            if lane_id is None:
                _skip_no_lane += 1
                continue
            if tls_id is None:
                _skip_no_tls += 1
                continue
            if groups is None:
                _skip_no_groups += 1
                continue
            try:
                group_list = sorted({int(g) for g in groups})
            except Exception:
                continue
            if not group_list:
                continue

            edge_id = self._lane_edge_id(lane_id)
            intersection_id = self._detector_intersection_id(det_id, tls_id=tls_id)
            edge_dir = edge_dirs.get(edge_id, {})
            direction_short = edge_dir.get("direction_short") or edge_id
            direction_long = edge_dir.get("direction_long") or edge_id

            key = (intersection_id, tls_id, edge_id)
            rec = approach_specs.setdefault(
                key,
                {
                    "intersection_id": intersection_id,
                    "tls_id": tls_id,
                    "approach_edge": edge_id,
                    "approach_direction": direction_short,
                    "approach_direction_long": direction_long,
                    "lanes": {},
                },
            )
            rec["lanes"][lane_id] = {
                "groups": tuple(group_list),
                "detector_id": det_id,
            }

        if not approach_specs:
            print(f"[QueueCount] No detector-linked approaches with TLS groups. "
                  f"(detectors={len(detector_mapping)}, skip_no_lane={_skip_no_lane}, "
                  f"skip_no_tls={_skip_no_tls}, skip_no_groups={_skip_no_groups})")
            return

        cycle_lengths = {}
        for rec in approach_specs.values():
            tls_id = rec["tls_id"]
            if tls_id in cycle_lengths:
                continue
            phases = (getattr(self, "tls_programs", {}) or {}).get(tls_id, [])
            cycle_len = 0.0
            for ph in phases:
                try:
                    cycle_len += float(ph.get("duration", 0.0))
                except Exception:
                    continue
            if cycle_len <= 0:
                inferred = self._infer_cycle_length_from_intervals(tls_id)
                if inferred and inferred > 0:
                    cycle_len = float(inferred)
            if cycle_len > 0:
                cycle_lengths[tls_id] = cycle_len

        if not cycle_lengths:
            print("[QueueCount] Could not determine signal cycle lengths.")
            return

        times = list(self.timesteps)
        if len(times) < 2:
            return

        def lane_is_red(tls_id, groups, t):
            state = self.tls_state(tls_id, t)
            if state is None:
                return False
            try:
                return all((gi >= len(state)) or (not _is_green_char(state[gi])) for gi in groups)
            except Exception:
                return False

        cycle_max = {}
        prev_time = None

        for t in times:
            if prev_time is None:
                prev_time = t
                continue
            dt = max(1e-9, float(t) - float(prev_time))
            lane_dict = self.timestep_lane_positions.get(t, {})
            prev_lane_dict = self.timestep_lane_positions.get(prev_time, {})
            if not lane_dict:
                prev_time = t
                continue

            for (intersection_id, tls_id, edge_id), rec in approach_specs.items():
                cycle_len = cycle_lengths.get(tls_id)
                if not cycle_len or cycle_len <= 0:
                    continue

                cycle_index = int(max(0.0, float(t)) // cycle_len)
                cycle_start = cycle_index * cycle_len
                cycle_end = cycle_start + cycle_len
                approach_key = (
                    intersection_id,
                    rec["approach_direction"],
                    edge_id,
                    tls_id,
                    cycle_index,
                    cycle_start,
                    cycle_end,
                )

                queued_now = 0
                for lane_id, lane_meta in rec["lanes"].items():
                    groups = lane_meta["groups"]
                    if not lane_is_red(tls_id, groups, t):
                        continue
                    vehs_now = lane_dict.get(lane_id)
                    if vehs_now is None:
                        vehs_now = self._lane_vehicle_map_at_time(t, lane_id, allow_edge_fallback=True)
                    vehs_prev = prev_lane_dict.get(lane_id)
                    if vehs_prev is None:
                        vehs_prev = self._lane_vehicle_map_at_time(prev_time, lane_id, allow_edge_fallback=True)
                    if not vehs_now:
                        continue
                    for veh_id, pos_now in vehs_now.items():
                        pos_prev = (vehs_prev or {}).get(veh_id)
                        if pos_prev is None:
                            continue
                        speed = abs(float(pos_now) - float(pos_prev)) / dt
                        if speed < float(stop_speed_thresh):
                            queued_now += 1

                cycle_max[approach_key] = max(cycle_max.get(approach_key, 0), int(queued_now))
            prev_time = t

        cycle_records = []
        grouped = defaultdict(list)
        for (
            intersection_id,
            approach_direction,
            edge_id,
            tls_id,
            cycle_index,
            cycle_start,
            cycle_end,
        ), max_q in sorted(cycle_max.items(), key=lambda item: item[0]):
            cycle_records.append({
                "intersection_id": intersection_id,
                "tls_id": tls_id,
                "approach_edge": edge_id,
                "approach_direction": approach_direction,
                "cycle_index": int(cycle_index),
                "cycle_start": float(cycle_start),
                "cycle_end": float(cycle_end),
                "cycle_max_queue": int(max_q),
            })
            grouped[(intersection_id, approach_direction, edge_id, tls_id)].append(int(max_q))

        summary_rows = []
        summary = {}
        all_avg = []
        all_max = []
        for (intersection_id, approach_direction, edge_id, tls_id), vals in sorted(grouped.items()):
            arr = np.array(vals, dtype=float)
            avg_q = float(arr.mean()) if arr.size else 0.0
            max_q = int(arr.max()) if arr.size else 0
            row = {
                "intersection_id": intersection_id,
                "tls_id": tls_id,
                "approach_edge": edge_id,
                "approach_direction": approach_direction,
                "Average Queue": avg_q,
                "Max Queue": max_q,
                "cycle_count": int(arr.size),
            }
            summary_rows.append(row)
            summary.setdefault(intersection_id, []).append(row)
            all_avg.append(avg_q)
            all_max.append(max_q)

        self.queue_length_values = {
            "timeseries": [],
            "summary": summary,
            "summary_rows": summary_rows,
            "cycle_records": cycle_records,
            "avg_queue_vehicles": float(np.mean(all_avg)) if all_avg else 0.0,
            "max_queue_vehicles": int(max(all_max)) if all_max else 0,
            "avg_queue_cav": 0.0,
            "avg_queue_hdv": 0.0,
        }

        if _VERBOSE:
            print(
                f"[QueueCount] cycle-based queue computed for {len(summary_rows)} approaches "
                f"across {len(summary)} intersections; global avg={self.queue_length_values['avg_queue_vehicles']:.1f} "
                f"veh, max={self.queue_length_values['max_queue_vehicles']} veh"
            )

    # =========================
    # v/c Ratio (Volume / Capacity) — per intersection, time-windowed
    # =========================
    def compute_vc_ratio(self, saturation_flow_rate=1800.0, window_s=300.0):
        """
        Estimate volume-to-capacity ratio per intersection per time window.

        volume = detector flow rate (veh/hr per lane group)
        capacity = (effective_green / cycle_length) * saturation_flow_rate

        Sets
        ----
        self.vc_ratio_values = {
            "timeseries": list[dict],   # per intersection per window
            "summary": dict,            # per intersection aggregates
            # Global backward-compatible:
            "vc_ratio_avg": float,
            "vc_ratio_max": float,
            "volume_vph": float,
            "capacity_vph": float,
            "per_detector": list[dict],
        }
        """
        _empty = {
            "timeseries": [], "summary": {},
            "vc_ratio_avg": 0.0, "vc_ratio_max": 0.0,
            "volume_vph": 0.0, "capacity_vph": 0.0,
            "per_detector": [],
        }
        self.vc_ratio_values = _empty

        if not getattr(self, "detector_data", None):
            print("[v/c Ratio] No detector data available.")
            return

        detector_mapping = getattr(self, "detector_mapping", None)

        def _det_intersection(det_id):
            if isinstance(detector_mapping, dict):
                mp = detector_mapping.get(det_id, {})
                return mp.get("tls_id") or "unknown"
            return "unknown"

        # g/C cache
        gc_cache = {}

        def get_gc_ratio(tls_id, groups):
            cache_key = (tls_id, tuple(groups or ()))
            if cache_key in gc_cache:
                return gc_cache[cache_key]
            green_windows, is_static, cycle_len, chosen_tls = self._get_green_windows_for_groups(
                tls_id=tls_id, groups=groups)
            if not green_windows or not cycle_len or cycle_len <= 0:
                if green_windows:
                    total_green = sum(ge - gs for gs, ge in green_windows)
                    total_span = green_windows[-1][1] - green_windows[0][0]
                    gc = total_green / total_span if total_span > 0 else 0.5
                else:
                    gc = 0.5
            else:
                total_green = sum(ge - gs for gs, ge in green_windows)
                gc = total_green / cycle_len
            gc = max(0.01, min(1.0, gc))
            gc_cache[cache_key] = gc
            return gc

        # Aggregate per (det_id, window_idx)
        # First find global time range
        all_begins = [float(d["begin"]) for d in self.detector_data]
        all_ends = [float(d["end"]) for d in self.detector_data]
        t_min = min(all_begins)
        t_max = max(all_ends)
        windows = _build_time_windows(t_min, t_max, window_s)
        if not windows:
            windows = [(t_min, t_max)]

        # Per (det_id, w_idx) accumulator
        det_win_flows = defaultdict(lambda: {"nVeh": 0, "duration_s": 0.0,
                                              "tls_id": None, "groups": None, "int_id": "unknown"})

        for d in self.detector_data:
            det_id, lane_id, tls_id, groups = self._resolve_detector_control(d, detector_mapping)
            begin = float(d["begin"])
            end = float(d["end"])
            nVeh = int(d.get("nVeh", 0))
            duration = end - begin
            if duration <= 0:
                continue
            int_id = _det_intersection(det_id)
            mid = (begin + end) / 2.0
            w_idx = min(int((mid - t_min) / window_s), len(windows) - 1) if window_s > 0 else 0

            rec = det_win_flows[(det_id, w_idx)]
            rec["nVeh"] += nVeh
            rec["duration_s"] += duration
            rec["int_id"] = int_id
            if tls_id:
                rec["tls_id"] = tls_id
                rec["groups"] = groups

        # Per (int_id, w_idx) aggregate v/c
        int_win_vc = defaultdict(lambda: {"vc_list": [], "vol_total": 0.0, "cap_total": 0.0})
        per_detector = []
        global_vc_list = []
        global_vol = 0.0
        global_cap = 0.0

        for (det_id, w_idx), rec in det_win_flows.items():
            if rec["duration_s"] <= 0:
                continue
            volume_vph = rec["nVeh"] / (rec["duration_s"] / 3600.0)
            gc = get_gc_ratio(rec["tls_id"], rec["groups"]) if rec["tls_id"] else 0.5
            capacity_vph = gc * saturation_flow_rate
            vc = volume_vph / capacity_vph if capacity_vph > 0 else 0.0

            int_id = rec["int_id"]
            iwv = int_win_vc[(int_id, w_idx)]
            iwv["vc_list"].append(vc)
            iwv["vol_total"] += volume_vph
            iwv["cap_total"] += capacity_vph

            global_vc_list.append(vc)
            global_vol += volume_vph
            global_cap += capacity_vph

            per_detector.append({
                "detector_id": det_id,
                "window_idx": w_idx,
                "intersection_id": int_id,
                "volume_vph": round(volume_vph, 1),
                "capacity_vph": round(capacity_vph, 1),
                "gc_ratio": round(gc, 3),
                "vc_ratio": round(vc, 3),
            })

        # Build timeseries
        timeseries = []
        for (int_id, w_idx), iwv in sorted(int_win_vc.items()):
            ws, we = windows[min(w_idx, len(windows) - 1)]
            timeseries.append({
                "intersection_id": int_id,
                "window_start": ws,
                "window_end": we,
                "vc_ratio_avg": float(np.mean(iwv["vc_list"])) if iwv["vc_list"] else 0.0,
                "vc_ratio_max": float(max(iwv["vc_list"])) if iwv["vc_list"] else 0.0,
                "volume_vph": round(iwv["vol_total"], 1),
                "capacity_vph": round(iwv["cap_total"], 1),
            })

        # Per-intersection summary
        int_agg = defaultdict(lambda: {"vc_avg_list": [], "vc_max_list": []})
        for row in timeseries:
            a = int_agg[row["intersection_id"]]
            a["vc_avg_list"].append(row["vc_ratio_avg"])
            a["vc_max_list"].append(row["vc_ratio_max"])

        summary = {}
        for int_id, a in int_agg.items():
            summary[int_id] = {
                "intersection_id": int_id,
                "vc_ratio_avg": float(np.mean(a["vc_avg_list"])) if a["vc_avg_list"] else 0.0,
                "vc_ratio_max": float(max(a["vc_max_list"])) if a["vc_max_list"] else 0.0,
            }

        # Global (backward-compatible)
        self.vc_ratio_values = {
            "timeseries": timeseries,
            "summary": summary,
            "vc_ratio_avg": float(np.mean(global_vc_list)) if global_vc_list else 0.0,
            "vc_ratio_max": float(max(global_vc_list)) if global_vc_list else 0.0,
            "volume_vph": round(global_vol, 1),
            "capacity_vph": round(global_cap, 1),
            "per_detector": per_detector,
        }

        if _VERBOSE:
            print(f"[v/c Ratio] avg={self.vc_ratio_values['vc_ratio_avg']:.3f}, "
                  f"max(critical)={self.vc_ratio_values['vc_ratio_max']:.3f}, "
                  f"total vol={global_vol:.0f} vph, cap={global_cap:.0f} vph, "
                  f"{len(summary)} intersections, {len(timeseries)} window-records")

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
