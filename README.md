# Connected & Autonomous Vehicle Controller and Multi-aspect Evaluation

[![Python](https://img.shields.io/badge/python-3.x-blue.svg)](https://www.python.org/downloads/)
[![SUMO](https://img.shields.io/badge/SUMO-microsimulation-orange.svg)](https://www.eclipse.org/sumo/)
[![License](https://img.shields.io/badge/license-See%20LICENSE-green.svg)](LICENSE)

A collection of calibrated SUMO scenarios and connected/autonomous vehicle (CAV) controllers for **microsimulation of mixed traffic**.  
The repository uses the **TraCI API** and optional **compiled controllers** (codegen + C/C++) to directly command CAV longitudinal behavior in simulation.

On top of the controllers and scenarios, the project provides:

- A **unified multi-controller simulation entry point** (`main_controller.py`) that supports PCC, ACC, CACC, IDM, and a growing set of literature-based CAV controllers — all selectable via `--cav-controller`.
- A **multi-aspect analysis pipeline** (`analysis.py`) that evaluates CAV penetration under each controller in terms of safety, throughput and stability, fuel consumption, and interactions with other drivers.
- An interactive, publication-oriented **dashboard** (`dashboard.py`) that visualizes per-controller metrics and exports plot-ready datasets and time–space diagrams.

- **Interactive dashboard**  
> Multi-aspect dashboard for mixed-traffic impacts of CAVs (scenario & CAV penetration selectors):  
> https://asu-ziyi-zhang.github.io/CAV_benchmark_evaluation/onramp_acc_p0.1_report.html


## Overview

`cav_pcc_evaluation` provides a reusable environment for exploring **CAV penetration**, **longitudinal controllers**, and **mixed traffic dynamics** in SUMO.

The workflow is:

1. Run SUMO with any supported CAV controller at different penetration levels using `main_controller.py`.
2. Use `analysis.py` to build a **multi-aspect evaluation** of each experiment:
   - safety, mobility, environmental impact, behavioral indicators, macroscopic wave patterns, and microsimulation quality control.
3. Use `dashboard.py` to **visualize and export** these metrics in a Dash/Plotly web app:
   - interactive tabs (safety, throughput and stability, fuel consumption, and interactions with other drivers),
   - per-controller and per-penetration selectors,
   - CSV/XLSX/JSON exports for offline analysis.

The goal is to make it easy to (i) deploy and swap CAV controllers in SUMO, (ii) run controlled experiments across scenarios, penetration levels, and controller types, and (iii) inspect the resulting traffic and controller performance in a systematic, reproducible way.

---

## Key Features

- **Multi-Controller CAV Simulation via TraCI (`main_controller.py`)**
  - Unified simulation entry point supporting all supported CAV controllers via `--cav-controller`:
  - Select the longitudinal controller with `--cav-controller`:
    | Controller | Category | Dispatch | Description |
    |---|---|---|---|
    | `pcc` | PCC | External (compiled) | Predictive Cruise Controller |
    | `acc` | ACC | SUMO internal | Adaptive Cruise Control |
    | `cacc` | CACC | SUMO internal | Cooperative ACC |
    | `idm` | IDM | SUMO internal | Intelligent Driver Model |
    | `gunter2020` | ACC | SUMO internal | Gunter et al. (2020) — Field-calibrated conservative ACC |
    | `vajedi2016` | ACC | SUMO internal | Vajedi & Azad (2016) — Eco-oriented ACC |
    | `li2018` | CACC | SUMO internal | Li & Wang (2018) — Short-headway CACC |
    | `mosharafian2022` | CACC | SUMO internal | Mosharafian & Velni (2022) — Robust cooperative CACC |
    | `kim2021` | CACC | SUMO internal | Kim et al. (2021) — CACC platoon setting |
    | `sun2024` | MPC | External (PCC) | Sun et al. (2024) — Robust DMPC-style platoon |
    | `wen2022` | MPC | External (PCC) | Wen et al. (2022) — Responsive MPC car-following |
    | `zhang2025` | MPC | External (PCC) | Zhang et al. (2025) — Conservative mixed-traffic MPC |
  - Each controller uses a **dedicated SUMO vType** (e.g., `cav_pcc`, `cav_acc`, `cav_idm`, …) so car-following parameters are fully isolated between runs.
  - Output files are automatically tagged with both penetration rate and controller name (e.g., `fcd_p0.3_acc.xml`, `stats_p0.5_pcc.xml`).

- **Compiled Controller Support (Codegen)**
  - Optional C/C++-compiled controllers with shared libraries (`.so` / `.dll`).
  - `cmake`-based build scripts and wrappers (`cwrapper.py`, `cppwrapper.py`).

- **Multi-Aspect Post-Simulation Analysis**
  - `analysis.py` ingests SUMO outputs (FCD, stats, lane/edge aggregates, detectors/TLS) and builds:
    - **Safety** metrics (TTC, PET, DRAC).
    - **Throughput and Stability** metrics (throughput, delay, speeds, acceleration, queues).
    - **Fuel Consumption** metrics (fuel consumption, fuel efficiency).
    - **Interaction with Other Drivers** metrics (headways, following gaps, lane changes, gap acceptance).
  - Optional **urban-signal** analytics: PAoG, GOR, TTS, spillback detection.

- **Interactive, Publication-Ready Dashboard**
  - `dashboard.py` wraps the analysis results in a Dash/Plotly web app:
    - Tabs for safety, throughput and stability, fuel consumption, and interactions with other drivers.
    - `--cav-controller` flag to select which controller's results to display.
    - Time–space viewer and exporters (CSV) for detailed lane-level visualization.
    - XLSX/CSV/JSON exports of plot-ready datasets and spillback events.

- **Batch Runners**
  - `run_batch.py` — parallel multi-penetration sweep for any supported controller.
  - `run_dashboard_batch.py` — batch metrics-table export across penetration rates and controllers.

- **Cross-Platform Setup**
  - Shell-based installer for Linux (and Git Bash on Windows).
  - Batch installer for Windows Developer Command Prompt.
  - Manual install path for custom Python/SUMO environments.

- **Simulation & Analysis Pipeline**
  - `main_controller.py` for SUMO mixed traffic runs (multi-controller).
  - `analysis.py` for aggregating, caching, and exporting metrics.
  - Scenario-based organization of inputs and outputs for reproducibility.
---

##  Installation

### Prerequisites

- **Python** 3.x
- **SUMO** (installed via this repo or system-wide)
- **Git** (to clone this repository)
- **cmake** (required if you want to compile codegen controllers)
- On Windows: **Visual Studio C/C++ build tools** + **Developer Command Prompt**

---

### Option 1: Linux / macOS (Recommended)

From the repository root (`cav_pcc_evaluation/`):

```bash
# Clone the repository
git clone <repository-url>
cd cav_pcc_evaluation

# Run installer (Python + SUMO + optional controllers)
. install.sh
```

What this does:

- Checks for a valid Python + `pip`.
- Installs SUMO and required Python packages (globally or in your chosen env).
- Optionally compiles and installs CAV controllers, placing shared libraries in `cav_pcc_evaluation/` and copying wrappers to `src/`.

>  You can edit `install.sh` to enable virtual environments, e.g. `USING_VENV=1`.

To run only the controller installers:

```bash
cd codegen
. install.sh
```

---

### Option 2: Windows (Developer Command Prompt / Git Bash)

**Approach A — Git Bash (Unix-like)**

1. Install:
   - Git (with Git Bash)
   - Visual Studio C/C++ tools
   - `cmake`
2. Open **Git Bash**
3. Follow the same steps as Linux:

```bash
git clone <repository-url>
cd cav_pcc_evaluation
. install.sh
```

**Approach B — Windows Developer Command Prompt**

From the **Developer Command Prompt for VS**:

```bat
git clone <repository-url>
cd cav_pcc_evaluation

rem Run Windows installer
install.bat
```

This will:

- Install SUMO dependencies.
- Attempt to compile the CAV controllers using `cmake`.
- Copy the shared libraries to the `cav_pcc_evaluation/` root.
- Copy `cwrapper.py` to `src/`.

To run only the controller installers (without SUMO install):

```bat
cd codegen
install.bat
```

---

### Option 3: Manual Python & Controller Installation

If you prefer manual setup:

```bash
# Python packages
pip install eclipse-sumo
pip install traci
pip install libsumo
```

To build the PCC controller (requires `cmake`):

```bash
cd codegen

# Choose build type
BUILD_TYPE=Release  # Options: Debug, RelWithDebInfo, Release

cd pcc_codegen
mkdir build

cmake -B build -DCMAKE_BUILD_TYPE=$BUILD_TYPE
cmake --build build --config $BUILD_TYPE
cmake --install build --config $BUILD_TYPE

rm -r build

python test.py
```

If the tests pass, copy shared libraries and wrappers into the main repo:

```bash
cp lib/pcc_so.dll ../../  # or .so on Linux
cp _cppwrapper.py ../../src/cppwrapper.py

cd ..
```

---

## Quick Start

### 1. Run Mixed Traffic Simulation

From the `cav_pcc_evaluation/` root, use `main_controller.py` — the unified multi-controller entry point:

```bash
# On-ramp scenario, 30% CAV penetration, PCC controller
python main_controller.py --scenario onramp --penetration 0.3 --cav-controller pcc

# On-ramp scenario, 50% CAV penetration, ACC controller (SUMO internal)
python main_controller.py --scenario onramp --penetration 0.5 --cav-controller acc

# I-24 scenario, 90% CAV penetration, IDM controller, with GUI
python main_controller.py --scenario i24 --penetration 0.9 --cav-controller idm --gui
```

Check available options:

```bash
python main_controller.py -h
```

Typical options include:

- `--scenario` — scenario to run (`onramp`, `i24`).
- `--penetration` — CAV penetration rate in `[0, 1]`.
- `--cav-controller` — longitudinal controller (`pcc`, `acc`, `cacc`, `idm`, or any literature variant: `gunter2020`, `vajedi2016`, `li2018`, `mosharafian2022`, `kim2021`, `sun2024`, `wen2022`, `zhang2025`).
- `--gui` — launch SUMO GUI (requires standard `traci`; not compatible with libsumo).
- `--seed` — random seed for reproducibility.

>  `main_controller.py` runs SUMO using a TraCI control loop, calling the selected CAV controller each step and tagging all outputs with both penetration rate and controller name.

---

### 2. Batch Run (Multi-Penetration Sweep)

```bash
# Sweep penetration 0.0–0.9 for the ACC controller on the onramp scenario (5 parallel workers)
python run_batch.py --scenario onramp --controller acc --penetrations 0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9 --workers 5

# PCC controller, I-24 scenario
python run_batch.py --scenario i24 --controller pcc --penetrations 0.0,0.3,0.5,0.7,0.9
```

Check options with `python run_batch.py -h`.

---

### 3. Run Simulation Analysis

After simulations are complete:

```bash
python analysis.py
python analysis.py -h   # View options
```

You can configure:

- Which output file/folder to read.
- Whether to run in freeway or urban mode.
- Where to write processed metrics and exports.

---

### 4. Launch the Dashboard

After `analysis.py` has created metrics and (optionally) caches:

```bash
# On-ramp, ACC controller, 10% CAV penetration, excluding ramp lanes
python dashboard.py --scenario onramp --urban no --exclude-lane ramp_0 --exclude-lane E2_0 --cav-controller acc --p 0.1

# On-ramp, PCC controller, 50% CAV penetration
python dashboard.py --scenario onramp --p 0.5 --cav-controller pcc

# I-24, IDM controller
python dashboard.py --scenario i24 --urban no \
    --exclude-lane E2_0 --exclude-lane E4_0 --exclude-lane E6_0 --exclude-lane E1_0 \
    --cav-controller idm --p 0.3
```

Then open your browser at:

- http://localhost:8050

Useful options (see `dashboard.py -h`):

- `--cav-controller` to select which controller's outputs to display.
- `--p` to select the CAV penetration rate.
- `--file` to open a specific FCD file.
- `--exclude-lane`, `--exclude-lane-prefix`, `--lane-pos-window` to control which samples are included.
- `--urban` to control Urban Signals integration (`auto` / `yes` / `no`).

---

### 5. Standalone Time–Space Plot

For a quick publication-quality time–space diagram from an exported CSV (without launching the full dashboard):

```bash
python visualize.py
```

Edit the CSV path near the top of [visualize.py](visualize.py) to point to the desired scenario's export file.

---

### 6. Batch Dashboard Export

To export metrics tables for all penetration rates in one shot:

```bash
# Export metrics for ACC controller on onramp across all penetration rates
python run_dashboard_batch.py --scenario onramp --controller acc --penetrations 0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9

# I-24, PCC controller
python run_dashboard_batch.py --scenario i24 --controller pcc
```

---

## 📁 Project Structure

A high-level view of the repository (simplified):

```text
cav_pcc_evaluation/
├── src/                       # Python source modules
│   ├── agents.py              # CAV controller bindings (PCC, CAV, EXT)
│   ├── settings.py            # Global constants and thresholds
│   ├── sensing.py             # Vehicle state and V2V communication
│   ├── cav_cwrapper.py        # C wrapper for CAV controller shared library
│   ├── cppwrapper.py          # C++ wrapper for PCC shared library
│   └── ...
│
├── codegen/                   # Compiled controller build scripts & sources
│   ├── pcc_codegen/           # PCC controller (C++, cmake)
│   ├── cavtrl_codegen/        # CAV controller (C, cmake)
│   ├── install.sh
│   └── install.bat
│
├── parsers/                   # CLI argument parsers
│   ├── sumo.py                # SUMO/TraCI argument registration
│   └── ...
│
├── scripts/                   # Utility helpers
│   ├── utils_data_read.py     # Route-file flow updater
│   ├── utils_vis.py           # Visualization helpers
│   └── utils_macro.py         # Macroscopic traffic utilities
│
├── sumo_scenarios/            # SUMO networks, routes, and configs
│   ├── onramp/                # On-ramp merge scenario
│   │   ├── onramp.sumocfg
│   │   ├── onramp_template.rou.xml
│   │   └── output/            # Simulation outputs (fcd, stats, edge/lane data)
│   ├── i24/                   # I-24 corridor scenario
│   │   └── output/
│   └── config.json
│
├── docs/                      # Pre-built HTML reports and metrics JSON exports
│
├── pcc_so.dll                 # Compiled PCC shared library (Windows; .so on Linux)
├── cav_so.dll                 # Compiled CAV shared library (Windows; .so on Linux)
│
├── main_controller.py         # Main SUMO simulation entry point (multi-controller)
├── analysis.py                # Post-processing & export script
├── dashboard.py               # Interactive Dash/Plotly visualization
├── visualize.py               # Standalone time-space diagram plotter
├── run_batch.py               # Batch runner: parallel multi-penetration sweep
├── run_dashboard_batch.py     # Batch runner: export metrics tables for all penetrations
├── install.sh                 # Linux / Git Bash installer
├── install.bat                # Windows installer
├── README.md                  # Project documentation (this file)
└── LICENSE                    # License file
```

> Note: Exact structure may vary slightly by branch or use case.

---

## Scenarios & Controllers

### Scenarios

- **Freeway on-ramp** (`onramp`)
- **I-24 corridor** (`i24`)
- Additional scenarios can be added under `sumo_scenarios/<scenario_name>/` with their own:
  - Network (`.net.xml`)
  - Routes template (`.rou.xml` with `_template` suffix)
  - Configuration and detector definitions.

### Controllers

`main_controller.py` supports the following longitudinal CAV controllers via `--cav-controller`:

#### PCC — Predictive Cruise Controller (external, compiled)
- Compiled via `codegen/pcc_codegen` and invoked through a shared library + Python wrapper.
- Disables SUMO's internal longitudinal model and closes the loop externally each step.
- Uses `cav_pcc` vType in SUMO.

#### SUMO-internal controllers
These controllers rely on SUMO's built-in car-following models.  Only one-time appearance and flag setup is performed by Python; SUMO drives the physics each step.

| `--cav-controller` | SUMO vType | Model |
|---|---|---|
| `acc` | `cav_acc` | Adaptive Cruise Control |
| `cacc` | `cav_cacc` | Cooperative ACC |
| `idm` | `cav_idm` | Intelligent Driver Model |

#### Literature-based configurations
Calibrated parameter sets from published CAV control studies, each with an isolated SUMO vType.  
Controllers are grouped by underlying control paradigm. **SUMO-internal** variants let SUMO's car-following model drive the vehicle using literature-recommended parameters. **External (PCC)** variants dispatch through the compiled PCC C++ library each step — the vType parameters (τ, minGap, accel, decel) tune the vehicle's initial SUMO state only.

| `--cav-controller` | SUMO vType | Category | Dispatch | Reference | Description |
|---|---|---|---|---|---|
| `gunter2020` | `cav_gunter2020` | ACC | SUMO internal | Gunter et al. (2020) | Field-calibrated conservative ACC: τ=1.7, minGap=8.0, accel=1.5 |
| `vajedi2016` | `cav_vajedi2016` | ACC | SUMO internal | Vajedi & Azad (2016) | Eco-oriented ACC: τ=1.6, minGap=10.0, decel=3.0 |
| `li2018` | `cav_li2018` | CACC | SUMO internal | Li & Wang (2018) | Short-headway CACC: τ=0.7, minGap=2.5, actionStep=0.1 |
| `mosharafian2022` | `cav_mosharafian2022` | CACC | SUMO internal | Mosharafian & Velni (2022) | Short-headway robust cooperative CACC: τ=0.7, minGap=2.0, accel=3.0, decel=5.0 |
| `kim2021` | `cav_kim2021` | CACC | SUMO internal | Kim et al. (2021) | CACC platoon setting: τ=1.0, minGap=7.5, accel=2.5, decel=6.0 |
| `sun2024` | `cav_sun2024` | MPC | External (PCC) | Sun et al. (2024) | Robust DMPC-style platoon: τ=1.0, minGap=1.5, accel=1.5, decel=2.0 |
| `wen2022` | `cav_wen2022` | MPC | External (PCC) | Wen et al. (2022) | Responsive MPC car-following: τ=1.2, minGap=5.0, accel=3.0, decel=5.0 |
| `zhang2025` | `cav_zhang2025` | MPC | External (PCC) | Zhang et al. (2025) | Conservative mixed-traffic MPC: τ=1.2, minGap=1.5, accel=1.01, decel=2.26 |

### Output file naming

All simulation outputs are tagged with `p<penetration>_<controller>` so results from different runs do not overwrite each other:

```
output/
  fcd_p0.3_pcc.xml
  fcd_p0.3_acc.xml
  stats_p0.5_idm.xml
  by_lane_p0.7_gunter2020.xml
  ...
```

### TraCI Loop Example (simplified)

This pattern is used inside `main_controller.py` for PCC dispatch and can be adapted for custom controllers:

```python
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()

    sim_time = traci.simulation.getTime()
    vehicle_ids = traci.vehicle.getIDList()

    # Only command vehicles of the active controller vType
    cav_ids = (vid for vid in vehicle_ids if traci.vehicle.getTypeID(vid) == cav_type_id)
    for ego_id in cav_ids:
        ego_speed = traci.vehicle.getSpeed(ego_id)
        ego_accel = traci.vehicle.getAcceleration(ego_id)
        ego_distance = traci.vehicle.getDistance(ego_id)

        leader_info = traci.vehicle.getLeader(ego_id)
        if leader_info is not None and leader_info[0] != "":
            lead_id = leader_info[0]
            lead_rel_distance = leader_info[1]
            lead_speed = traci.vehicle.getSpeed(lead_id)
            lead_accel = traci.vehicle.getAcceleration(lead_id)
        else:
            lead_rel_distance = 2000.0
            lead_speed = 0.0
            lead_accel = 0.0

        desired_accel = getCommand(
            sim_time, ego_accel, ego_speed, ego_distance,
            s_max, v_max, lead_accel, lead_speed, lead_rel_distance
        )

        traci.vehicle.setAcceleration(ego_id, desired_accel, acc_duration=2.0)
```

For SUMO-internal controllers (`acc`, `cacc`, `idm`, …), the loop still runs but skips the external dispatch block — SUMO handles longitudinal control natively based on the vType's car-following parameters.

---

##  Analysis: Multi-Aspect Evaluation

`analysis.py` is the main **post-simulation evaluation engine**.  
It takes raw SUMO outputs (FCD, statistics, edge/lane aggregates, and optionally detector/TLS data) and builds a **multi-aspect evaluation** of PCC and other controllers under different CAV penetration levels.

### Inputs

Typical input files from SUMO include:

- **FCD (floating car data)**, e.g. `fcd.xml`  
  Per-timestep, per-vehicle states with leader information (gap, speed, acceleration).

- **Simulation statistics**, e.g. `stats.xml`  
  Real-time factor, number of loaded/inserted/running/teleported vehicles, emergency braking, etc.

- **Edge / lane aggregates**, e.g. `by_edge.xml`, `by_lane.xml`  
  For macroscopic and quality-control measures.

- **(Optional: Urban mode)** Detector and signal data  
  TLS time series, static `tlLogic`, detector outputs, and lane mapping (`add.xml`).

### Metric families

The script organizes metrics into several dimensions:

1. **Safety**
   - TTC (Time-to-Collision) distribution.
   - PET (Post-Encroachment Time) distribution.
   - DRAC (Deceleration Rate to Avoid a Crash).

2. **Throughput and Stability**
   - Throughput vs. penetration rate.
   - Total delay and average speed.
   - Lane-/edge-specific travel times and level of service.
   - Queue length and stop time at bottlenecks / intersections.

3. **Fuel Consumption**
   - Emissions (per vehicle / scenario).
   - Fuel consumption rate based on a physics-inspired traction-power model:
     traction power + auxiliaries, idle floor, BSFC-based fuel-rate conversion (g/s), with stable behavior at low speeds.

4. **Interaction with Other Drivers**
   - Overtaking / lane-change rate and duration.
   - HDV/CAV following gap and headway distributions.
   - Gap acceptance at merges / lane changes.
   - Time headway distribution (HDV and CAV).
   - (Planned) reaction time and other microscopic behavior indicators.

5. **Macroscopic characteristics**
   - Shockwave propagation and stop-and-go wave intensity along the corridor.
   - Aggregate speed-flow-density patterns.

### Urban signal analytics (`urban_mode`)

When `urban_mode` is enabled, `analysis.py` adds a full **signalized-corridor** pipeline:

- Detector → lane → TLS mapping from detector outputs, `tlLogic`, and lane mapping files.
- Urban metrics:
  - **PAoG** (Percent of Arrivals on Green).
  - **GOR** (Green Occupancy Ratio), both overall and by movement type.
  - Intersection approach delay and queue spillback detection.
  - **Time-To-Service (TTS)** for queued vehicles.

This allows PCC impacts to be evaluated consistently in both freeway and urban signalized environments.

### Caching and robustness

To support fast iteration and stable metrics, `analysis.py`:

- Uses `load_or_build_metrics(file_dir, urban_mode)` as a **cache loader**:
  - Computes a signature of all relevant inputs (e.g., mtime + size).
  - Reuses cached results (e.g., `metrics_p0.1t.pkl`) when inputs have not changed.
  - Automatically rebuilds metrics when inputs differ.

- Applies **gating, limits, and sanitization**:
  - Spatial gates (`POS_MIN_ANALYSIS`, `TT_USE_POS_GATE`, `ENERGY_USE_POS_GATE`).
  - Physical bounds on speed/acceleration (`V_MAX`, `ACC_MIN`, `ACC_MAX`).
  - Visualization bounds for histograms (`SPEED_MIN_HIST`, `SPEED_MAX_HIST`, `ACC_MAX_HIST`).
  - Cut-offs for TTC/PET/headway/space-gap to remove numerical artifacts.

In short, `analysis.py` turns raw SUMO logs into a consistent, publication-ready **multi-aspect evaluation** of any supported CAV controller across different penetration levels.

---

## Dashboard

`dashboard.py` is an interactive **front-end** on top of `analysis.py`.  
It reads the processed metrics and exports from the analysis pipeline and exposes them through a Dash/Plotly web app.

### What the dashboard provides

- Multiple analysis tabs:
  - **Safety** – TTC/PET/DRAC distributions and related indicators.
  - **Throughput and Stability** – throughput, delay, speeds, accelerations, travel times.
  - **Interaction with Other Drivers** – space gap, lane-change frequency, gap acceptance, time headways.
  - **Fuel Consumption** – fuel efficiency, per-trip fuel consumption, avergage fuel consumption.
  - **Time–Space** (disable) – time–space diagrams with CAV penetration overlays and export tools.
  - **Urban Signals** (optional) – PAoG, GOR, approach delay, TTS, and spillback charts.

- Interactive filters:
  - Scenario selection and CAV penetration.
  - **Controller selector** (`--cav-controller`) to switch between PCC, ACC, IDM, and other controllers.
  - Direction, lanes, lane prefixes, and position windows.
  - Warm-up trimming and queue-clearing thresholds (aligned with `analysis.py`).


### Relationship to `analysis.py`

The dashboard is intentionally **thin** on heavy computation:

- It calls `load_or_build_metrics(...)` from `analysis.py` to either:
  - Reuse cached metrics when SUMO outputs are unchanged, or
  - Rebuild metrics when inputs changed.
- This keeps the dashboard responsive even for large networks and long simulations.
- All thresholds, gates, and definitions are shared with the analysis code, so the plots match the numerical results used in papers.

### Time–Space viewer and exporters

The **Time–Space** tab is built as a combined viewer and exporter:

- Hooks into an internal visualizer to render time–space diagrams.
- Provides a robust CSV exporter (e.g., `export_time_space_csv(...)`) with options for:
  - Warm-up removal and position gating.
  - Queue-clearing rules (`CLEAR_V`, `CLEAR_M`).
  - Jump detection and temporal/vehicle stride.
  - Optional fields like speed, plus gzip compression for large files.

These exports are intended to support downstream analysis or figure generation outside the dashboard (e.g., in notebooks).

### Reproducible figure exports

To support paper-quality figures and post-processing, the dashboard also exposes export helpers, such as:

- `export_dashboard_datasets(...)`:
  - Writes compact, plot-ready tables to **XLSX** (multi-sheet) or CSV.
  - Includes time-series data and histogram bins, so plots can be recreated offline.

- `export_spillback(...)`:
  - Saves detected spillback events to **CSV/JSON**.
  - Prints a short console summary for quick checks.

### Usage recap

Typical usage pattern:

1. Run SUMO with the desired controller to produce raw outputs (`fcd_p<x>_<controller>.xml`, `stats_p<x>_<controller>.xml`, etc.).
2. Run `analysis.py` (optionally in `urban_mode`) to build metrics and caches.
3. Launch the dashboard and point it to the scenario folder:

```bash
# Freeway onramp, ACC controller
python dashboard.py --scenario_folder sumo_scenarios \
    --scenario onramp --urban no \
    --exclude-lane ramp_0 --exclude-lane E2_0 \
    --cav-controller acc --p 0.1

# Freeway onramp, PCC controller
python dashboard.py --scenario_folder sumo_scenarios \
    --scenario onramp --urban no \
    --exclude-lane ramp_0 --exclude-lane E2_0 \
    --cav-controller pcc --p 0.3

# I-24, IDM controller
python dashboard.py --scenario i24 --urban no \
    --exclude-lane E2_0 --exclude-lane E4_0 \
    --exclude-lane E6_0 --exclude-lane E1_0 \
    --cav-controller idm --p 0.5
```

The dashboard then loads (or rebuilds) the metrics via `analysis.py` and exposes them in an interactive, publication-style web UI.

---

## Debugging / FAQ

### 1. Shared library `<cav_controller>` not found

> _“The `<cav_controller>` shared library file cannot be found, but I can see it in my directory.”_

- Check that the installation path in the **PCC** class inside `src/agents.py` points to the correct shared library location.
- If the library was built on another machine or OS, you may need to **rebuild** it:
  - Follow the compiled controller install steps in [Installation](#installation).
  - Ensure `cmake` and your compiler toolchain are correctly installed.

### 2. Dashboard shows empty plots or fails to load

- Confirm that `analysis.py` has been run and that:
  - `<scenario>/excel_file` (or equivalent exports) exist.
  - The expected CSV/Excel files are present.
- Double-check that `--scenario`, `--scenario_folder`, and `--cav-controller` match the controller you used during simulation.
- Output files are tagged by controller name (e.g., `fcd_p0.3_acc.xml`); make sure the dashboard `--cav-controller` flag matches the simulation run.

### 3. SUMO / TraCI import errors

- Make sure `eclipse-sumo`, `traci`, and `libsumo` are installed in the active Python environment.
- On Windows, verify that SUMO is on your `PATH` or that `SUMO_HOME` is set correctly.

---

##  Contributing

Contributions are welcome! Suggested improvements include:

- New microsimulation scenarios (e.g., signalized corridors).
- Additional CAV controllers (eco-approach, merging control, etc.).
- Quality-control scripts and new dashboard views.

Typical workflow:

1. Fork the repository.
2. Create a feature branch  
   `git checkout -b feature/amazing-feature`
3. Commit your changes  
   `git commit -m "Add amazing feature"`
4. Push to your fork  
   `git push origin feature/amazing-feature`
5. Open a Pull Request.

Please:

- Follow PEP 8 for Python style where practical.
- Add docstrings to new functions/classes.
- Include tests or small examples when adding major features.
- Update the README / docs for user-facing changes.

---

##  License

See the [LICENSE](LICENSE) file for details.

---

##  Contact

For questions, bug reports, or feature requests:

- Open an issue on the project’s GitHub repository.
- Or contact the maintainer / research team listed in the repo metadata (fill in as appropriate for your deployment).

---

##  Acknowledgments

This repository builds on work in:

- SUMO (Simulation of Urban MObility) and the TraCI API.
- Prior CAV controller and code generation examples by the project contributors.
- Coursework and research on mixed traffic microsimulation, PCC, and CAV controller benchmarking.

---

**Last Updated**: March 2026  
**Status**: Research / Development Ready













