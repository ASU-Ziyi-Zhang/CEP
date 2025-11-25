"""
Simple visualization script for exported time-space CSVs.

This script loads a compressed time-space CSV (produced by
`export_time_space_csv`) and produces a time–space scatter plot where
points are colored by vehicle speed. It sets a default serif font
(prefers Times New Roman), configures plot sizes and labels, and
saves the figure to the scenario's `excel_file` folder before
displaying it.

Inputs:
 - A time-space CSV at the path hardcoded near the top of the file
	 (modify `pd.read_csv(...)` to point to other scenarios).

Outputs:
 - A PNG file written to the scenario `excel_file` folder.
 - An interactive matplotlib window (when run in GUI environments).

Usage:
 - Run `python visualize.py` (ensure the CSV path exists).
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import font_manager


preferred_fonts = [
	"Times New Roman",            # Windows/macOS if installed
	"Nimbus Roman No9 L",         # Common on many Linux distros (URW)
	"Liberation Serif",           # Widespread replacement
	"DejaVu Serif",               # Always available in many envs
	"FreeSerif",
	"Times"
]
available_fonts = {f.name for f in font_manager.fontManager.ttflist}
chosen_font = next((f for f in preferred_fonts if f in available_fonts), None)
if chosen_font is None:
	# Fallback to generic serif without breaking execution
	chosen_font = "serif"
	print("Warning: Preferred fonts not found. Falling back to 'serif'. "
		  "To use Times New Roman, install a TNR-compatible font package.")
mpl.rcParams["font.family"] = chosen_font
# Make axis tick labels larger; axis labels slightly smaller than ticks/colorbar as requested
mpl.rcParams["xtick.labelsize"] = 14
mpl.rcParams["ytick.labelsize"] = 14
mpl.rcParams["axes.labelsize"] = 14
mpl.rcParams["axes.titlesize"] = 14

# df = pd.read_csv("sumo_scenarios/i24/excel_file/i24_cav10_timespace.csv.gz")
df = pd.read_csv("sumo_scenarios/onramp/excel_file/onramp_cav10_timespace.csv.gz")
print(df.columns.tolist())
print(df['speed'].describe())
print(df[['time','veh_id','pos','speed']].head())

plt.figure(figsize=(10, 6))
plt.scatter(df["time"], df["pos"], c=df["speed"], cmap="viridis", s=1, alpha=0.7)

# Colorbar with larger tick labels and slightly smaller label text
cbar = plt.colorbar()
cbar.ax.tick_params(labelsize=14)
cbar.set_label("Speed (m/s)", fontsize=16)

# Axis labels slightly smaller; ticks already set larger via rcParams
plt.xlabel("Time (s)", fontsize=16)
plt.ylabel("Position (m)", fontsize=16)
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
# plt.title("Time–Space Diagram with Speed Color (CAV80)", fontsize=14)

plt.tight_layout()
# Save before show to ensure the file is written even in non-interactive runs
# plt.savefig("sumo_scenarios/i24/excel_file/timespace_p10.png", dpi=300, bbox_inches="tight")
plt.savefig("sumo_scenarios/onramp/excel_file/timespace_p10.png", dpi=300, bbox_inches="tight")
plt.show()

