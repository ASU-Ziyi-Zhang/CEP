#######################################
### Jihun Han                       ###
### Tyler Ard                       ###
### Argonne National Laboratory     ###
### Vehicle Mobility Systems        ###
#######################################

import json
import os
import pandas as pd
import matplotlib.pyplot as plt

############
### Settings
############
drct = "east" # "east" or "west" to select direction of Roosevelt corridor to plot

log_folder = "logs/0" # Log folder should be in the format "logs/<timestamp>/<penetration>" where timestamp is optional and the time when the simulation was run and penetration is the CAV penetration rate used in the simulation. For example, "logs/2024-06-01_12-00-00/20" for a simulation run on June 1, 2024 at 12:00 PM with 20% CAV penetration.

### Hardcode Roosevelt corridor data data
if drct == "east":
    # Eastbound
    tlIDList_sel = ['Clinton', 'Canal', 'Delano', 'Clark', 'State', 'Wabash', 'Michigan']
    postls_sel = [410.45, 534.81, 1092.29, 1263.0, 1514.67, 1649.47, 1788.01] # Eastbound
    phase_sel = 4

elif drct == "west":
    # Westbound
    tlIDList_sel = ['Michigan', 'Wabash', 'State', 'Clark', 'Delano', 'Canal', 'Clinton']
    postls_sel = [956.13, 1102.57, 1234.22, 1488.77, 1660.64, 2213.41, 2340.94] # Westbound
    phase_sel = 8

else: 
    raise ValueError("Invalid direction. Please choose 'east' or 'west'.")

#############
### Load Data
#############
# Get file paths
sim_log_file_path = os.path.join(log_folder, "sim_log_data.csv")
veh_log_file_path = os.path.join(log_folder, "veh_log_data.csv")
tl_data_file_path = os.path.join(log_folder, "roosevelt_simple_tl_data.json")

if not (os.path.exists(sim_log_file_path) and os.path.exists(veh_log_file_path) and os.path.exists(tl_data_file_path)):
    raise ValueError("One or more data files are missing. Please check the file paths.")

# Load data
dfsim = pd.read_csv(sim_log_file_path)
dfveh = pd.read_csv(veh_log_file_path)

try:
    with open(tl_data_file_path, "r") as f:
            TLInfo = json.load(f)

except json.JSONDecodeError as e:
    raise ValueError("Error loading TL data:", e)

############
### Plotting
############

# Plot traffic light states and trajectories of human-driven vehicles (HVs) and connected and automated vehicles (CAVs)
fig1, ax1 = plt.subplots()
for i, tlID in enumerate( list(TLInfo.keys()) ):
    indUnqLanes = TLInfo[tlID]["indUnqLanes"]

    tlsdata = dfsim[dfsim["tlID"] == tlID]

    act = indUnqLanes[phase_sel - 1]
    indAct = [i for i, v in enumerate(act) if v == 1]

    RYGStates = tlsdata["RYGStates"].tolist()
    states = [s[indAct[0]] for s in RYGStates]

    itls = [i for i, tlIDs in enumerate(tlIDList_sel) if tlIDs == tlID]
    postl = postls_sel[itls[0]]
    rArr = [postl if s == 'r' else None for s in states]

    ax1.plot(tlsdata["t"], rArr, color="red", linewidth=1)

# Human and CAV trajectories
text_size=14 # Text size for axis labels - adjust as needed
t_min = 350 # Min time to plot - adjust as needed
t_max = 1000 # Max time to plot - adjust as needed

HVstr = "".join([drct, "_", "hdv"])
CAVstr = "".join([drct, "_", "cav"])

HV_ids = [vid for vid in dfveh["vehID"].unique() if HVstr.lower() in vid.lower()]
CAV_ids = [vid for vid in dfveh["vehID"].unique() if CAVstr.lower() in vid.lower()]

if len(HV_ids) == 0 and len(CAV_ids) == 0:
    raise ValueError("No vehicle trajectories found for the specified direction. Please check the vehicle IDs in the data.")

fig2, ax2 = plt.subplots()
fig3, ax3 = plt.subplots()
fig4, ax4 = plt.subplots()

for vid in HV_ids:
    df_v = dfveh[dfveh["vehID"] == vid]
    ax1.plot(df_v["t"], df_v["s"], color="black", linewidth=0.6)
    ax2.plot(df_v["s"], df_v["v"], color="black", linewidth=0.6)
    ax3.plot(df_v["t"], df_v["gap"], color="black", linewidth=0.6)
    ax4.plot(df_v["s"], df_v["a"], color="black", linewidth=0.6)

for vid in CAV_ids:
    df_v = dfveh[dfveh["vehID"] == vid]
    ax1.plot(df_v["t"], df_v["s"], color="blue", linewidth=0.6)
    ax2.plot(df_v["s"], df_v["v"], color="blue", linewidth=0.6)
    ax3.plot(df_v["t"], df_v["gap"], color="blue", linewidth=0.6)
    ax4.plot(df_v["s"], df_v["a"], color="blue", linewidth=0.6)

ax1.set_xlabel("time [s]", size=text_size)
ax1.set_ylabel("position [m]", size=text_size)
ax1.grid(True)
ax1.set_xlim(left=t_min, right=t_max)

ax2.set_xlabel("position [m]", size=text_size)
ax2.set_ylabel("speed [m/s]", size=text_size)
ax2.grid(True)


ax3.set_xlabel("time [s]", size=text_size)
ax3.set_ylabel("gap [m]", size=text_size)
ax3.grid(True)
ax3.set_xlim(left=t_min, right=t_max)

ax4.set_xlabel("position [m]", size=text_size)
ax4.set_ylabel("acceleration [m/s^2]", size=text_size)
ax4.grid(True)


plt.show(block=False)

#########
### Done
#########
input("Press Enter to exit...")