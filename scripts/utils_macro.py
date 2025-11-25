import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt
import pandas as pd
import csv
import xml.etree.ElementTree as ET
import pickle
from matplotlib.ticker import FuncFormatter, MultipleLocator
import datetime
from scipy.interpolate import interp1d
from multiprocessing import Pool
import time
import os

def compute_macro(trajectory_file, dx, dt, start_time, end_time, start_pos, end_pos, save=False, plot=True):
    """
    Compute macroscopic flow (Q), density (Rho) or speed (V) from trajectory data.
    An implementation of Edie's method
    Compute mean speed, density and flow from trajectory data using Edie's definition
    flow Q = TTD/(dx dt)
    density Rho = TTT/(dx dt)
    speed = Q/Rho, or can be computed directory from data

    Parameters:
    ----------
    
    trajectory_file : string
        Path for NGSIM-like trajectory data file in .csv
        Sampling frequency should be at least 10Hz for accuracy
        If too coarse, use compute_macro_generalized() instead
    dx : float
        Spatial discretization in meter.
    dt : float
        Temporal discretization in second.
    start_time : float
        starting time in second
    end_time : float
        ending time in second
    start_pos : float
        starting position in meter
    end_pos : float
        ending position in meter
    save : bool
        save as calibration_result/macro_{trajectory_file_name}.pkl if True
    plot : bool
        run plot_macro() if True

    Returns: macro_data : dict
        macro_data = {
            "speed": np.array(),
            "flow": np.array(),
            "density": np.array(),
         }
    """
    # find the spatial and temporal ranges
    start_x = 999999
    end_x = -9999999
    t1 = 999999
    t2 = -999999
    
    first_line = True

    # initialize
    prev_vehicle_id = None
    time_range = end_time - start_time
    space_range = end_pos - start_pos
    TTT_matrix = np.zeros((int(time_range / dt), int( space_range / dx)))
    TTD_matrix = np.zeros((int(time_range / dt), int( space_range / dx)))

    # Read the data file line by line
    first_line = True
    with open(trajectory_file, mode='r') as input_file:
        for line in input_file:
            # Split the line into columns
            columns = line.strip().split()
            if len(columns)==1:
                columns = columns[0].split(',')
            vehicle_id = columns[0] # int(columns[0])

            # if first_line and not vehicle_id.isalpha(): # has all letters, then the first line is a header, skip it
            if first_line and all(isinstance(item, str) for item in columns):
                first_line = False
                # print("skip header")
                continue

            # extract current info
            if prev_vehicle_id is None:
                traj = defaultdict(list)

            elif vehicle_id != prev_vehicle_id: # start a new dict
                # write previous traj to matrics
                data_index = pd.DataFrame.from_dict(traj)
                data_index['space_index'] = (data_index['p'] // dx)
                data_index['time_index'] = (data_index['timestamps'] // dt)
                grouped = data_index.groupby(['space_index', 'time_index'])

                for (space, time), group_df in grouped:
                    if (time >= 0 and time < (int(time_range / dt)) and space >= 0 and space < (int(space_range / dx))):
                        TTD_matrix[int(time)][int(space)] += (group_df.p.max() - group_df.p.min()) # meter
                        TTT_matrix[int(time)][int(space)] += (group_df.timestamps.max() - group_df.timestamps.min()) #sec
                # break
                traj = defaultdict(list)

            time = float(columns[1]) #* 0.1#* 0.1
            foll_v_val = float(columns[4])
            foll_p_val = float(columns[3])

            # foll_a_val = float(columns[5])
            traj["timestamps"].append(time)
            traj["v"].append(foll_v_val)
            traj["p"].append(foll_p_val)

            prev_vehicle_id = vehicle_id
            

    Q = TTD_matrix/(dx*dt) # veh/sec all lanes
    Rho = TTT_matrix/(dx*dt) # veh/m all lanes
    macro_data = {
        "flow": Q,
        "density": Rho,
        "speed": Q/Rho,
    }
    if save:
        trajectory_file_name = trajectory_file.split(".")[0]
        with open(f'calibration_result/macro_{trajectory_file_name}.pkl', 'wb') as f:  # open a text file
            pickle.dump(macro_data, f) # serialize the list
        print(f'macro_{trajectory_file_name}.pkl file saved.')
    # Plotting
    if plot:
        plot_macro(macro_data, dx, dt)
    return macro_data

def compute_macro_generalized(trajectory_file, dx, dt, start_time, end_time, start_pos, end_pos, save=False, plot=True):
    """
    Compute macroscopic flow (Q), density (Rho) or speed (V) from trajectory data, with a data sampling step.
    An implementation of Edie's method
    Compute mean speed, density and flow from trajectory data using Edie's definition
    flow Q = TTD/(dx dt)
    density Rho = TTT/(dx dt)
    speed = Q/Rho, or can be computed directory from data

    Parameters:
    ----------
    
    trajectory_file : string
        Path for NGSIM-like trajectory data file in .csv
        Data is sampled to 10Hz to improve accuracy
    dx : float
        Spatial discretization in meter.
    dt : float
        Temporal discretization in second.
    start_time : float
        starting time in second
    end_time : float
        ending time in second
    start_pos : float
        starting position in meter
    end_pos : float
        ending position in meter
    save : bool
        save as calibration_result/macro_{trajectory_file_name}.pkl if True
    plot : bool
        run plot_macro() if True

    Returns: macro_data : dict
        macro_data = {
            "speed": np.array(),
            "flow": np.array(),
            "density": np.array(),
         }
    """
    # find the spatial and temporal ranges
    new_timestep = 0.1

    # initialize
    prev_vehicle_id = None
    time_range = end_time - start_time
    space_range = end_pos - start_pos
    TTT_matrix = np.zeros((int(time_range / dt), int( space_range / dx)))
    TTD_matrix = np.zeros((int(time_range / dt), int( space_range / dx)))
    V_matrix = np.zeros((int(time_range / dt), int(space_range / dx)))  # Sum of velocities
    count_matrix = np.zeros((int(time_range / dt), int(space_range / dx)))  # Count of velocities


    # Read the data file line by line
    first_line = True
    with open(trajectory_file, mode='r') as input_file:
        for line in input_file:
            # Split the line into columns
            columns = line.strip().split()
            if len(columns)==1:
                columns = columns[0].split(',')
            vehicle_id = columns[0] # int(columns[0])

            # if first_line and not vehicle_id.isalpha(): # has all letters, then the first line is a header, skip it
            if first_line and all(isinstance(item, str) for item in columns):
                first_line = False
                # print("skip header")
                continue

            # extract current info
            if prev_vehicle_id is None:
                traj = defaultdict(list)

            elif vehicle_id != prev_vehicle_id: # start a new dict
                # write previous traj to matrics
                # interpolate traj at higher-resolution (dt=0.1, so that Edie's method has little error)
                new_timestamps = np.arange(traj["timestamps"][0], traj["timestamps"][-1] + new_timestep, new_timestep)
                # Interpolate positions (p) and velocities (v) over the new time steps
                interp_p = interp1d(traj["timestamps"], traj["p"], kind='linear', bounds_error=False, fill_value="extrapolate")
                interp_v = interp1d(traj["timestamps"], traj["v"], kind='linear', bounds_error=False, fill_value="extrapolate")

                new_positions = interp_p(new_timestamps)
                new_velocities = interp_v(new_timestamps)

                # Update the dictionary with the interpolated values
                traj["timestamps"] = new_timestamps.tolist()
                traj["p"] = new_positions.tolist()
                traj["v"] = new_velocities.tolist()

                data_index = pd.DataFrame.from_dict(traj)
                data_index['space_index'] = (data_index['p'] // dx)
                data_index['time_index'] = (data_index['timestamps'] // dt)
                grouped = data_index.groupby(['space_index', 'time_index'])

                for (space, time), group_df in grouped:
                    if (time >= 0 and time < (int(time_range / dt)) and space >= 0 and space < (int(space_range / dx))):
                        
                        TTD_matrix[int(time)][int(space)] += (group_df.p.max() - group_df.p.min()) # meter
                        TTT_matrix[int(time)][int(space)] += (group_df.timestamps.max() - group_df.timestamps.min()) #sec
                        V_matrix[int(time)][int(space)] += group_df.v.sum()
                        count_matrix[int(time)][int(space)] += len(group_df.v)

                # break
                traj = defaultdict(list)

            time = float(columns[1]) #* 0.1#* 0.1
            foll_v_val = float(columns[4])
            foll_p_val = float(columns[3])

            # foll_a_val = float(columns[5])
            traj["timestamps"].append(time)
            traj["v"].append(foll_v_val)
            traj["p"].append(foll_p_val)

            prev_vehicle_id = vehicle_id
            
    # Calculate average velocity in each cell, avoiding division by zero
    with np.errstate(divide='ignore', invalid='ignore'):
        V_matrix = np.divide(V_matrix, count_matrix, out=np.zeros_like(V_matrix), where=count_matrix != 0)

    Q = TTD_matrix/(dx*dt) # veh/sec all lanes
    Rho = TTT_matrix/(dx*dt) # veh/m all lanes
    macro_data = {
        "flow": Q,
        "density": Rho,
        "speed": V_matrix, # m/s
    }
    if save:
        trajectory_file_name = trajectory_file.split(".")[0]
        with open(f'calibration_result/macro_{trajectory_file_name}.pkl', 'wb') as f:  # open a text file
            pickle.dump(macro_data, f) # serialize the list
        print(f'macro_{trajectory_file_name}.pkl file saved.')
    # Plotting
    if plot:
        plot_macro(macro_data, dx, dt)
    return macro_data




def process_trajectory(traj, dx, dt, time_range, space_range):
    """
    TODO: used for parallel compute macro, to be implemented
    Process a single trajectory to update TTT, TTD, V_matrix, and count_matrix.
    """
    new_timestep = 0.1
    new_timestamps = np.arange(traj["timestamps"][0], traj["timestamps"][-1] + new_timestep, new_timestep)
    interp_p = interp1d(traj["timestamps"], traj["p"], kind='linear', bounds_error=False, fill_value="extrapolate")
    interp_v = interp1d(traj["timestamps"], traj["v"], kind='linear', bounds_error=False, fill_value="extrapolate")
    new_positions = interp_p(new_timestamps)
    new_velocities = interp_v(new_timestamps)

    data_index = pd.DataFrame({
        'timestamps': new_timestamps,
        'p': new_positions,
        'v': new_velocities
    })
    data_index['space_index'] = (data_index['p'] // dx).astype(int)
    data_index['time_index'] = (data_index['timestamps'] // dt).astype(int)

    TTT_local = np.zeros((int(time_range / dt), int(space_range / dx)))
    TTD_local = np.zeros((int(time_range / dt), int(space_range / dx)))
    V_local = np.zeros((int(time_range / dt), int(space_range / dx)))
    count_local = np.zeros((int(time_range / dt), int(space_range / dx)))

    grouped = data_index.groupby(['space_index', 'time_index'])
    for (space, time), group_df in grouped:
        if 0 <= time < (int(time_range / dt)) and 0 <= space < (int(space_range / dx)):
            TTD_local[time, space] += (group_df.p.max() - group_df.p.min())  # meter
            TTT_local[time, space] += (group_df.timestamps.max() - group_df.timestamps.min())  # seconds
            V_local[time, space] += group_df.v.sum()
            count_local[time, space] += len(group_df.v)

    return TTT_local, TTD_local, V_local, count_local


def compute_macro_parallel(trajectory_file, dx, dt, start_time, end_time, start_pos, end_pos, save=False, plot=True):
    """
    TODO: to be implemented
    Parallelized computation of mean speed, density, and flow using Edie's generalized formulation.
    The current version is not memory efficient
    """
    time_range = end_time - start_time
    space_range = end_pos - start_pos
    TTT_matrix = np.zeros((int(time_range / dt), int(space_range / dx)))
    TTD_matrix = np.zeros((int(time_range / dt), int(space_range / dx)))
    V_matrix = np.zeros((int(time_range / dt), int(space_range / dx)))
    count_matrix = np.zeros((int(time_range / dt), int(space_range / dx)))

    # Reading trajectory data and dividing into trajectories
    trajectories = []
    traj = defaultdict(list)
    prev_vehicle_id = None
    first_line = True

    with open(trajectory_file, mode='r') as input_file:
        for line in input_file:
            columns = line.strip().split()
            if len(columns) == 1:
                columns = columns[0].split(',')
            vehicle_id = columns[0]

            # if first_line and not vehicle_id.isalpha(): # has all letters, then the first line is a header, skip it
            if first_line and all(isinstance(item, str) for item in columns):
                first_line = False
                # print("skip header")
                continue

            if prev_vehicle_id and vehicle_id != prev_vehicle_id:
                trajectories.append(traj)
                traj = defaultdict(list)
            
            time = float(columns[1])
            foll_v_val = float(columns[4])
            foll_p_val = float(columns[3])
            traj["timestamps"].append(time)
            traj["v"].append(foll_v_val)
            traj["p"].append(foll_p_val)
            prev_vehicle_id = vehicle_id

        if traj:  # Add the last trajectory
            trajectories.append(traj)

    # Parallel processing of trajectories
    pool = Pool()
    results = pool.starmap(process_trajectory, [(traj, dx, dt, time_range, space_range) for traj in trajectories])
    pool.close()
    pool.join()

    # Combine results
    for TTT_local, TTD_local, V_local, count_local in results:
        TTT_matrix += TTT_local
        TTD_matrix += TTD_local
        V_matrix += V_local
        count_matrix += count_local

    # Calculate average velocity in each cell
    with np.errstate(divide='ignore', invalid='ignore'):
        V_matrix = np.divide(V_matrix, count_matrix, out=np.zeros_like(V_matrix), where=count_matrix != 0)

    Q = TTD_matrix / (dx * dt)  # veh/sec all lanes
    Rho = TTT_matrix / (dx * dt)  # veh/m all lanes
    macro_data = {"flow": Q, "density": Rho, "speed": V_matrix}  # m/s

    if save:
        trajectory_file_name = os.path.splitext(trajectory_file)[0]
        with open(f'macro_{trajectory_file_name}.pkl', 'wb') as f:
            pickle.dump(macro_data, f)
        print(f'macro_{trajectory_file_name}.pkl file saved.')

    if plot:
        plot_macro(macro_data, dx, dt)

    return macro_data



def gen_VT(v_matrix, t0, x0, dx=10, dt=10):
    """
    Generalize virtual trajectory from macroscopic speed field
    Credit to Junyi Ji, a different implementation

    Parameters:
    ----------
    
    v_matrix : np.array()
        speed in mph 
    t0 : float
        initial time in second
    x0 : float
        starting position in meter
    dx : float
        spatial discretization of macro_data in meter
    dt : float
        temporal discretization of macro_data in sec

    Returns: t_arr, x_arr
    t_arr: np.array
        Time-series of virtual trajecotry timesteps
    x_arr: np.array
        Time-series of virtual trajecotry positions
    """

    # initialize
    t_arr = [t0] # store states
    x_arr = [x0]
    t = t0 # keep track of current state
    x = x0
    num_time_steps, num_space_points = v_matrix.shape
    t_total = dt*num_time_steps
    x_total = dx*num_space_points
    time_idx = int(t//dt)
    space_idx = int(x//dx)

    while x<x_total and t<t_total:
        # time.sleep(0.5)
        # print(time_idx, space_idx, t,x)
        v = v_matrix[time_idx][space_idx] #33.7
        rem_time = round(dt - (t % dt), 3) # remaining time in the current grid
        rem_space = round(dx - (x % dx), 3)
        if rem_time < 1e-3: rem_time = dt # for numerical issue
        if rem_space < 1e-3: rem_space = dx
        
        time_to_reach_next_space = rem_space / v
        # print("* ",rem_time, rem_space)

        if time_to_reach_next_space <= rem_time:
            # If the VT will hit the spatial boundary first
            space_idx += 1
            t += time_to_reach_next_space
            # x += rem_space
            x = space_idx * dx
            
        else:
            # If the VT will hit the temporal boundary first
            time_idx += 1
            x += v * rem_time
            # t += rem_time
            t = time_idx * dt
            
        t_arr.append(t)
        x_arr.append(x)

    return t_arr, x_arr


def plot_macro_sim(macro_data, dx=10, dt=10):
    """
    Plot macroscopic flow (Q), density (Rho) or speed (V) in a 1x3 grid plot.
    for on-ramp scenario

    Parameters:
    ----------
    macro_data : dict
        Path where macroscopic data is located.
        macro_data = {
            "speed": np.array(),
            "flow": np.array(),
            "density": np.array(),
         }
    dx : float, optional
        Spatial discretization in meter.
    dt : float, optional
        Temporal discretization in second.
    hours: float
        number of hours to be plotted
    Returns: None
    """
    fs = 18
    minutes = 10
    length = int(minutes * 60/dt)
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['font.size'] = fs
    fig, axs = plt.subplots(1,3, figsize=(20, 6))
    Q, Rho, V = macro_data["flow"][:length,:], macro_data["density"][:length,:], macro_data["speed"][:length,:]

    # flow
    h = axs[0].imshow(Q.T*3600, aspect='auto',vmin=0, vmax=4000)# , vmax=np.max(Q.T*3600)) # 2000 convert veh/s to veh/hr/lane
    colorbar = fig.colorbar(h, ax=axs[0])
    axs[0].set_title("Flow (vph)")

    # density
    h= axs[1].imshow(Rho.T*1609.34, aspect='auto',vmin=0, vmax=600) #, vmax=np.max(Rho.T*1000)) # 200 convert veh/m to veh/mile
    colorbar = fig.colorbar(h, ax=axs[1])
    axs[1].set_title("Density (veh/mile)")

    # speed
    h = axs[2].imshow(V.T * 2.23694, aspect='auto',vmin=0, vmax=60) #, vmax=110) # 110 convert m/s to mph
    colorbar = fig.colorbar(h, ax=axs[2])
    axs[2].set_title("Speed (mph)")

    def time_formatter(x, pos):
        # Calculate the time delta in minutes
        minutes = 5*60 + x * xc # starts at 5am
        # Convert minutes to hours and minutes
        time_delta = datetime.timedelta(minutes=minutes)
        # Convert time delta to string in HH:MM format
        return str(time_delta)[3:]  # Remove seconds part

    # Multiply x-axis ticks by a constant
    xc = dt/60  # convert sec to min
    yc = dx
    for ax in axs:
        ax.invert_yaxis()
        ax.xaxis.set_major_formatter(FuncFormatter(time_formatter))
        yticks = ax.get_yticks()
        ax.set_yticklabels([str(int(tick * yc)) for tick in yticks])
        ax.set_ylabel("Position (m)")
        ax.set_xlabel("Time (min)")
        
    plt.tight_layout()
    return fig, axs




def plot_macro(macro_data, dx=10, dt=10, hours=3):
    """
    Plot macroscopic flow (Q), density (Rho) or speed (V) in a 1x3 grid plot.
    for I-24 scenario

    Parameters:
    ----------
    macro_data : dict
        Path where macroscopic data is located.
        macro_data = {
            "speed": np.array(),
            "flow": np.array(),
            "density": np.array(),
         }
    dx : float, optional
        Spatial discretization in meter.
    dt : float, optional
        Temporal discretization in second.
    hours: float
        number of hours to be plotted
    Returns: None
    """
    length = int(hours * 3600/dt)
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['font.size'] = 18
    fig, axs = plt.subplots(1,3, figsize=(20, 6))
    Q, Rho, V = macro_data["flow"][:length,:], macro_data["density"][:length,:], macro_data["speed"][:length,:]

    # flow
    h = axs[0].imshow(Q.T*3600, aspect='auto',vmin=0, vmax=8000)# , vmax=np.max(Q.T*3600)) # 2000 convert veh/s to veh/hr/lane
    colorbar = fig.colorbar(h, ax=axs[0])
    axs[0].set_title("Flow (nVeh/hr)")

    # density
    h= axs[1].imshow(Rho.T*1000, aspect='auto',vmin=0) #, vmax=np.max(Rho.T*1000)) # 200 convert veh/m to veh/km
    colorbar = fig.colorbar(h, ax=axs[1])
    axs[1].set_title("Density (veh/km)")

    # speed
    h = axs[2].imshow(V.T * 2.23694, aspect='auto',vmin=0, vmax=80) #, vmax=110) # 110 convert m/s to mph
    colorbar = fig.colorbar(h, ax=axs[2])
    axs[2].set_title("Speed (mph)")

    def time_formatter(x, pos):
        # Calculate the time delta in seconds
        seconds = 5 * 3600 + x * xc * 60  # Starts at 5:00 AM
        # Convert seconds to hours and minutes in HH:MM format
        base_time = datetime.datetime(1900, 1, 1, 0, 0)  # Arbitrary base date
        time = base_time + datetime.timedelta(seconds=seconds)
        return time.strftime("%H:%M")

    # Multiply x-axis ticks by a constant
    xc = dt/60  # convert sec to min
    yc = dx
    for ax in axs:
        ax.invert_yaxis()
        ax.xaxis.set_major_formatter(FuncFormatter(time_formatter))
        ax.xaxis.set_major_locator(MultipleLocator(60 / xc))
        ax.set_xticklabels(ax.get_xticklabels(), rotation=45)
        yticks = ax.get_yticks()
        # ax.set_yticks(yticks)
        ax.set_yticklabels(["{:.1f}".format(57.6- tick * yc / 1609.34 ) for tick in yticks])
        # ax.set_yticklabels([str(int(tick * yc/ 1609.34)) for tick in yticks])
        ax.set_xlabel("Time (hour of day)")
        ax.set_ylabel("Milemarker")
        
    plt.tight_layout()
    plt.show()







def calc_tot_time_spent(rho_matrix, dx=0.1, dt=30):
    """
    Integrate density map to get total time spent

    Parameters:
    rho_matrix : np.array
        in veh / mile / lane
    dx : in mile
    dt : in sec

    Return: tot_time_spent, float
        Unit: veh x time (sec)
    """
    return np.sum(rho_matrix) * dx * dt

def calc_travel_time(v_matrix, dx=160.934, dt=30):
    """
    Calculate lane-avg. travel time given varying departure time

    Parameters:
    v_matrix : np.array [N_time x N_space]
        in m/s
    dx : in mile
    dt : in sec

    Return: tot_time_spent, float
        Unit: veh x time (sec)
    """
    
    hours = 5
    departure_time_arr = np.linspace(0, hours*3600, 5*60) # generate a VT every 5min
    # departure_time_arr = [5250]
    departure_time = []
    travel_time = [] # travel time corresponding to each departure time
    for t0 in departure_time_arr:
        t_arr, x_arr = gen_VT(v_matrix, t0, x0=0, dx=dx, dt=dt)
        if x_arr[-1]-x_arr[0] >= 5632:
            travel_time.append(t_arr[-1]-t_arr[0])
            departure_time.append(t0)
        # plt.plot(t_arr, x_arr, "x-")
        # plt.xlabel("Time (sec)")
        # plt.ylabel("Space (m)")
        # plt.show()


    return departure_time, travel_time



def compare_macro(macro_data_1, macro_data_2):
    '''
    TO BE REMOVED
    '''
    fig, axs = plt.subplots(1,3, figsize=(20, 5))
    Q1, Rho1, V1 = macro_data_1["flow"], macro_data_1["density"], macro_data_1["speed"]
    Q2, Rho2, V2 = macro_data_2["flow"], macro_data_2["density"], macro_data_2["speed"]
    Q, Rho, V = Q1-Q2, Rho1-Rho2, V1-V2

    # flow
    h = axs[0].imshow(Q.T*3600, aspect='auto',vmin=-1500, vmax=1500, cmap="bwr") # convert veh/s to veh/hr
    colorbar = fig.colorbar(h, ax=axs[0])
    axs[0].set_title("Flow difference (veh/hr)")

    # density
    h= axs[1].imshow(Rho.T*1000, aspect='auto',vmin=-100, vmax=100, cmap="bwr") # convert veh/m to veh/km
    colorbar = fig.colorbar(h, ax=axs[1])
    axs[1].set_title("Density difference (veh/km)")

    # speed
    h = axs[2].imshow(V.T * 3.6, aspect='auto',vmin=-10, vmax=10, cmap="bwr") # convert m/s to km/hr
    colorbar = fig.colorbar(h, ax=axs[2])
    axs[2].set_title("Speed difference (km/hr)")


    # Multiply x-axis ticks by a constant
    dx, dt = 10, 10
    xc = dt/60  # Example constant
    yc = dx
    for ax in axs:
        ax.set_xlim([0,8])
        ax.invert_yaxis()
        xticks = ax.get_xticks()
        ax.set_xticklabels([str(int(tick * xc)) for tick in xticks]) # convert to milemarker
        yticks = ax.get_yticks()
        ax.set_yticklabels([str(int(tick * yc)) for tick in yticks])
        ax.set_xlabel("Time (min)")
        ax.set_ylabel("Space (m)")
    plt.show()

def idm_fd(theta):
    '''
    TO BE REMOVED
    Traffic Flow Dynamics Treiber L09 https://www.mtreiber.de/Vkmod_Skript/Lecture09_Micro2.pdf
    Get the homogeenous steady state s_e(v) from IDM parameters (same cars)
    Derive macroscopic FD out of s_e(\rho)
    '''
    v0, s0, T, a, b, l = theta # l is car length, must be homogeneous fleet

    v_arr = np.linspace(0, v0-0.01, 100) # m/s   
    s_e_arr = (s0+v_arr*T) / (np.sqrt(1-(v_arr/v0)**4))
    rho_arr = 1/(s_e_arr+l)
    q_arr = v_arr * rho_arr

    return v_arr, s_e_arr, rho_arr, q_arr

def plot_multiple_idm_fd(thetas, legends=None):
    '''
    TO BE REMOVED
    thetas is a list of theta's
    '''
    fig, axs = plt.subplots(1,3, figsize=(15,7))
    if legends is None:
        legends = ['' for _ in range(len(thetas))]
    for i, theta in enumerate(thetas):
        v_arr, s_e_arr, rho_arr, q_arr = idm_fd(theta)
        axs[0].plot(s_e_arr, v_arr, label=legends[i])
        axs[1].plot(rho_arr*1000, v_arr)
        axs[2].plot(rho_arr*1000, q_arr*3600 )

    axs[0].set_xlabel('Gap $s$ [m]')
    axs[0].set_ylabel('$v_e$ [m/s]')
    axs[0].set_xlim(left=0)
    axs[0].set_ylim(bottom=0)

    axs[1].set_xlabel('Density $\\rho$ [veh/km]')
    axs[1].set_ylabel('$v_e$ [m/s]')
    axs[1].set_xlim(left=0)
    axs[1].set_ylim(bottom=0)

    axs[0].legend(loc='upper left')
    axs[2].set_xlabel('Density $\\rho$ [veh/km]')
    axs[2].set_ylabel('Flow $q$ [veh/hr]')
    axs[2].set_xlim(left=0)
    axs[2].set_ylim(bottom=0)
    
    # axs[2].set_ylim([0, 3200])
    # plt.tight_layout()
    plt.show()
    return

def calc_ss_speed(rho, s0, tau, l):
    '''
    TO BE REMOVED
    calculate steady state speed given rho (veh/km)
    '''
    s_e = 1000/rho # equilibrium spacing
    gap_e = s_e - s0 - l
    v_e = gap_e / tau
    print(f"steady state speed: {v_e}")
    return v_e

def get_detector_data(xml_file):
    '''
    TO BE REMOVED
    plot the flow/density/speed relationship from xml_file (.out.xml)
    v, rho, q are background equilibrium macro quantities, derived from IDM parameters
    '''
    try:
        tree = ET.parse(xml_file)
    except:
        with open(xml_file, 'a') as file:
            file.write("</detector>" + '\n')

    tree = ET.parse(xml_file)
    root = tree.getroot()

    data = {}
    for interval in root.findall('interval'):
        id_value = interval.get('id')
        occupancy = float(interval.get('occupancy'))
        flow = float(interval.get('flow'))
        speed = float(interval.get('speed'))
        if speed == -1:
            continue
        interval_time = float(interval.get('end')) - float(interval.get('begin'))
        nVeh = float(interval.get('nVehContrib'))
        # vLen = float(interval.get('length'))
        # convert occupancy to density
        # occupancy_fraction = occupancy / 100
        effective_length = speed * interval_time
        density = nVeh / (effective_length / 1000) # vehcles per km

        if id_value not in data:
            data[id_value] = defaultdict(list)
        
        data[id_value]['occupancy'].append(occupancy)
        data[id_value]['flow'].append(flow)
        data[id_value]['speed'].append(speed)
        data[id_value]['density'].append(density)
    
    return data

def plot_detector_data(xml_file, idm_param, initial_val=None):
    '''
    TO BE REMOVED
    Overlay FD of idm_param with loop detector data from simulation (det_data)
    '''
    l = idm_param[-1]
    v_arr, s_e_arr, rho_arr, q_arr = idm_fd(idm_param)
    det_data = get_detector_data(xml_file)
    

    fig, axs = plt.subplots(2,1, figsize=(4,7))

    # plot background FD
    axs[0].plot(rho_arr*1000, v_arr, label="IDM FD")
    axs[1].plot(rho_arr*1000, q_arr*3600, label="IDM FD" )
    

    axs[0].set_xlabel('Density $\\rho$ [veh/km]')
    axs[0].set_ylabel('$v_e$ [m/s]')
    
    axs[1].set_xlabel('Density $\\rho$ [veh/km]')
    axs[1].set_ylabel('Flow $q$ [veh/hr]')
    
    # plt.tight_layout()

    # plot detector data
    for i, (id_value, values) in enumerate(det_data.items()):
        print(i)
        axs[0].scatter(values['density'], values['speed'], label="loop detector" if i==0 else "")
        axs[1].scatter(values['density'], values['flow'], label="loop detector" if i==0 else "")

    # plot initial simulation values
    if initial_val is not None:
        axs[0].scatter(initial_val[0], initial_val[1], label="initial")
        axs[1].scatter(initial_val[0], initial_val[0]*initial_val[1]*3.6, label="initial")
    
    axs[0].set_xlim(left=0)
    axs[0].set_ylim(bottom=0)
    axs[1].set_xlim(left=0)
    axs[1].set_ylim(bottom=0)

    axs[0].legend()
    axs[1].legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    print("not implemented")