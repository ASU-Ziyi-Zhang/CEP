# cav-sumo

Repository containing various calibrated SUMO scenarios and autonomous vehicle controllers for microsimulation of mixed traffic.

This library uses the TraCI API to directly command the acceleration behavior of autonomous vehicle agents in microsimulation.


<!-- Requires **I24scenario/** folder to be populated from other project <https://github.com/yanb514/CorridorCalibration> -->

<!-- Requires shared library **.so / .dll** files to be generated from another project <https://github.com/tylerard33/cav_codegen_examples> and needs the accompanying **cwrapper.py** and **cppwrapper.py** interfaces. To run, pull these files into the current working directory -->


## File Contents Structure
Compiled autonomous vehicle controllers are generated in **codegen/** folder

The main source files to execute the CAV controllers are located in the **src/** folder

Simulation analysis scripts are located in **scripts/** folder

Simulation output data will be generated into a local **<scenario\>/output/** folder

Simulation saved plots will be generated into a local **figures/** folder

Various helper scripts for adjusting the simulations are located in **tools/** folder

Everything-in-the-loop (XIL) extended framework scripts are located in **ext/**

Uncalibrated and nominal scenarios are located in **dev/** folder

## Dependencies
    - Python
    - pip

    - cmake (If compiling a codegen controller)

## Linux Installation

From the parent **cav_sumo** directory, run the installer script for a Python installation of SUMO and its libraries

    . install.sh

This installer will first check for a valid ``Python`` installation and ``pip`` installation, and then install SUMO. By default, this will install to the global Python installation found in *env* path. 

*Optionally, a user can open and modify the script settings to install to a virtual environment.*

> USING_VENV=1

### Compiled controller installation
This installer will additionally run the CAV controller installers, which will compile the CAV controls and move their shared libraries to the parent **cav_sumo/** directory

The codegen installer relies on having ``cmake`` installed and found in the system path. 

Additionally, this installer will copy the corresponding controller API *cwrapper.py* files to the **src/** directory. To run only the controller installers:

    cd codegen
    . install.sh

## Windows Installation

If compiling the autogen CAV controllers, make sure that Visual Studio C/C++ tools are installed, along with cmake. Then use the Windows developer command prompt to either launch `git bash` or continue the installation with `Windows developer command prompt for VS`

### Using git bash
The ``git bash`` tool can be used to run a unix-like terminal, and the above Linux installation steps can be followed from there. 

### Using Windows developer command prompt for VS
The developer command prompt can execute a Windows installer script

    install.bat

This installer will additionally attempt to run the CAV controller installers, which will compile the CAV controls using ``cmake`` and then move the resulting shared libraries into the parent **cav_sumo/** directory

Additionally, this installer will copy the corresponding controller API *cwrapper.py* files to the **src/** directory. To run only the controller installers without the SUMO install:

    cd codegen
    install.bat

## Using manual installation commands
Otherwise, to manually install from a terminal:

    pip install eclipse-sumo
    pip install traci
    pip install libsumo

To install the vehicle controllers (PCC in this case), given ``cmake`` is installed:

    cd codegen

    BUILD_TYPE=Release # Can choose: Debug, RelWithDebInfo, Release

    cd pcc_codegen
    
    mkdir build

    cmake -B build -DCMAKE_BUILD_TYPE=$BUILD_TYPE
    cmake --build build --config $BUILD_TYPE
    cmake --install build --config $BUILD_TYPE

    rm -r build
    
    python test.py

If the test suite reports no problems, then the appropriate program files can be copied to the cav_sumo directories for use. In the PCC controller case:

    cp lib/pcc_so.dll ../../
    cp _cppwrapper.py ../../src/cppwrapper.py

    cd ..

## Run SUMO mixed traffic simulations

From the parent **cav_sumo** directory, run the main Python script

    python main.py

This script comes with several command line options to affect the simulation. Check the options via help command

    python main.py -h

Several options allow a user to specify the penetration rate of automated agents, select the scenario to run, etc

### The basic TraCI simulation loop

The basic loop for running a cruise control in Python is as follows:

    while traci.simulation.getMinExpectedNumber() > 0: # While there are active vehicles/pedestrians in network
        ## Run simulation step
        traci.simulationStep()

        ## Process microsimulation data
        # Get simulation status
        sim_time = traci.simulation.getTime()

        # Get vehicle data
        vehicle_ids = traci.vehicle.getIDList()

        type_gen = (ego_id for ego_id in vehicle_ids if traci.vehicle.getTypeID(ego_id) == 'cav') # Generator to get vehicles that match a given vType string
        for ego_id in type_gen:
            ### Get ego states
            ego_distance = traci.vehicle.getDistance(ego_id)
            ego_speed = traci.vehicle.getSpeed(ego_id)
            ego_accel = traci.vehicle.getAcceleration(ego_id)

            # Get preceding vehicle states
            leader = traci.vehicle.getLeader(ego_id)
            if leader is not None and leader[0] != "": # There is a vehicle in front
                lead_id = leader[0] # string of "flow id.veh id"
                
                lead_rel_distance = leader[1] # [m] (Front bumper + min_gap) to (rear bumper)
                lead_speed = traci.vehicle.getSpeed(lead_id) # [m/s]
                lead_accel = traci.vehicle.getAcceleration(lead_id) # [m/s2]
            
            else: # Assign some default values
                lead_rel_distance = 2000.
                lead_speed = 0.
                lead_accel = 0.

            ### Run cruise controller
            desired_acceleration = getCommand(sim_time, ego_accel, ego_speed, ego_distance, s_max, v_max, lead_accel, lead_speed, lead_rel_distance)

            ### Set SUMO values
            # Set ego acceleration
            acc_duration = 2.0 # [s] Little documentation on this - How many seconds the command is filtered through an internal low pass filter on realized acceleration? 
            traci.vehicle.setAcceleration(ego_id, desired_acceleration, acc_duration)

## Run analysis of SUMO traffic simulations

From the parent **cav_sumo** directory, run the analysis Python script

    python analysis.py

Check the options via help command 

    python analysis.py -h

Several options allow a user to select the data file to read, select the folder to search for the data file, etc


## Run XIL-based simulations with SUMO mixed traffic

From the parent **cav_sumo** directory, run the main XIL Python script

    python main_xil.py

This script launches SUMO with a multi-threaded server real-time listening for incoming client messages and embeds them into simulation. Check the options via help command

    python main_xil.py -h

Several options allow a user to specify all the previous simulation-related options. Additionally, the number of external clients, the server port, the server IP address can be specified.

#### Example client program
To perform SIL testing to check functionality, in a separate terminal, run the vehicle simulation script to launch a client.

    python -m ext.vehicle_sim

Pass `-h` as before to get help options.

## Dashboard (dashboard.py)

This repository includes an interactive dashboard script (`dashboard.py`) that
loads analysis outputs (exported Excel/CSV datasets from simulations) and provides
visualization and exploration tools (time-space diagrams, aggregated metrics,
and per-scenario charts).

Purpose
- Visualize simulation metrics and time-space datasets produced by the analysis
    pipeline (e.g., outputs written to `<scenario>/excel_file`).
- Provide an interactive Dash/Flask-based UI to explore runs and export figures.

Inputs
- A scenario folder containing analysis outputs (default: `sumo_scenarios/<scenario>/excel_file`).
- Time-space CSVs and summary metrics produced by `analysis.py` or the
    `export_*` helpers in `dashboard.py`.

How to run
1. Ensure dependencies are installed (Dash, plotly, pandas, etc.).
2. From the repository root, run:

```powershell
# Example: run the dashboard for the 'onramp' scenario
python dashboard.py --scenario_folder sumo_scenarios --scenario onramp --urban no --p 0.1
```


3. The script starts a local Flask/Dash server (default port 8050). Open
     http://localhost:8050 in your browser to view the dashboard.

Options / Notes
- Use `--file` to point to a specific timespace CSV if you want to open a
    single dataset instead of the scenario folder.
- You can pass lane-filtering options such as `--exclude-lane`,
    `--exclude-lane-prefix` and `--lane-pos-window` to control which samples are
    included in the visualizations (see `dashboard.py -h` for full CLI).
- The dashboard expects the analysis outputs to be present; run
    `analysis.py` first if you haven't generated the `excel_file` dataset.

For scenario:
    omramp:
    python dashboard.py --scenario onramp --urban no --exclude-lane ramp_0 --exclude-lane E2_0 --p 0.1

    i24：
    python dashboard.py --scenario i24 --urban no --exclude-lane E2_0 --exclude-lane E4_0 --exclude-lane E6_0  --exclude-lane E1_0 --p 0.1


# TODO 
### Microsim
Add traffic light scenarios - base it on Peachtree and Chicago connected corridor locations and timings (just got travel time data)

Add traffic light CAV eco-approach control interface

Add microsim Quality Control script - This script will check for various safety and performance metrics across a batch of simulations and summarize them into a single table for display. Metrics include energy economy (for only HVs, only AVs, and total mixed fleet), number of collisions, number of traffic rule violations, number of unsafe detected behaviors/near-misses, travel time, throughput, etc

Improve merging control for CAVs

### XIL Testing
(Optional but nice to have) Add a scripted way to adjust the traffic light positions and timings

(Optional but nice to have) Add a scripted way to build the road network based on desired X-Y coordinates

Add a scripted way to place initial starting locations of microsimulated traffic agents

Add a ROS binding


# Debugging/FAQ

- The <cav_controller> shared library file cannot be found, but I can see it in my directory

    > Check that the installation path in the *PCC* class within **agents.py** points to the correct file location.

    > After this, the <cav_controller> file may need to be re-compiled for your system. Follow the compiled controller install steps above, which rely on a working *cmake* installation.