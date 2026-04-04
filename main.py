#!/usr/bin/env python3

#######################################
### Tyler Ard                       ###
### Yaozhong Zhang                  ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

### python -m cProfile -o <output_name>.prof <script_name>.py
### snakeviz <output_name>.prof

### https://sumo.dlr.de/docs/Tutorials/Hello_SUMO.html

# Running microsimulations
### https://sumo.dlr.de/docs/sumo.html
### https://sumo.dlr.de/docs/TraCI.html
### https://sumo.dlr.de/docs/Libsumo.html

### https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html
### https://sumo.dlr.de/docs/TraCI/Traffic_Lights_Value_Retrieval.html
### https://sumo.dlr.de/docs/Definition_of_Vehicles%2C_Vehicle_Types%2C_and_Routes.html

### https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html

# More detailed documentation
### https://sumo.dlr.de/pydoc/traci._edge.html

# Modeling
### https://sumo.dlr.de/docs/Simulation/SublaneModel.html
### https://sumo.dlr.de/docs/Simulation/Safety.html

# Outputting data
### https://sumo.dlr.de/docs/Tools/Xml.html#xml2csvpy

# Creating networks
### https://sumo.dlr.de/docs/netconvert.html
### https://sumo.dlr.de/docs/Netedit/index.html


import importlib
import numpy as np
import pandas as pd
import json
from time import perf_counter as counter
from time import sleep
import warnings
import os
import argparse
from typing import List, Dict
from math import fmod, pi, floor
from copy import deepcopy
import random
from datetime import datetime
import xml.etree.ElementTree as ET

from src.agents import PCC, CAV, EXT
from src.sensing import Communications
from src.settings import RADAR_RANGE, VISION_RANGE, CONN_RANGE
from src.logging import logger, pretty_printer

import scripts.utils_data_read as reader

import parsers.sumo
import parsers.xil
import parsers.server

from sumolib import checkBinary

### Handle input arguments
PARSER = argparse.ArgumentParser(description='Python SUMO Simulation command-line options')
parsers.sumo.register_parser(PARSER)
# parsers.xil.register_parser(PARSER)
# parsers.server.register_parser(PARSER)

ARGS = PARSER.parse_args() # Parse

# Import TraCI or libsumo, or their Polaris versions, based on command-line argument
traci = importlib.import_module( ARGS.backend ) # traci or libsumo are imported from CLI logic from --backend 

# -------------------------------------------------------------------------------------------------------

MERGING_SCENARIOS = ['i24', 'onramp']
TRAFFIC_LIGHT_SCENARIOS = ['chi_clinton_canal', 'roosevelt', 'roosevelt_simple']

ONRAMP_LANES = ['ramp_0', 'E2_0', 'E1_0', 'E3_0', 'E4_0', 'E6_0', 'E7_0'] # TODO - Hardcoded this list of lane ids for now for onramp scenario - all lanes attached to onramp
I24_ENDING_LANES = ['E1_0', 'E7_0'] # TODO - Hardcoded this list of lane ids for now for onramp scenario - lanes that end at the end of the onramp
ONRAMP_ENDING_LANES = ['E2_0'] # TODO - Hardcoded this list of lane ids for now for onramp scenario - lanes that end at the end of the onramp

class simulation():
    ### Class definitions
    def __init__(self, args: argparse.Namespace | None = None):
        '''
        Initialize a wrapper around the SUMO microsimulation tool.

        Inputs:
        - args (argparse): Command-line input arguments. If empty, will check for default input arguments
        '''
        
        ### Handle input arguments
        if not args:  
            self.args = ARGS
            
        else:
            self.args = args

        # Check command line arguments
        if self.args.penetration >= 5: # If penetration is given as a percentage then convert to a rate
            self.args.penetration /= 100.

        assert self.args.penetration <= 1 and self.args.penetration >= 0, 'Penetration rate must be in interval [0, 1].'
        self.penetration_tag = f"p{self.args.penetration:g}"
        self.run_tag = f"{self.args.scenario}_{self.penetration_tag}"
        if self.args.time_to_teleport is None:
            # 60 s is short enough to trigger within a single green phase at an
            # urban intersection (~40-60 s), teleporting any CAV whose ACC
            # controller fails to resume after a red stop.
            self.time_to_teleport = 60.0 if self.args.scenario in TRAFFIC_LIGHT_SCENARIOS else -1.0
        else:
            self.time_to_teleport = float(self.args.time_to_teleport)
        
        if self.args.replay:
            # Error check for replay arguments
            assert self.args.replay_dir is not None, 'Replay directory must be specified when using --replay flag.'
            assert self.args.replay_file is not None, 'Replay file must be specified when using --replay flag.'
        
            assert self.args.no_inflow == True, 'Replay mode must be run with no inflow.'
            assert self.args.gui == True, 'Replay mode must be run with GUI enabled.'

            # Select the output data folder
            replay_dir = os.path.join(self.args.scenario_folder, self.args.scenario, self.args.replay_dir) # Directory where output file to read is located
            
            assert os.path.isdir(replay_dir), f'Cannot find replay directory! {replay_dir}'

            # Initialize metrics with stats file if exists
            self.replay_fcd_path = os.path.join(replay_dir, self.args.replay_file)
            assert os.path.exists(self.replay_fcd_path), f'Cannot find replay fcd file! {self.replay_fcd_path}'
    
        ### Initialize SUMO microsimulation
        # Check library imports
        if traci.isLibsumo():
            print('Using lib sumo.')

            # Error check
            assert not self.args.gui, 'Libsumo API does not support using GUI option! Adjust the Python import, or adjust command-line --gui option'

        # SUMO scenario input settings
        SUMO_CFG_FILE = f"{self.args.scenario_folder}/{self.args.scenario}/{self.args.scenario}.sumocfg"
        self.route_output_file = f"{self.args.scenario_folder}/{self.args.scenario}/{self.args.scenario}_{self.penetration_tag}.rou.xml"
        
        assert os.path.isfile(SUMO_CFG_FILE), f'Could not find .sumocfg file at {SUMO_CFG_FILE}'

        # SUMO scenario output settings
        SUMO_OUT_DIR = f"{self.args.scenario_folder}/{self.args.scenario}/output" # Directory where extra output data defined here through TraCI will be saved
        
        if self.args.timestamp_output: # Will make the output folder have a output_<date>/ format
            now = datetime.now()
            self.formatted_timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
            
            SUMO_OUT_DIR += '_' + self.formatted_timestamp # Add the current timestamp to the output folder 
        
        else:
            self.formatted_timestamp = ''

        os.makedirs(SUMO_OUT_DIR, exist_ok=True) # Make the data directory if it does not exist
        self.output_dir = SUMO_OUT_DIR
        self.detector_output_file = os.path.join(SUMO_OUT_DIR, f"detector_output_{self.penetration_tag}.xml")
        self.detector_additional_file = os.path.join(SUMO_OUT_DIR, f"sensors_{self.penetration_tag}.add.xml")
        self.tls_output_file = os.path.join(SUMO_OUT_DIR, f"tls_{self.penetration_tag}.xml")
        self.tls_additional_file = os.path.join(SUMO_OUT_DIR, f"tls_output_{self.penetration_tag}.add.xml")

        # SUMO exe launch settings
        if self.args.gui: # GUI is requested and TraCI is imported - libsumo does not support GUI
            SUMO_BINARY = checkBinary('sumo-gui')
        
        else:
            SUMO_BINARY = checkBinary('sumo')

        # Set up SUMO start command
        SUMO_CMD = [
            # SUMO run command and configuration file
            SUMO_BINARY,
            '-c', SUMO_CFG_FILE,
            "--start",
            
            ### Additional commands that override the .sumocfg
            '--no-warnings', # Remove the warnings output to command line

            # Simulation behavior
            '--collision.action', 'warn', # - teleport, warn, none, remove
            '--time-to-teleport', f'{self.time_to_teleport:g}',

            '--step-length', str(0.10), # [s] Time interval between simulation steps
            '--no-step-log', # Add this flag if suppressing the TraCI outputs to command line
            '--step-log.period', '50', # Every n simulation steps TraCI output to command line
            
            # Lane change modeling
            # '--lateral-resolution', str(LANEWIDTH/1.0), # [m] Exclusive for when using sublane model - Sublane models lane width - Default true lane width is 3.2m
            # '--lanechange.duration', '5.0', # [s] Exclusive for when using continuous lane change model
            
            # Random seed fix
            '--seed', str(self.args.seed) # Random seed for SUMO simulation rng
        ]

        # If SUMO should be logging output, then add additional output commands to the SUMO command list
        if not self.args.no_inflow: # In non-replay case where inflows to network are used, we want to log output data
            SUMO_CMD += [
                # Output logging
                '--fcd-output', os.path.join(SUMO_OUT_DIR, f"fcd_{self.penetration_tag}.xml"), # Log floating car data every simulation step
                '--fcd-output.acceleration', # Add acceleration to output
                '--fcd-output.max-leader-distance', str(RADAR_RANGE), # add attributes leaderGap, leaderSpeed, leaderID whenever a vehicle has a leader within the given distance. Otherwise, leaderID will be "" and leaderGap, leaderSpeed will be -1.
                
                '--collision-output', os.path.join(SUMO_OUT_DIR, f"coll_{self.penetration_tag}.xml"), # Log collision car data
                '--edgedata-output', os.path.join(SUMO_OUT_DIR, f"by_edge_{self.penetration_tag}.xml"),
                '--lanedata-output', os.path.join(SUMO_OUT_DIR, f"by_lane_{self.penetration_tag}.xml"),
                '--statistic-output', os.path.join(SUMO_OUT_DIR, f"stats_{self.penetration_tag}.xml"),
                '--summary-output', os.path.join(SUMO_OUT_DIR, f"summary_{self.penetration_tag}.xml"),
                '--tripinfo-output', os.path.join(SUMO_OUT_DIR, f"tripinfo_{self.penetration_tag}.xml"),
            ]

        if self.is_traffic_light_scenario() and not self.args.replay:
            sensors_template = os.path.join(self.args.scenario_folder, self.args.scenario, "sensors.add.xml")
            self.write_detector_output_additional_file(
                sensors_template,
                self.detector_additional_file,
                self.detector_output_file,
            )
            self.write_tls_output_additional_file(self.tls_additional_file, self.tls_output_file)
            additional_files = [
                self.detector_additional_file,
                os.path.join(self.args.scenario_folder, self.args.scenario, "signal.add.xml"),
                self.tls_additional_file,
            ]
            additional_files = [path for path in additional_files if os.path.isfile(path)]
            SUMO_CMD += ['--additional-files', ",".join(additional_files)]

        # Modify flow according to penetration rate
        reader.update_flows(penetration_rate=self.args.penetration, 
                            input_file=f"{self.args.scenario_folder}/{self.args.scenario}/{self.args.scenario}_template.rou.xml", 
                            output_file=self.route_output_file,
                            seed=self.args.seed)
        SUMO_CMD += ['--route-files', self.route_output_file]

        print(f'Starting SUMO with configuration file: {SUMO_CFG_FILE}.')
        print(f'Run settings: scenario={self.args.scenario}, backend={self.args.backend}, gui={self.args.gui}, replay={self.args.replay}, penetration={self.args.penetration:g} ({self.penetration_tag}).')
        print(f'Time-to-teleport: {self.time_to_teleport:g} s.')
        print(f'Generated route file: {self.route_output_file}.')
        print(f'Outputting additional SUMO data to dir: {SUMO_OUT_DIR}.')
        if self.is_traffic_light_scenario() and not self.args.replay:
            print(f'Writing detector output to: {self.detector_output_file}.')
            print(f'Writing TLS states to: {self.tls_output_file}.')

        ### Initialize (C)AV controllers
        if self.args.penetration > 0 or True:
            # Urban scenarios now keep one Python controller object per CAV so
            # per-vehicle stop/sign flags do not leak across the fleet. The
            # underlying C controller API is initialized once and then shared.
            if self.is_traffic_light_scenario():
                self.shared_cav_api = None
            else: # Non traffic light scenario
                self.ego = PCC() # Predictive cruise controller for highway

        # Initialize CAV communications
        self.comms = Communications() # Structure where communications are stored for later access

        ### Start SUMO using traci/libsumo
        traci.start(SUMO_CMD)

        ### Parameters
        # Timer parameters
        self.sim_time = 0. # [s] Current simulation time - updated each step
        self.dt = traci.simulation.getDeltaT() # [s] time between each simulation frame - needed for realtime option
        
        self.rt_start_time = counter() # [unix s] Realtime tic timer keeping track of time when last simulation frame was executed
        self.rt_iter = 0 # Number of simulation iterations executed

        # Color parameters
        self.colors = { # Tuple of RGBA: 0-255 RGB scale, 0-255 Alpha scale
            'hdv': (200, 200, 50, 255),
            'cav': (50, 50, 220, 255),
            'ramp': (110, 170, 250, 255),
            'ext': (50, 255, 50, 255),
            'error': (255, 0, 0, 255),
            'ghost': (0, 0, 0, 0)
        }

        # Control parameters
        self.acc_duration = 1.0 # Time spent in acceleration filter

        # SUMO traci bookkeepping variables
        self.vehs = {} # Object placed into microsimulation framework
        
        self.spawned_vehs = {} # Dict of Id, Type containing currently spawned vehicles
        self.has_been_set_vehs = set() # Vehicle ids that have had one-time properties set
        self.cav_controllers = {} # Per-vehicle Python controller state for urban CAVs
        self.last_route_lane_guidance = {} # Last sim-time when route lane guidance was issued per vehicle
        self.route_lane_guidance_debug = [] # Rolling buffer of recent route-lane nudges for crash diagnosis
        self.cav_stuck_ticks = {} # Consecutive ticks each CAV has been at the stop line with speed≈0
        self.cav_suspended_until = {} # sim_time after which suspended CAVs resume TraCI control

        self.TLInfo = {} # Traffic light info
        self.SPATmsgPrev = {} # 

        self.veh_log = []
        self.sim_log = []
        self.summary_interval = 10.0
        self.next_summary_time = 0.0
        self.wall_start_time = counter()

    def stop(self):
        '''Close the SUMO simulation and connection from TraCI'''
        self.cav_controllers.clear()
        self.has_been_set_vehs.clear()
        if hasattr(self, "shared_cav_api"):
            self.shared_cav_api = None

        # Save custom log data
        if self.veh_log and self.sim_log: # Check if logging was done
            # Make data frames
            dfveh = pd.DataFrame(self.veh_log)
            dfsim = pd.DataFrame(self.sim_log)

            # Make save directory
            save_dir_parts = ["logs", self.args.scenario, self.penetration_tag]
            if self.formatted_timestamp:
                save_dir_parts.append(self.formatted_timestamp)
            save_dir = os.path.join(*save_dir_parts)
            os.makedirs(save_dir, exist_ok=True)

            print(f'Saving log to {save_dir}.')
            
            # Make CSV and JSON outputs
            dfveh.to_csv(os.path.join(save_dir, "veh_log_data.csv"), index=False)
            dfsim.to_csv(os.path.join(save_dir, "sim_log_data.csv"), index=False)

            with open(os.path.join(save_dir, f"{self.args.scenario}_tl_data.json"), "w") as f:
                json.dump(self.TLInfo, f, indent=2)

        # Close SUMO
        try:
            if traci.isLoaded():
                print('Closing SUMO.')
                traci.close() # TODO This throws an error sometimes?
        except:
            pass

    def get_controller_for_vehicle(self, ego_id: str):
        """
        Return the controller instance responsible for this vehicle.

        Urban traffic-light scenarios use one Python CAV object per vehicle so
        controller-side flags remain vehicle-local, while sharing one native
        C API instance to avoid repeated controller initialization.
        """

        if not self.is_traffic_light_scenario():
            return self.ego

        controller = self.cav_controllers.get(ego_id)
        if controller is not None:
            return controller

        if self.shared_cav_api is None:
            controller = CAV()
            self.shared_cav_api = controller.api
        else:
            controller = CAV(api=self.shared_cav_api)

        self.cav_controllers[ego_id] = controller
        return controller


    def init_vehicle(self, veh_id, route_id="mainlane", type_id="DEFAULT_VEHTYPE"):
        """
        """

        # Add a new vehicle
        traci.vehicle.add(veh_id, route_id)
        traci.vehicle.setType(veh_id, type_id)

        print(f"  Added new '{type_id}' vehicle '{veh_id}' with route '{route_id}'")

    def remove_vehicle(self, veh_id):
        """
        """

        # Remove a vehicle
        traci.vehicle.remove(veh_id)

        print(f'  Removed vehicle {veh_id}')

    def is_traffic_light_scenario(self):
        return self.args.scenario in TRAFFIC_LIGHT_SCENARIOS
    
    def is_merging_scenario(self):
        return self.args.scenario in MERGING_SCENARIOS
    
    def is_exit_lane(self, lane_id: str) -> bool:
        connections = traci.lane.getLinks(lane_id)
        return len(connections) == 0

    def write_tls_output_additional_file(self, additional_path: str, tls_output_path: str):
        """
        Generate a small SUMO additional file that records traffic light states
        to an XML dump readable by the urban-signal analysis pipeline.
        """
        tls_output_abs = os.path.abspath(tls_output_path).replace("\\", "/")
        root = ET.Element("additional")
        ET.SubElement(
            root,
            "timedEvent",
            {
                "type": "SaveTLSStates",
                "dest": tls_output_abs,
            },
        )
        tree = ET.ElementTree(root)
        tree.write(additional_path, encoding="UTF-8", xml_declaration=True)

    def write_detector_output_additional_file(self, template_path: str, additional_path: str, detector_output_path: str):
        """
        Clone the detector add-file and rewrite detector `file=` destinations so
        each run writes to its own penetration-specific XML output.
        """
        if not os.path.isfile(template_path):
            return

        detector_output_abs = os.path.abspath(detector_output_path).replace("\\", "/")
        tree = ET.parse(template_path)
        root = tree.getroot()

        for elem in root.iter():
            if "file" in elem.attrib:
                elem.attrib["file"] = detector_output_abs

        tree.write(additional_path, encoding="UTF-8", xml_declaration=True)

    def print_runtime_summary(self):
        '''Print a compact runtime summary so long runs are easier to monitor.'''
        vehicle_ids = traci.vehicle.getIDList()
        in_net = len(vehicle_ids)
        cav_in_net = sum(1 for veh_id in vehicle_ids if self.spawned_vehs.get(veh_id) == 'cav')
        hdv_in_net = max(in_net - cav_in_net, 0)
        loaded = len(traci.vehicle.getLoadedIDList())
        departed = len(traci.simulation.getDepartedIDList())
        arrived = len(traci.simulation.getArrivedIDList())
        min_expected = traci.simulation.getMinExpectedNumber()
        wall_elapsed = counter() - self.wall_start_time
        rtf = self.sim_time / max(wall_elapsed, 1e-9)

        print(
            f"[SIM] t={self.sim_time:6.1f}s | wall={wall_elapsed:6.1f}s | RTF={rtf:5.3f} | "
            f"minExpected={min_expected:4d} | inNet={in_net:4d} | cav={cav_in_net:4d} | "
            f"hdv={hdv_in_net:4d} | loaded={loaded:3d} | departed={departed:3d} | arrived={arrived:3d}"
        )

    def set_camera(self, veh_id, zoom_factor = 300.0):
        """
        Sets the camera position of the GUI frame to track a given vehicle id.
        """
        if self.args.gui:
            traci.gui.trackVehicle("View #0", veh_id)
            traci.gui.setZoom("View #0", zoom_factor)

            print(f'  Set camera to follow {veh_id}')

    def set_status_message(self, status = ""):
        """
        Sets the status message string in the GUI frame for display.
        """
        view_str = "View #0"

        traci.gui.setSchema(view_str, status)
        traci.gui.setOffset(view_str, x=10, y=20)  # Adjust position of the message

    def add_external_vehicle_type(self, copy_type_id, new_type_id, new_color):
        """
        Copies an existing vehicle type, modifies the color, and assigns a new ID.
        
        :param copy_type_id: The vehicle type ID to copy
        :param new_type_id: The new vehicle type ID
        :param new_color: A tuple (R, G, B, A) representing the new color
        """
        
        ### Get attributes of the original vehicle type and set a default if not defined
        params = traci.vehicletype.getParameter(copy_type_id, "width")
        width = float(params) if params else 1.9  # Default width
        
        params = traci.vehicletype.getParameter(copy_type_id, "length")
        length = float(params) if params else 4.2  # Default length
        
        params = traci.vehicletype.getParameter(copy_type_id, "maxSpeed")
        max_speed = float(params) if params else 40.  # Default max speed
        
        # Add a new vehicle type with modified color and ID
        traci.vehicletype.copy(copy_type_id, new_type_id)

        # Set new vehicle type attributes
        traci.vehicletype.setWidth(new_type_id, width)
        traci.vehicletype.setLength(new_type_id, length)
        traci.vehicletype.setMaxSpeed(new_type_id, max_speed)
        
        traci.vehicletype.setMinGap(new_type_id, 2.0)
        traci.vehicletype.setMinGapLat(new_type_id, 0.1)

        traci.vehicletype.setColor(new_type_id, new_color)
        
        # Report
        print(f"  Added new vehicle type '{new_type_id}' with color {new_color}")
    
    def step_external_vehicles(self, exts:Dict[str,EXT]):
        """
        Step the XIL-portion of the microsimulation interface for a list of external vehicles.         
        Sets an external vehicle position and velocity in the network, and returns states of nearby vehicles.

        :param exts: dict of vehicle objects to place in the network
        """

        # Check that the external agents are found
        vehicle_ids = traci.vehicle.getIDList()

        for ext in exts.values():
            ego_id = ext.veh_id
            if ego_id not in vehicle_ids:
                raise ValueError(f'Cannot find external vehicle {ego_id}!')
            
            ### Set states of the ext vehicles
            ego_x = ext.states.longitude
            ego_y = ext.states.latitude
            ego_speed = ext.states.speed
            ego_angle = ext.states.heading
            ego_accel = ext.states.acceleration
            
            # MovetoXY
            # - If no suitable position is found within 100m of the road, function will error
            xy_edge_id = "" # Can use "" if not known
            xy_lane = -1 # Can use -1 if not known
            xy_angle = ego_angle * (180./pi) # Nautical units wrt +y going clockwise - Ego angle is measured wrt y but in radians
            xy_keep_route = int('000', 2)

            traci.vehicle.moveToXY(ego_id, xy_edge_id, xy_lane, ego_x, ego_y, xy_angle, xy_keep_route)
            traci.vehicle.setSpeed(ego_id, ego_speed)
            traci.vehicle.setAcceleration(ego_id, ego_accel, 1.)

            # Set ego V2V communications into simulation
            time_traj, pos_traj = ext.states.get_trajectory()

            self.comms.update_veh_comms(ego_id, time_traj, pos_traj)

            # Prevent internal vehicle model
            traci.vehicle.setSpeedFactor(ego_id, 1.0)

            traci.vehicle.setSpeedMode(ego_id, int('1100000', 2))
            traci.vehicle.setLaneChangeMode(ego_id, int('000000000000', 2))

            # Set ext color
            traci.vehicle.setColor(ego_id, self.colors['ext']) # RGBA: 0-255 RGB scale, 0-255 Alpha scale (ubyte)

            ### Get network information for the ext vehicles            
            ego_lane_id, is_lane_onramp, is_lane_ending = self.get_onramp_lane(ego_id)

            # Get the max travel distance to end of current lane - ego control constraints
            lane_end_rel_distance = self.get_end_of_lane_distance(ego_id, ego_lane_id)

            # Get the max speed of the current edge
            v_max = self.get_lane_speed_limit(ego_id, ego_lane_id)

            ## Get traffic intersection states
            # tl_states = self.get_routewise_upcoming_traffic_light_states(ego_id)

            # Get preceding vehicle states
            leader = traci.vehicle.getLeader(ego_id, dist=RADAR_RANGE)
            if leader is not None and leader[0] != "":
                lead_id = leader[0] # string of "flow id.veh id"
                
                lead_rel_distance = leader[1] # (Front bumper + min_gap) to (rear bumper)
                lead_speed = traci.vehicle.getSpeed(lead_id)
                lead_accel = traci.vehicle.getAcceleration(lead_id)
                lead_heading = traci.vehicle.getAngle(lead_id)
                lead_heading_rate = 0.
                lead_x, lead_y = traci.vehicle.getPosition(lead_id)
                lead_lat_gap = 0.

                lead_type = traci.vehicle.getTypeID(lead_id)

                # Calculate headway (time gap to the leader)
                headway = lead_rel_distance / ego_speed if ego_speed > 0 else 0
            
            else: # No leader, so choose default sensor values
                lead_id = "-1" # Default values
            
                lead_rel_distance = 2000. # Bumper to bumper gap [m]
                lead_speed = 0. # [m/s]
                lead_accel = 0. # [m/s2]
                lead_heading = 0. # [rad]
                lead_heading_rate = 0.
                lead_x = 0.
                lead_y = 0.
                lead_lat_gap = 0.
                
                lead_type = None
                
                # No leader, so headway is invalid
                headway = -1

            # Get preceding vehicle communications
            lead_t_comms, lead_s_comms, lead_v_comms = self.comms.get_veh_comms(lead_id, self.sim_time, distance_from_sv=lead_rel_distance)

            ## Pack neighboring vehicle information
            ext.reset_nv_states()

            if lead_type or lead_type != 'None':
                ext.update_nv_states(lead_id, lead_y, lead_x, lead_rel_distance, lead_lat_gap, lead_speed, lead_accel, lead_heading, lead_heading_rate)
            
                if lead_t_comms:
                    ext.nv_states[lead_id].set_trajectory(times=lead_t_comms, rel_long_pos=lead_s_comms)

            ### Print detailed ego vehicle information
            if abs(fmod(self.sim_time, 10)) < 1e-6: # Print output every n seconds of simulation time
                print(f"  Ext: {ego_id}, Leader: {lead_id}, Gap: {lead_rel_distance:.2f} m, Headway: {headway:.2f} s")

    def step_replay_vehicle(self, veh_id, veh_type, x, y, speed, heading, acceleration):
        """
        Step the microsimulation interface for a list of replay vehicles.
        Sets replay vehicle position, heading, and velocity in the network.
        """

        # Check that the replay agents are found
        vehicle_ids = traci.vehicle.getIDList()

        ego_id = veh_id
        # if ego_id not in vehicle_ids:
        #     print(vehicle_ids)
        #     raise ValueError(f'Cannot find replay vehicle {ego_id}!')
        
        ### Set states of the ext vehicles
        ego_x = x
        ego_y = y
        ego_speed = speed
        ego_angle = heading
        ego_accel = acceleration
        
        # MovetoXY
        # - If no suitable position is found within 100m of the road, function will error
        xy_edge_id = "" # Can use "" if not known
        xy_lane = -1 # Can use -1 if not known
        xy_angle = ego_angle # Nautical units wrt +y going clockwise - Ego angle is measured wrt y but in radians
        xy_keep_route = int('000', 2)

        traci.vehicle.moveToXY(ego_id, xy_edge_id, xy_lane, ego_x, ego_y, xy_angle, xy_keep_route)
        traci.vehicle.setSpeed(ego_id, ego_speed)
        traci.vehicle.setAcceleration(ego_id, ego_accel, 1.)

        # Prevent internal vehicle model
        traci.vehicle.setSpeedFactor(ego_id, 1.0)

        traci.vehicle.setSpeedMode(ego_id, int('1100000', 2))
        traci.vehicle.setLaneChangeMode(ego_id, int('000000000000', 2))

        # Set ext color
        color = self.colors[veh_type] # RGBA: 0-255 RGB scale, 0-255 Alpha scale (ubyte)
        
        traci.vehicle.setColor(ego_id, color)

    def get_onramp_lane(self, ego_id):
        """
        Checks for hardcoded, specific lanes in onramp and i24 scenario

        Returns lane_id in onramp scenario, otherwise ''
                is_lane_onramp Flag to indicate if lane is part of any lanes attached to onramp
                is_lane_ending Flag to indicate if lane is last in onramp and will end
        """

        lane_id = ''
        is_lane_onramp = False
        is_lane_ending = False

        if self.is_merging_scenario(): # TODO - Hardcoded these id values for now
            lane_id = traci.vehicle.getLaneID(ego_id)

            is_lane_onramp = ('onramp' in lane_id.lower()) or (lane_id in ONRAMP_LANES)
            is_lane_ending = (lane_id in I24_ENDING_LANES) if self.args.scenario == 'i24' else (lane_id in ONRAMP_ENDING_LANES)

        return lane_id, is_lane_onramp, is_lane_ending

    def get_end_of_lane_distance(self, ego_id, lane_id):
        """
        Checks for specific lanes to end and returns the position where the lane ends.

        Returns:
            s_max (float): Max distance until the lane ends
        """
        s_max = 5000. # Max allowed forward travel distance [m]: ego_distance(t) <= s_max \forall t

        ### Get road geometry information
        if lane_id == 'E2_0': # TODO - Hardcoded this id value for now - onramp scenario merging lane
            link_length = traci.lane.getLength(lane_id)
            link_distance = traci.vehicle.getLanePosition(ego_id)

            s_max = link_length - link_distance # s max at the remaining travel distance on current merging lane

        return s_max
    
    def get_lane_speed_limit(self, ego_id='', ego_lane_id=''):
        """
        TODO - Checks for speed limit of the current edge and returns the speed limit.

        Returns:
            v_max (float): Max speed
        """

        # Get the physical engine speed limit - important for trucks
        speed_engine = 60. # traci.vehicle.getMaxSpeed(ego_id)

        # Get the road speed limit
        speed_limit_default = 30. # Choose a default speed limit for now - how to get this edge information
        if not ego_lane_id:
            ego_lane_id = traci.vehicle.getLaneID(ego_id)

        speed_limit = traci.lane.getMaxSpeed(ego_lane_id) if ego_lane_id else speed_limit_default

        # Return max speed allowed for controller
        v_max = max(0, min(speed_engine, speed_limit)) # Max allowed travel speed [m/s]: ego_speed(t) <= v_max \forall t
        
        return v_max

    def get_route_aligned_lane_target(self, ego_id: str, next_tls, approach_distance: float = 400.0):
        """
        Return a strategically preferred lane on the current edge when a vehicle
        is approaching a signalized junction and its current lane does not match
        the remaining route.
        """

        if self.args.no_cav_lanechange or not next_tls:
            return None

        ego_lane_id = traci.vehicle.getLaneID(ego_id)
        if not ego_lane_id or self.is_on_junction(ego_lane_id):
            return None

        tls_distance = next_tls[0][2]
        if tls_distance < 0 or tls_distance > approach_distance:
            return None

        best_lanes = traci.vehicle.getBestLanes(ego_id)
        if not best_lanes:
            return None

        current_lane_index = traci.vehicle.getLaneIndex(ego_id)
        current_info = next((lane for lane in best_lanes if lane[0] == ego_lane_id), None)
        if current_info is None:
            return None

        valid_lanes = [lane for lane in best_lanes if lane[4]]
        if not valid_lanes:
            return None

        def lane_index(lane_id: str) -> int:
            try:
                return int(lane_id.rsplit("_", 1)[1])
            except (IndexError, ValueError):
                return current_lane_index

        target_info = min(
            valid_lanes,
            key=lambda lane: (
                abs(lane[3]),
                abs(lane_index(lane[0]) - current_lane_index),
                -lane[1],
            ),
        )

        current_offset = current_info[3]
        current_allows_continuation = bool(current_info[4])
        target_lane_id = target_info[0]
        target_lane_index = lane_index(target_lane_id)

        if current_allows_continuation and current_offset == 0:
            return None

        if target_lane_index == current_lane_index:
            return None

        lane_length = traci.lane.getLength(ego_lane_id)
        lane_pos = traci.vehicle.getLanePosition(ego_id)
        remaining_distance = max(lane_length - lane_pos, 0.0)
        lane_steps_needed = abs(target_lane_index - current_lane_index)

        # Do not issue hopeless late commands once the vehicle is already at the
        # stop line. Everywhere else, keep nudging toward the route-valid lane.
        if remaining_distance < 15.0 * max(lane_steps_needed, 1):
            return None

        return {
            "target_lane_id": target_lane_id,
            "target_lane_index": target_lane_index,
            "current_lane_id": ego_lane_id,
            "current_lane_index": current_lane_index,
            "lane_delta": target_lane_index - current_lane_index,
            "remaining_distance": remaining_distance,
            "tls_distance": tls_distance,
            "current_offset": current_offset,
            "current_allows_continuation": current_allows_continuation,
            "guidance_source": "best_lanes",
            "guidance_interval": 1.0,
        }

    def align_vehicle_to_route_lane(self, ego_id: str, next_tls):
        """
        Force a route-consistent lane choice for CAVs before short turn pockets
        so they do not reach the stop line in an invalid lane.
        """

        lane_target = self.get_route_aligned_lane_target(ego_id, next_tls)
        if lane_target is None:
            return

        lane_delta = lane_target["lane_delta"]
        if lane_delta == 0:
            return

        # Nudge only one lane at a time and avoid issuing a hard command every
        # 0.1s. This is more stable than repeatedly forcing an absolute target
        # lane on crowded urban approaches.
        last_guidance_time = self.last_route_lane_guidance.get(ego_id, -1e9)
        guidance_interval = lane_target.get("guidance_interval", 1.0)
        if self.sim_time - last_guidance_time < guidance_interval:
            return

        min_remaining_distance = lane_target.get("min_remaining_distance", 25.0)
        if lane_target["remaining_distance"] < min_remaining_distance:
            return

        lane_step = 1 if lane_delta > 0 else -1

        try:
            traci.vehicle.changeLaneRelative(ego_id, lane_step, 2.0)
            self.last_route_lane_guidance[ego_id] = self.sim_time
            self.route_lane_guidance_debug.append({
                "sim_time": round(self.sim_time, 1),
                "ego_id": ego_id,
                "guidance_source": lane_target.get("guidance_source"),
                "current_lane_id": lane_target["current_lane_id"],
                "target_lane_id": lane_target["target_lane_id"],
                "current_lane_index": lane_target["current_lane_index"],
                "target_lane_index": lane_target["target_lane_index"],
                "lane_step": lane_step,
                "remaining_distance": round(lane_target["remaining_distance"], 2),
                "tls_distance": round(lane_target["tls_distance"], 2),
            })
            if len(self.route_lane_guidance_debug) > 25:
                self.route_lane_guidance_debug = self.route_lane_guidance_debug[-25:]
        except Exception:
            return
    
    def is_on_junction(self, ego_lane_id:str):
        '''
        Checks if the ego is currently on a junction.
        '''

        return ego_lane_id.startswith(":")

    def rescue_wrong_lane_at_stopline(self, ego_id: str) -> bool:
        """
        If a CAV has entered a short turn-pocket lane (< 20 m) that has no
        valid connection to its route's next edge, immediately teleport it
        laterally to the best route-valid lane at the same position.

        This handles the Clinton intersection trap: edge 435834768#0 is only
        9.95 m long and lane 0 is right-turn-only.  CAVs routed straight or
        left that end up on lane 0 have no valid forward connection and will
        deadlock.  Moving them to a valid lane before the controller runs
        prevents the deadlock entirely.

        Returns True if a rescue teleport was performed.
        """
        lane_id = traci.vehicle.getLaneID(ego_id)
        if not lane_id or self.is_on_junction(lane_id):
            return False

        lane_length = traci.lane.getLength(lane_id)
        lane_pos = traci.vehicle.getLanePosition(ego_id)
        # Only rescue when close enough to the stop line that a normal lane
        # change is no longer possible.  Vehicles with > 15 m of approach
        # remaining are handled by align_vehicle_to_route_lane instead.
        if (lane_length - lane_pos) > 15.0:
            return False

        best_lanes = traci.vehicle.getBestLanes(ego_id)
        if not best_lanes:
            return False

        current_info = next((l for l in best_lanes if l[0] == lane_id), None)
        if current_info is None or current_info[4]:
            # Already on a route-valid lane — nothing to do.
            return False

        current_edge = lane_id.rsplit('_', 1)[0]
        valid_on_edge = [
            l for l in best_lanes
            if l[4] and l[0].rsplit('_', 1)[0] == current_edge
        ]
        if not valid_on_edge:
            return False

        # Prefer the lane with the smallest required lane-change offset.
        target_lane = min(valid_on_edge, key=lambda l: abs(l[3]))
        target_lane_id = target_lane[0]
        if target_lane_id == lane_id:
            return False

        safe_pos = min(lane_pos, lane_length - 0.1)
        try:
            traci.vehicle.moveTo(ego_id, target_lane_id, safe_pos)
            return True
        except Exception:
            return False
    
    def get_numb(self, var: str, default: float | int = 0):
        txt = var.strip()
        numbs = txt.replace(",", " ").replace(";", " ").split()
        arr = [int(p) for p in numbs]
        
        return arr

    def obtain_all_tl_ctrl_data(self):

        tl_id_list = traci.trafficlight.getIDList()    

        tl_logic = {}
        for tl_id in tl_id_list:        
            current_prog = traci.trafficlight.getProgram(tl_id)    
            defs = traci.trafficlight.getAllProgramLogics(tl_id)
            tldef_sel = next((d for d in defs if d.programID == current_prog), defs[0])
            tl_logic[tl_id] = tldef_sel
        
        TLInfo = {}
        for tlID in tl_id_list:
            TLInfo[tlID] = {}
            TLInfo[tlID]["timingTable"] = [None]
            TLInfo[tlID]["SUMOStateTable"] = [None]
            TLInfo[tlID]["indUnqLanes"] = [None]
            TLInfo[tlID]["rings"] = [None]
            TLInfo[tlID]["barriers"] = [None]

        for indtl, tlID in enumerate(tl_id_list):
            # tlID = "Delano"
            tl_logic_s = tl_logic[tlID]
            phases = tl_logic_s.phases
            subParameters = tl_logic_s.subParameter

            toff = subParameters.get("offset")
            tcyc = subParameters.get("total-cycle-length")
            ring1 = subParameters.get("ring1")
            ring2 = subParameters.get("ring2")
            barrier2Phases = subParameters.get("barrier2Phases")
            barrierPhases = subParameters.get("barrierPhases")

            ring1 = self.get_numb(ring1, 0)
            ring2 = self.get_numb(ring2, 0)
            barrier2Phases = self.get_numb(barrier2Phases, 0)
            barrierPhases = self.get_numb(barrierPhases, 0)    

            rings = [ring1, ring2]
            barriers = [barrier2Phases, barrierPhases]

            nPhase = 8
            tVehExt_arr = [0]*nPhase
            tY_arr = [0]*nPhase
            tRC_arr = [0]*nPhase

            # Manually coded values for veh ext, yellow, red clearance 
            if tlID == "Canal":
                tY_arr = [3]*nPhase
                for k in range(1,9,2):
                    tRC_arr[k] = 2
            elif tlID == "Clark":
                tY_arr = [3]*nPhase
                for k in range(1,9,2):
                    tRC_arr[k] = 2
            elif tlID == "Clinton":
                tY_arr = [3]*nPhase
                for k in range(1,9,2):
                    tRC_arr[k] = 2
            elif tlID == "Delano":
                tVehExt_arr[5] = 5
                for k in [1,3,5,6,7]:
                    tY_arr[k] = 3
                for k in range(1,9,2):
                    tRC_arr[k] = 2
            elif tlID == "Michigan":
                for k in [0,4,6]:
                    tVehExt_arr[k] = 3
                tY_arr = [3]*nPhase    
                for k in range(1,9,2):
                    tRC_arr[k] = 2
                for k in range(0,8,2):
                    tRC_arr[k] = 1
            elif tlID == "State":
                tVehExt_arr[6] = 3
                tY_arr = [3]*nPhase
                for k in range(1,9,2):
                    tRC_arr[k] = 2
            elif tlID == "Wabash":
                tY_arr = [3]*nPhase
                for k in range(1,9,2):
                    tRC_arr[k] = 2
            elif tlID == "J1":
                tY_arr = [3]*nPhase
                for k in range(1,9,2):
                    tRC_arr[k] = 2
            else:
                raise ValueError(f'Intersection {tlID} not recognized!')

            # name, tcyc, toffset, duration, minDur, maxDur, vehExt, yellow, red
            timingTable = [[0]*9 for _ in range(8)]
            for r in range(8):
                timingTable[r][0] = r + 1
                timingTable[r][1] = float(tcyc)
                timingTable[r][2] = float(toff)

            for ph in phases: 
                name = ph.name
                duration = ph.duration
                minDur = ph.minDur
                maxDur = ph.maxDur
                r = int(name.strip())-1
                timingTable[r][3] = float(duration)
                timingTable[r][4] = float(minDur)
                timingTable[r][5] = float(maxDur)
                timingTable[r][6] = tVehExt_arr[r]
                timingTable[r][7] = tY_arr[r]
                timingTable[r][8] = tRC_arr[r]


            SUMOStateTable = [None] * 8
            nl = 0
            for ph in phases: 
                state = ph.state
                name = ph.name
                r = int(name.strip())-1
                SUMOStateTable[r] = state
                if state:
                    nl = max(nl, len(state))

            # find unique lanes for each phase
            indUnqLanes = np.zeros((8, nl), dtype=int)
            for i in range(8):
                phT = SUMOStateTable[i]
                if phT:
                    cond = np.array([c == 'G' for c in phT], dtype=bool)
                    for j in range(8):
                        if j != i:
                            phC = SUMOStateTable[j]
                            if phC: 
                                cond &= np.array([c != 'G' for c in phC], dtype=bool)
                    indUnqLanes[i,:] = cond.astype(int)

            # TLInfo["timingTable"][indtl] = timingTable
            # TLInfo["SUMOStateTable"][indtl] = SUMOStateTable
            # TLInfo["indUnqLanes"][indtl] = indUnqLanes
            # TLInfo["rings"][indtl] = rings
            # TLInfo["barriers"][indtl] = barriers
        
            TLInfo[tlID]["timingTable"] = timingTable
            TLInfo[tlID]["SUMOStateTable"] = SUMOStateTable
            TLInfo[tlID]["indUnqLanes"] = indUnqLanes.tolist()
            TLInfo[tlID]["rings"] = rings
            TLInfo[tlID]["barriers"] = barriers

        return TLInfo

    def extract_pre_timed_spat_from_states(self):

        SPATmsg = {}

        if not self.is_traffic_light_scenario():
            return SPATmsg

        time = self.sim_time # current time
        ts = self.dt # sampling time
        TLInfo = self.TLInfo
        SPATmsgPrv = self.SPATmsgPrev

        tlIDList = list(TLInfo.keys()) 
        Ntl = len(tlIDList)

        RYGStates = [0]*Ntl
        for idx, tlID in enumerate(tlIDList):
            RYGStates[idx] = traci.trafficlight.getRedYellowGreenState(tlID)

        # for each SPAT
        # state, startTime, minEndTime, maxEndTime, nextTime, time, phase activation
        SPATformat = [["", None, None, None, None, None, 0] for _ in range(8)]

        # SPAT msg initialization for the first time
        if not SPATmsgPrv:  
            SPATmsgPrv = {}    
            for tlID in tlIDList:
                SPATmsgPrv[tlID] = deepcopy(SPATformat)

        for idx, tlID in enumerate(tlIDList):
            # idx = 4
            # tlID = tlIDList[idx]
            RYGStates_s = RYGStates[idx]
            timingTable_s = TLInfo[tlID]["timingTable"]
            SUMOStateTable_s = TLInfo[tlID]["SUMOStateTable"]
            indUnqLanes_s = TLInfo[tlID]["indUnqLanes"]
            rings_s = TLInfo[tlID]["rings"]
            barriers_s = TLInfo[tlID]["barriers"]

            tcyc = timingTable_s[0][1]
            toff = timingTable_s[0][2]

            SPATPrv = SPATmsgPrv[tlID]
            cStatesPhasesPrv = [row[0] for row in SPATPrv]
            phaseActivation = [row[6] for row in SPATPrv]

            if not any(phaseActivation):
                idxPhasesPrv = []
                cStatesPrv = ""
            else:
                idxPhasesPrv = [i for i, pa in enumerate(phaseActivation) if pa == 1]
                cStatesPrv_list = [cStatesPhasesPrv[i] for i in idxPhasesPrv]
                cStatesPrv_list = ["r" if s == "rc" else s for s in cStatesPrv_list]
                cStatesPrv = "".join(cStatesPrv_list) # e.g., 'rG'
                
            # generate new SPAT based on previous SPAT    
            SPAT = [row[:] for row in SPATPrv]

            # find states for each phase using "indUnqLanes"
            statesPhases = []
            for k, indUnqLane in enumerate(indUnqLanes_s):
                statePhase = [RYGStates_s[i] for (i, iln) in enumerate(indUnqLane) if iln == 1]
                if statePhase:
                    statePhaseNong = [sp for sp in statePhase if sp != "g"]        
                    statesPhases.append(statePhaseNong[0])
                else:
                    statesPhases.append("r")

            # find active two phases using current states per phase
            idxPhases = [i for i, sp in enumerate(statesPhases) if sp != "r"]    
            
            # in case meaningless phase exists (e.g., phase 2 at Delano)
            # check if new phase compard to previous phases
            if len(idxPhases) == 1:
                one = idxPhases[0]
                if not idxPhasesPrv:
                    cond = True
                else: 
                    if one != idxPhasesPrv[0] and one != idxPhasesPrv[1]:
                        cond = True
                    else:
                        cond = False

                if cond:
                    r1, r2 = rings_s[0], rings_s[1]
                    if one in [p-1 for p in r1]:
                        c = [p-1 for p in r1].index(one) 
                        other = r2[c]-1
                        idxPhases = [one, other]
                    elif one in [p-1 for p in r2]:
                        c = [p-1 for p in r2].index(one) 
                        other = r1[c]-1
                        idxPhases = [other, one]                
                    
            # red clearance in the same phases
            if len(idxPhases) < 2:    
                idxPhases = idxPhasesPrv

            if idxPhases: 
                cStates = "".join(statesPhases[i] for i in idxPhases) # e.g., 'rG'
                if cStatesPrv: 
                    if idxPhases != idxPhasesPrv: # phase change case 
                        idxUpds = [i for i in range(len(idxPhases)) if idxPhases[i] != idxPhasesPrv[i]]
                        for iUpd in idxUpds: 
                            # old phase switching to 'r'
                            idxPhasesPrv_s = idxPhasesPrv[iUpd]
                            Dur = timingTable_s[idxPhasesPrv_s][3]
                            minEndTime = time + tcyc - Dur
                            maxEndTime = minEndTime

                            SPAT[idxPhasesPrv_s][0] = 'r'
                            SPAT[idxPhasesPrv_s][1] = time
                            SPAT[idxPhasesPrv_s][2] = minEndTime
                            SPAT[idxPhasesPrv_s][3] = maxEndTime
                            SPAT[idxPhasesPrv_s][4] = time + tcyc

                            # new phase switching to 'g'                
                            idxPhases_s = idxPhases[iUpd]
                            minDur = timingTable_s[idxPhases_s][4]
                            maxDur = timingTable_s[idxPhases_s][5]
                            cState = cStates[iUpd]

                            SPAT[idxPhases_s][0] = cState
                            SPAT[idxPhases_s][1] = time
                            SPAT[idxPhases_s][2] = time + minDur
                            SPAT[idxPhases_s][3] = time + maxDur
                            SPAT[idxPhases_s][4] = time + tcyc

                    else: # no phase change
                        if cStates != cStatesPrv: # status change case (e.g., g -> y -> rc)
                            idxUpds = [i for i in range(len(cStates)) if cStates[i] != cStatesPrv[i]]
                            for iUpd in idxUpds: 
                                idxPhases_s = idxPhases[iUpd]                            
                                tYllw = timingTable_s[idxPhases_s][7]  # yellow
                                tRC = timingTable_s[idxPhases_s][8]    # red-clear
                                cState = cStates[iUpd]
                                if cState != 'y':
                                    SPAT[idxPhases_s][0] = 'rc'
                                else:
                                    SPAT[idxPhases_s][0] = cState
                                SPAT[idxPhases_s][1] = time
                                if cState == 'y':                        
                                    SPAT[idxPhases_s][2] = time + tYllw
                                    SPAT[idxPhases_s][3] = time + tYllw
                                else:   
                                    SPAT[idxPhases_s][2] = time + tRC
                                    SPAT[idxPhases_s][3] = time + tRC
                                SPAT[idxPhases_s][4] = time + tcyc
                else:        
                    SPAT[idxPhases[0]][0] = cStates[0]
                    SPAT[idxPhases[1]][0] = cStates[1]
            for r in range(8):
                SPAT[r][5] = time
                SPAT[r][6] = 0
            for r in idxPhases: 
                SPAT[r][6] = 1

            SPATmsg[tlID] = SPAT   
            
        self.SPATmsgPrev = SPATmsg

        return SPATmsg

    def step(self):
        '''Run simulation step of currently connected SUMO network'''
        ### Run simulation step
        traci.simulationStep()

        ### Process microsimulation data
        ## Get simulation time
        self.sim_time = traci.simulation.getTime()

        ## Get vehicle data IDs
        vehicle_ids = traci.vehicle.getIDList()

        # Add new vehicles to the registry
        for id in vehicle_ids:
            if id not in self.spawned_vehs:
                vehicle_type = traci.vehicle.getTypeID(id)
                self.spawned_vehs[id] = vehicle_type

        # Remove vehicles that have left the network
        vehicle_id_set = set(vehicle_ids)
        for id in list(self.spawned_vehs.keys()):
            if id not in vehicle_id_set:
                del self.spawned_vehs[id]
                self.has_been_set_vehs.discard(id)
                self.cav_controllers.pop(id, None)
                self.last_route_lane_guidance.pop(id, None)
                self.cav_stuck_ticks.pop(id, None)
                self.cav_suspended_until.pop(id, None)
                self.comms.remove_veh_comms(id)

        step_spat = {}
        if self.is_traffic_light_scenario() and not self.args.replay:
            step_spat = self.extract_pre_timed_spat_from_states()

        # Two-stage control loop:
        # 1. Snapshot all inputs and compute commands from the previous-step comms.
        # 2. Apply lane/acceleration commands and then publish new comms.
        control_plan = {}
        comm_updates = {}
        cav_ids = sorted(
            ego_id for ego_id, vtype in self.spawned_vehs.items()
            if vtype == 'cav' and ego_id in vehicle_id_set
        )

        for ego_id in cav_ids:
            ego_lane_id, is_lane_onramp, is_lane_ending = self.get_onramp_lane(ego_id)

            if self.args.no_cav_onramp and is_lane_onramp:
                control_plan[ego_id] = {
                    "skip_onramp": True,
                }
                continue

            # Suspended CAVs are under SUMO control until their suspension
            # period expires.  Skip all TraCI commands for them.
            if ego_id in self.cav_suspended_until:
                if self.sim_time < self.cav_suspended_until[ego_id]:
                    continue
                else:
                    del self.cav_suspended_until[ego_id]

            # Fix wrong-lane trap on short turn-pocket edges before computing
            # the control command.  This must happen before getNextTLS so that
            # remDistTLS and best-lane data reflect the corrected position.
            if self.is_traffic_light_scenario() and not self.args.replay:
                self.rescue_wrong_lane_at_stopline(ego_id)

            lane_end_rel_distance = self.get_end_of_lane_distance(ego_id, ego_lane_id)
            v_max = self.get_lane_speed_limit(ego_id, ego_lane_id)

            if self.is_traffic_light_scenario() and not self.args.replay:
                next_tls = traci.vehicle.getNextTLS(ego_id)
            else:
                next_tls = []

            ego_distance = traci.vehicle.getDistance(ego_id)
            ego_speed = traci.vehicle.getSpeed(ego_id)
            ego_accel = traci.vehicle.getAcceleration(ego_id)
            s_max = lane_end_rel_distance + ego_distance

            leader = traci.vehicle.getLeader(ego_id, dist=RADAR_RANGE)
            if leader is not None and leader[0] != "":
                lead_id = leader[0]
                lead_rel_distance = leader[1]
                lead_speed = traci.vehicle.getSpeed(lead_id)
                lead_accel = traci.vehicle.getAcceleration(lead_id)
            else:
                lead_id = "-1"
                lead_rel_distance = 2000.
                lead_speed = 0.
                lead_accel = 0.

            lead_t_comms, lead_s_comms, lead_v_comms = self.comms.get_veh_comms(
                lead_id,
                self.sim_time,
                distance_from_sv=lead_rel_distance,
            )

            controller = self.get_controller_for_vehicle(ego_id)
            desired_acceleration, time_traj, pos_traj, vel_traj, acc_traj = controller.getCommand(
                self.sim_time,
                ego_accel,
                ego_speed,
                ego_distance,
                s_max,
                v_max,
                lead_accel,
                lead_speed,
                lead_rel_distance,
                lead_t_comms,
                lead_s_comms,
                lead_v_comms,
                NextTLS=next_tls,
                SPATmsg=step_spat,
                TLInfo=self.TLInfo,
            )

            control_plan[ego_id] = {
                "skip_onramp": False,
                "desired_acceleration": desired_acceleration,
                "next_tls": next_tls,
                "controller": controller,
            }
            comm_updates[ego_id] = (time_traj, pos_traj, vel_traj)

        for ego_id in cav_ids:
            # Skip vehicles currently suspended (under SUMO control for teleport)
            if ego_id in self.cav_suspended_until and self.sim_time < self.cav_suspended_until[ego_id]:
                continue

            plan = control_plan.get(ego_id)
            if plan is None:
                continue

            if plan["skip_onramp"]:
                self.has_been_set_vehs.discard(ego_id)
                traci.vehicle.setColor(ego_id, self.colors['ramp'])
                continue

            if ego_id not in self.has_been_set_vehs:
                # Prevent internal vehicle model - set one-time parameters
                traci.vehicle.setSpeedFactor(ego_id, 1.0)
                traci.vehicle.setSpeedMode(ego_id, int('1100000', 2))

                if self.args.no_cav_lanechange:
                    traci.vehicle.setLaneChangeMode(ego_id, int('000000000000', 2))
                else:
                    traci.vehicle.setLaneChangeMode(ego_id, int('011001010101', 2))

                traci.vehicle.setColor(ego_id, self.colors['cav'])
                self.has_been_set_vehs.add(ego_id)

            if self.is_traffic_light_scenario() and not self.args.replay:
                self.align_vehicle_to_route_lane(ego_id, plan["next_tls"])

            traci.vehicle.setAcceleration(ego_id, plan["desired_acceleration"], self.acc_duration)
            self.debug(ego_id=ego_id, controller=plan["controller"])

            # --- Stuck-at-stop-line rescue ---
            # If a CAV's actual speed stays near zero while it is at the end of
            # an approach lane for too long, suspend TraCI speed control and let
            # SUMO's own model (and teleport mechanism) recover it.
            # The suspension duration (150 s) covers 2-3 full TL cycles so SUMO
            # can accumulate the 60 s of green-phase stuck time needed to
            # trigger a teleport, even on approaches with long red phases.
            if self.is_traffic_light_scenario() and not self.args.replay:
                actual_speed = traci.vehicle.getSpeed(ego_id)
                rescue_lane_id = traci.vehicle.getLaneID(ego_id)
                if not self.is_on_junction(rescue_lane_id):
                    rescue_lane_len = traci.lane.getLength(rescue_lane_id)
                    rescue_lane_pos = traci.vehicle.getLanePosition(ego_id)
                    at_stop_line = (rescue_lane_len - rescue_lane_pos) < 1.5
                    if at_stop_line and actual_speed < 0.05:
                        self.cav_stuck_ticks[ego_id] = self.cav_stuck_ticks.get(ego_id, 0) + 1
                    else:
                        self.cav_stuck_ticks.pop(ego_id, None)

                    # 100 ticks × 0.1 s/tick = 10 s stuck at stop line → suspend
                    if self.cav_stuck_ticks.get(ego_id, 0) >= 100:
                        traci.vehicle.setSpeed(ego_id, -1)
                        traci.vehicle.setSpeedMode(ego_id, 31)  # restore all SUMO safety checks
                        self.has_been_set_vehs.discard(ego_id)
                        self.cav_stuck_ticks.pop(ego_id, None)
                        self.cav_suspended_until[ego_id] = self.sim_time + 150.0

        for ego_id, (time_traj, pos_traj, vel_traj) in comm_updates.items():
            self.comms.update_veh_comms(ego_id, time_traj, pos_traj, vel_traj)

        ### Print output
        if self.sim_time + 1e-9 >= self.next_summary_time:
            self.print_runtime_summary()
            self.next_summary_time += self.summary_interval

        ### Custom logging
        self.log()

    def rate(self):
        """
        Sleep program until next time to execute another frame if running with real-time option.
        """
        if self.args.realtime: # Only if --realtime option selected
            # Calculate Sleep duration and sleep
            self.rt_iter += 1
            
            sleep_duration = self.dt*self.rt_iter - (counter()-self.rt_start_time)
            if sleep_duration > 0:
                sleep(sleep_duration) # Control broadcast update rate

            # Error check
            if sleep_duration < 1e-3:
                warnings.warn("Broadcast update too slow to maintain desired update rate")

    def debug(self, ego_id = '', controller=None):
        ''' If in debug mode, print the ego vehicle controller data from its _repr_ method.'''
        if self.args.debug:
            if 'east_cav.8' in ego_id or 'ramp_0_cav.2' in ego_id:
                if controller is not None:
                    print(controller)
                elif hasattr(self, "ego"):
                    print(self.ego)

    def log(self):
        ''' If in log TL mode, log detailed vehicle and simulation data for analysis and visualization after the simulation is complete '''
        if self.args.log_tl:
            if self.is_traffic_light_scenario():
                vehicle_ids = traci.vehicle.getIDList()

                # Log vehicle and simulation data
                type_gen = (ego_id for ego_id in vehicle_ids)
                for ego_id in type_gen:
                    leader = traci.vehicle.getLeader(ego_id, dist=RADAR_RANGE)

                    if leader is not None and leader[0] != "":
                        lead_id = leader[0] # string of "flow id.veh id"
                        lead_rel_distance = leader[1] # (Front bumper + min_gap) to (rear bumper)
                        lead_speed = traci.vehicle.getSpeed(lead_id)
                        lead_accel = traci.vehicle.getAcceleration(lead_id)

                        lead_type = traci.vehicle.getTypeID(lead_id)

                    else:
                        lead_id = "-1" # Default values

                        lead_rel_distance = 2000. # Bumper to bumper gap [m]
                        lead_speed = 0. # [m/s]
                        lead_accel = 0. # [m/s2]
                        
                        lead_type = 'None'
                        
                    ego_distance = traci.vehicle.getDistance(ego_id) # From starting point - TODO behavior if multiple starting points?
                    ego_speed = traci.vehicle.getSpeed(ego_id)
                    ego_accel = traci.vehicle.getAcceleration(ego_id)
                    ego_lane_id, is_lane_onramp, is_lane_ending = self.get_onramp_lane(ego_id)

                    v_max = self.get_lane_speed_limit(ego_id, ego_lane_id)

                    ego_lane_index = traci.vehicle.getLaneIndex(ego_id)

                    # Log data
                    self.veh_log.append({
                        "t": self.sim_time,
                        "vehID": ego_id,
                        "laneID": ego_lane_index,
                        "s": ego_distance,
                        "v": ego_speed,
                        "a": ego_accel,
                        "PVID": lead_id,
                        "gap": lead_rel_distance,
                        "vps": lead_speed,
                        "aps": lead_accel,
                        "vmax": v_max,
                    })
                
                # Log traffic light status
                tlIDList = list(self.TLInfo.keys()) 
                for tlID in tlIDList: 
                    RYGStates = traci.trafficlight.getRedYellowGreenState(tlID)    
                    phase = traci.trafficlight.getPhase(tlID)    
                    nextSwitch  = traci.trafficlight.getNextSwitch(tlID)   
                    arrNemaPhase  = traci.trafficlight.getNemaPhaseCalls(tlID)            
                    NemaPhase = "".join(arrNemaPhase)
                    
                    self.sim_log.append({
                                "t": self.sim_time,
                                "tlID": tlID,
                                "RYGStates": RYGStates,
                                "phase": phase,
                                "NemaPhase": NemaPhase,
                                "nextSwitch": nextSwitch,                    
                            })

    def is_running(self, lower_bound=0):
        """
        Check if there are simulated agents still in the network that are active.
        Terminate when the number of agents left in the network are less than the specified lower bound,
        or when max_sim_time is exceeded (if set).
        """
        max_t = getattr(self.args, 'max_sim_time', 0) or 0
        if max_t > 0 and self.sim_time >= max_t:
            print(f"[SIM] max_sim_time={max_t:g}s reached at t={self.sim_time:.1f}s — terminating.")
            return False
        return traci.simulation.getMinExpectedNumber() > lower_bound # While there are active vehicles/pedestrians in network

    def sim(self):
        """
        Run a simulation of the SUMO network. Checks to run SUMO simulation to completion as defined in is_running().
        """
        try:
            # Scenarios with traffic lights specifically
            ## TODO We are hard-coding the pre-loading of traffic light timings, so CAVs are not run for the first n steps
            if self.is_traffic_light_scenario():
                n_pre_steps = 3500

                print(f"Pre-loading traffic light scenario and acquiring traffic light data for {n_pre_steps} simulation steps.")

                # Get TL info
                self.TLInfo = self.obtain_all_tl_ctrl_data()
                
                # Run n steps of simulation to get SPAT messages
                for i in range(n_pre_steps):
                    traci.simulationStep()

                    SPATmsg = self.extract_pre_timed_spat_from_states()
                    self.SPATmsgPrev = SPATmsg

            # Run SUMO mixed traffic simulation
            print("Starting SUMO simulation.")
            self.wall_start_time = counter()
            self.next_summary_time = 0.0
        
            while self.is_running():
                self.step()
                
                # Rate limit if using the --realtime command line argument
                self.rate()

        except KeyboardInterrupt:
            pass

        except Exception as e:
            print(f'Unexpected simulation error {str(e)}')
            if self.route_lane_guidance_debug:
                print('Recent route lane guidance commands before failure:')
                for item in self.route_lane_guidance_debug[-10:]:
                    print(item)
            raise

        finally:
            # Cleanup
            self.stop()

    def replay_sim(self):
        ''' 
        Runs the main simulation loop within the XIL interface.
        - Adds external vehicle types to the microsimulation.
        - Waits for necessary client connections before proceeding.
        - Iterates through simulation steps, updating external vehicles and publishing data.
        - Ensures consistency between microsimulation step size and communication framerate.
        - Manages error handling, capturing exceptions related to simulation runtime failures.
        '''
        # try:
        # Create a new external vehicle type
        template_type='hdv'
        self.add_external_vehicle_type(template_type, EXT().type_id, self.colors['ext'])

        ### Simulation
        t_start = counter()

        tree = ET.parse(self.replay_fcd_path)
        root = tree.getroot()

        seen_vehicles = set()

        for timestep in root.iter('timestep'):
            sim_time = float(timestep.get('time'))
    
            ### Step microsim
            self.step()

            vehicle_ids = traci.vehicle.getIDList()

            for vehicle in timestep.iter('vehicle'):
                veh_id = vehicle.get('id')
                veh_type = vehicle.get('type')
                x = float(vehicle.get('x'))
                y = float(vehicle.get('y'))
                speed = float(vehicle.get('speed'))
                accel = float(vehicle.get('acceleration'))
                angle = float(vehicle.get('angle'))

                # Inject vehicle only if it's new
                if veh_id not in seen_vehicles and veh_id not in vehicle_ids and veh_type == EXT().type_id:
                    if 'east' in veh_id:
                        route_id = "r_east"
                    elif 'west' in veh_id:
                        route_id = "r_west"
                    else:
                        route_id = "mainlane"

                    self.init_vehicle(veh_id, route_id=route_id, type_id=veh_type)

                    # Set the camera if an external vehicle detected
                    if veh_type == EXT().type_id:
                        self.set_camera(veh_id, 650)

                ### Step the replay of microsim
                self.step_replay_vehicle(veh_id, veh_type, x, y, speed, angle, accel)

                seen_vehicles.add(veh_id)
            
            # Iterate
            if abs(fmod(self.sim_time, 1)) < 1e-6: # Print output every n seconds of simulation time
                print(f'real time={(counter()-t_start):0.2f}, sim time={self.sim_time:0.2f}')

            ### Rollover and control runtime rate
            self.rate()

        print('Microsim shutdown automatically.')

        # except KeyboardInterrupt:
        #     pass

        # except Exception as e:
        #     print(f'Unexpected simulation error {str(e)}')

        # finally:
            # Cleanup
        self.stop()

# -------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    # Run simulation
    sumo = simulation()

    if sumo.args.replay:
        sumo.replay_sim()

    else:
        sumo.sim()
