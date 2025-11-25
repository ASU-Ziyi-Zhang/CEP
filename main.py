#!/usr/bin/env python3

#######################################
### Tyler Ard                       ###
### Yaozhong Zhang                  ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

# Profiling helpers
#   python -m cProfile -o <output_name>.prof <script_name>.py
#   snakeviz <output_name>.prof
#
# Quick-start and API references (SUMO / TraCI):
#   https://sumo.dlr.de/docs/Tutorials/Hello_SUMO.html
#
# Running microsimulations
#   https://sumo.dlr.de/docs/sumo.html
#   https://sumo.dlr.de/docs/TraCI.html
#   https://sumo.dlr.de/docs/Libsumo.html
#
# Selected TraCI docs
#   https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html
#   https://sumo.dlr.de/docs/TraCI/Traffic_Lights_Value_Retrieval.html
#   https://sumo.dlr.de/docs/Definition_of_Vehicles%2C_Vehicle_Types%2C_and_Routes.html
#   https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html
#
# More detailed documentation
#   https://sumo.dlr.de/pydoc/traci._edge.html
#
# Modeling
#   https://sumo.dlr.de/docs/Simulation/SublaneModel.html
#   https://sumo.dlr.de/docs/Simulation/Safety.html
#
# Outputting data
#   https://sumo.dlr.de/docs/Tools/Xml.html#xml2csvpy
#
# Creating networks
#   https://sumo.dlr.de/docs/netconvert.html
#   https://sumo.dlr.de/docs/Netedit/index.html

'''
Changelog (English-only comments version):

YW [02/14/2025]
1) Add "CAV penetration rate" configuration function in sumoAPI.
2) Add scenario configuration in sumoAPI.

YW [03/06/2025]
1) Switch to TraCI for broader functionality (compared with libsumo).
2) Update FCD output to include leader ID, leader gap, and headway to leader.

ZY [10/17/2025]
1) Add a triple‑guard stop in `sim()`: (i) natural completion when `getMinExpectedNumber()==0`,
   (ii) optional wall‑clock guard via `max_wall_time`, and (iii) simulation‑time guard via
   `max_sim_time_guard` (falls back to ~2× inferred planned end if available).

'''

import time
import numpy as np
from time import perf_counter as counter
from time import sleep
import warnings
import os
import argparse
from typing import List, Dict
from math import fmod, pi
import random
from datetime import datetime
import xml.etree.ElementTree as ET

from sumolib import checkBinary
import traci # traci has full API options
# import libsumo as traci  # libsumo is faster but has no GUI support and certain API options are limited

from src.agents import PCC, CAV, EXT
from src.sensing import Communications
from src.settings import *
import parsers.sumo
from src.logging import logger

import scripts.utils_data_read as reader

# -------------------------------------------------------------------------------------------------------

class simulation():
    """
    High-level wrapper around the SUMO microsimulation tool.

    Responsibilities
    ----------------
    - Parse command-line arguments and construct the SUMO command.
    - Start/stop the SUMO instance via TraCI/libsumo.
    - Manage vehicle bookkeeping and one-time vehicle property setup.
    - Integrate a virtual CAV controller (PCC/CAV) and V2V communication stubs.
    - Support XIL-style "external" vehicles and "replay" vehicles.
    - Provide utilities for lane/speed-limit queries and traffic-light state lookups.

    Notes
    -----
    * This implementation imports `libsumo as traci` for performance (no GUI).
      If GUI is requested, you must import/use standard `traci` and launch `sumo-gui`.
    * Only comments/docstrings were modified to English; the program logic remains unchanged.
    """

    def zero_route_flows(self, route_file, output_file):
        """
        Set all `<flow>` elements' `vehsPerHour` attribute to zero in a route XML file
        and write the result to `output_file`.

        Parameters
        ----------
        route_file : str
            Input route file path (.rou.xml).
        output_file : str
            Output file path to write the modified routes.
        """
        tree = ET.parse(route_file)
        root = tree.getroot()

        for flow in root.findall('flow'):
            if 'vehsPerHour' in flow.attrib:
                flow.set('vehsPerHour', '0')

        tree.write(output_file, encoding='UTF-8', xml_declaration=True)

    def __init__(self, args=[]):
        """
        Initialize the simulation wrapper and launch a SUMO/TraCI session.

        Parameters
        ----------
        args : argparse.Namespace or list, optional
            Parsed CLI arguments. If empty, an ArgumentParser will be created
            and `parsers.sumo.register_parser(parser)` will be used to register
            options, then `parser.parse_args()` will be called.

        Side Effects
        ------------
        - Validates penetration argument within [0, 1].
        - Creates an output directory (optionally timestamped).
        - Builds and executes the SUMO command via `traci.start`.
        - Instantiates communication buffers and a logger.
        """
        # Handle input arguments
        if not args:
            parser = argparse.ArgumentParser(description='Python SUMO Simulation command-line options')
            parsers.sumo.register_parser(parser)

            # Parse
            self.args = parser.parse_args()
        else:
            self.args = args

        # Check command line arguments
        assert self.args.penetration <= 1 and self.args.penetration >= 0, 'Penetration argument must be in interval [0, 1].'

        # Initialize microsimulation time-keeping
        self.sim_time = 0.0          # [s] Current simulation time - updated each step
        self.dt = 0.10               # [s] Step size (simulation frame interval). Used when --realtime is enabled.
        self.rt_start_time = counter()  # [s, monotonic] Wall-clock reference when last frame executed
        self.rt_iter = 0             # Number of simulation iterations executed

        # Vehicle color palette (RGBA in 0–255)
        # Updated: HDV=blue, CAV=red (always, including on-ramp)
        self.hdv_color = (0, 0, 255, 255)        # HDVs in blue
        self.cav_color = (255, 0, 0, 255)        # CAVs in red
        self.error_color = (255, 0, 0, 255)
        self.ext_color = (50, 255, 50, 255)

        self.colors = {
            'hdv': self.hdv_color,
            'cav': self.cav_color,
            'ext': self.ext_color,
            'error': self.error_color,
            'ghost': (0, 0, 0, 0)
        }

        # Set up SUMO backend
        if traci.isLibsumo():
            print('Using lib sumo.')
            # Guard: libsumo does not support the GUI flag
            assert not self.args.gui, 'Libsumo API does not support using GUI option! Adjust the Python import, or adjust command-line option'

        # SUMO scenario inputs
        SUMO_CFG_FILE = f"{self.args.scenario_folder}/{self.args.scenario}/{self.args.scenario}.sumocfg"
        assert os.path.isfile(SUMO_CFG_FILE), f'Could not find .sumocfg file at {SUMO_CFG_FILE}'

        # SUMO scenario outputs
        SUMO_OUT_DIR = f"{self.args.scenario_folder}/{self.args.scenario}/output"  # Directory where additional TraCI-defined outputs are saved

        if self.args.timestamp_output:  # Optionally suffix outputs with a timestamp
            now = datetime.now()
            formatted = now.strftime("_%Y-%m-%d_%H-%M-%S")
            SUMO_OUT_DIR += formatted

        os.makedirs(SUMO_OUT_DIR, exist_ok=True)

        # Choose SUMO binary
        if self.args.gui:  # GUI requires standard traci (libsumo does not support GUI)
            SUMO_BINARY = checkBinary('sumo-gui')
        else:
            SUMO_BINARY = checkBinary('sumo')

        # Compose SUMO command
        SUMO_CMD = [
            SUMO_BINARY,
            '-c', SUMO_CFG_FILE,
            '--start',

            # Verbosity/warnings
            '--no-warnings',  # Suppress warnings in console

            # Simulation behavior
            '--collision.action', 'warn',  # one of {teleport, warn, none, remove}

            '--step-length', str(self.dt),
            '--no-step-log',
            '--step-log.period', '50',     # Print TraCI step log every N simulation steps

            # Sublane model resolution (lane width proxy). Default true width is ~3.2 m.
            # '--lateral-resolution', str(LANEWIDTH/1.0),
            '--lanechange.duration', '3.0',  # Example for continuous lane change model

            '--seed', str(self.args.seed)   # RNG seed for reproducibility
        ]

        # Build a single penetration tag (e.g. 'p0.1', 'p0.25') so filenames stay consistent
        # Use the general format specifier to avoid floating-point artifacts (0.1 -> '0.1')
        penetration_tag = f"p{self.args.penetration:g}"

        if not self.args.no_inflow:  # For non-replay cases with inflow, enable data logging
            SUMO_CMD += [
                # Output files (use penetration_tag to keep names consistent)
                '--fcd-output', os.path.join(SUMO_OUT_DIR, f"fcd_{penetration_tag}.xml"),
                '--fcd-output.acceleration',
                '--fcd-output.max-leader-distance', str(RADAR_RANGE),  # include leader fields when within distance

                '--collision-output', os.path.join(SUMO_OUT_DIR, f"coll_{penetration_tag}.xml"),
                '--edgedata-output', os.path.join(SUMO_OUT_DIR, f"by_edge_{penetration_tag}.xml"),
                '--lanedata-output', os.path.join(SUMO_OUT_DIR, f"by_lane_{penetration_tag}.xml"),
                '--statistic-output', os.path.join(SUMO_OUT_DIR, f"stats_{penetration_tag}.xml"),
            ]

        # Modify flows according to CAV penetration rate (route file uses the same tag)
        output_file = f"{self.args.scenario_folder}/{self.args.scenario}/{self.args.scenario}_{penetration_tag}.rou.xml"

        reader.update_flows(
            penetration_rate=self.args.penetration,
            input_file=f"{self.args.scenario_folder}/{self.args.scenario}/{self.args.scenario}_template.rou.xml",
            output_file=output_file
        )

        SUMO_CMD += ['--route-files', output_file]

        if self.args.no_inflow:
            input_file = output_file
            output_file = output_file
            assert os.path.exists(input_file), f'Cannot find route file {input_file}'
            # self.zero_route_flows(input_file, output_file)

        print(f'Starting SUMO with configuration file: {SUMO_CFG_FILE}.')
        print(f'Outputting additional SUMO data to dir: {SUMO_OUT_DIR}.')

        # Initialize (C)AV controllers if penetration > 0
        if self.args.penetration > 0:
            # Pick the virtual ego controller:
            self.ego = PCC()  # Predictive Cruise Controller
            # self.ego = CAV() # TODO: CAV controller for eco-approach with I2V-connected intersections

        # Initialize V2V communications buffer
        self.comms = Communications()

        # Initialize traffic-light logger
        self.tl_logger = logger()

        # Start SUMO (TraCI/libsumo)
        traci.start(SUMO_CMD)

        # TraCI bookkeeping
        self.spawned_vehs = {}  # {veh_id: type_id} for vehicles currently in the network
        self.has_set_vehs = []  # List of veh_ids for which one-time properties were configured

    def stop(self):
        """Close the SUMO simulation and detach the TraCI/libsumo connection."""
        if traci.isLoaded():
            print('Closing SUMO.')
            traci.close()

    def init_vehicle(self, veh_id, route_id="mainlane", type_id="DEFAULT_VEHTYPE"):
        """
        Insert a new vehicle into the network and assign its route/type.

        Parameters
        ----------
        veh_id : str
            Vehicle ID to create.
        route_id : str
            Predefined route ID in the scenario/network.
        type_id : str
            Vehicle type ID to assign (must exist in the network).
        """
        traci.vehicle.add(veh_id, route_id)
        traci.vehicle.setType(veh_id, type_id)
        print(f"  Added new '{type_id}' vehicle '{veh_id}' with route '{route_id}'")

    def remove_vehicle(self, veh_id):
        """
        Remove a vehicle from the network.

        Parameters
        ----------
        veh_id : str
            Vehicle ID to remove.
        """
        traci.vehicle.remove(veh_id)
        print(f'  Removed vehicle {veh_id}')

    def set_camera(self, veh_id, zoom_factor = 300.0):
        """
        Focus the GUI camera on a given vehicle ID (only effective with --gui).

        Parameters
        ----------
        veh_id : str
            Vehicle ID to follow in the GUI.
        zoom_factor : float
            Zoom level to apply.
        """
        if self.args.gui:
            traci.gui.trackVehicle("View #0", veh_id)
            traci.gui.setZoom("View #0", zoom_factor)
            print(f'  Set camera to follow {veh_id}')

    def set_status_message(self, status = ""):
        """
        Write a transient status string into the GUI frame (only with --gui).

        Parameters
        ----------
        status : str
            Status message to display.
        """
        view_str = "View #0"
        traci.gui.setSchema(view_str, status)
        traci.gui.setOffset(view_str, x=10, y=20)  # Message position in the GUI

    def add_external_vehicle_type(self, copy_type_id, new_type_id, color):
        """
        Clone an existing vehicle type, adjust basic dimensions and color, and register it.

        Parameters
        ----------
        copy_type_id : str
            Existing vehicle type ID to copy from.
        new_type_id : str
            New vehicle type ID to create.
        color : tuple[int, int, int, int]
            RGBA color (0–255 each channel).
        """
        # Read attributes of the original type (fallbacks if missing)
        params = traci.vehicletype.getParameter(copy_type_id, "width")
        width = float(params) if params else 1.9

        params = traci.vehicletype.getParameter(copy_type_id, "length")
        length = float(params) if params else 4.2

        params = traci.vehicletype.getParameter(copy_type_id, "maxSpeed")
        max_speed = float(params) if params else 38.0

        # Create the new type
        traci.vehicletype.copy(copy_type_id, new_type_id)

        # Apply attributes
        traci.vehicletype.setWidth(new_type_id, width)
        traci.vehicletype.setLength(new_type_id, length)
        traci.vehicletype.setMaxSpeed(new_type_id, max_speed)
        traci.vehicletype.setMinGap(new_type_id, 2.0)
        traci.vehicletype.setMinGapLat(new_type_id, 0.1)
        traci.vehicletype.setColor(new_type_id, color)

        print(f"  Added new vehicle type '{new_type_id}' with color {color}")

    def step_external_vehicles(self, exts:Dict[str,EXT]):
        """
        Advance the XIL portion of the interface for a set of external vehicles.

        For each external agent, place it in the network at the provided position,
        set its kinematics (speed/accel/heading), push its V2V broadcast trajectory,
        and query nearby-vehicle and traffic-light context for the caller.

        Parameters
        ----------
        exts : Dict[str, EXT]
            Mapping of external vehicle IDs to `EXT` agent objects.
        """
        # Verify the vehicles exist in the network
        vehicle_ids = traci.vehicle.getIDList()

        for ext in exts.values():
            ego_id = ext.veh_id
            if ego_id not in vehicle_ids:
                raise ValueError(f'Cannot find external vehicle {ego_id}!')

            # Set states for the external vehicle
            ego_x = ext.states.longitude
            ego_y = ext.states.latitude
            ego_speed = ext.states.speed
            ego_angle = ext.states.heading
            ego_accel = ext.states.acceleration

            # MoveToXY: If no suitable road is found within ~100 m, this may raise
            xy_edge_id = ""     # unknown edge is acceptable as ""
            xy_lane = -1        # unknown lane is acceptable as -1
            xy_angle = ego_angle * (180.0 / pi)  # radians -> degrees
            xy_keep_route = int('000', 2)

            traci.vehicle.moveToXY(ego_id, xy_edge_id, xy_lane, ego_x, ego_y, xy_angle, xy_keep_route)
            traci.vehicle.setSpeed(ego_id, ego_speed)
            traci.vehicle.setAcceleration(ego_id, ego_accel, 1.0)

            # Push V2V trajectory into the communications buffer
            time_traj, pos_traj = ext.states.get_trajectory()
            self.comms.update_veh_comms(ego_id, time_traj, pos_traj)

            # Disable internal car-following and lane-changing behavior
            traci.vehicle.setSpeedFactor(ego_id, 1.0)
            traci.vehicle.setSpeedMode(ego_id, int('1100000', 2))
            traci.vehicle.setLaneChangeMode(ego_id, int('000000000000', 2))

            # Visual cue for external vehicles
            traci.vehicle.setColor(ego_id, self.colors['ext'])

            # Controller/context constraints
            s_max, is_lane_ending = self.get_end_of_lane_distance(ego_id)
            v_max = self.get_lane_speed_limit(ego_id)

            # Route-wise upcoming traffic lights
            tl_states = self.get_routewise_upcoming_traffic_light_states(ego_id)

            # Leader detection
            leader = traci.vehicle.getLeader(ego_id, dist=RADAR_RANGE)
            if leader is not None and leader[0] != "":
                lead_id = leader[0]                  # "flowID.vehID"
                lead_rel_distance = leader[1]        # gap from ego front to leader rear (plus minGap)
                lead_speed = traci.vehicle.getSpeed(lead_id)
                lead_accel = traci.vehicle.getAcceleration(lead_id)
                lead_heading = traci.vehicle.getAngle(lead_id)
                lead_heading_rate = 0.0
                lead_x, lead_y = traci.vehicle.getPosition(lead_id)
                lead_lat_gap = 0.0
                lead_type = traci.vehicle.getTypeID(lead_id)
                headway = lead_rel_distance / ego_speed if ego_speed > 0 else 0.0
            else:
                # Defaults when no leader is present
                lead_id = "-1"
                lead_rel_distance = 2000.0
                lead_speed = 0.0
                lead_accel = 0.0
                lead_heading = 0.0
                lead_heading_rate = 0.0
                lead_x = 0.0
                lead_y = 0.0
                lead_lat_gap = 0.0
                lead_type = None
                headway = -1

            # Preceding vehicle V2V snapshot at current sim time
            lead_t_comms, lead_s_comms, lead_v_comms = self.comms.get_veh_comms(
                lead_id, self.sim_time, distance_from_sv=lead_rel_distance
            )

            # Pack neighboring-vehicle information for the external agent
            ext.reset_nv_states()

            if lead_type or lead_type != 'None':
                ext.update_nv_states(
                    lead_id, lead_y, lead_x, lead_rel_distance, lead_lat_gap,
                    lead_speed, lead_accel, lead_heading, lead_heading_rate
                )
                if lead_t_comms:
                    ext.nv_states[lead_id].set_trajectory(times=lead_t_comms, rel_long_pos=lead_s_comms)

            # Periodic debug print
            if abs(fmod(self.sim_time, 10)) < 1e-6:
                print(f"  Ext: {ego_id}, Leader: {lead_id}, Gap: {lead_rel_distance:.2f} m, Headway: {headway:.2f} s")

    def step_replay_vehicle(self, veh_id, veh_type, x, y, speed, heading, acceleration):
        """
        Advance the "replay" vehicle to the given state (position, heading, speed)
        without engaging SUMO's internal driver models.

        Parameters
        ----------
        veh_id : str
        veh_type : str
            One of keys used in `self.colors` (e.g., 'hdv', 'cav', 'ext', 'error', 'ghost').
        x, y : float
            World coordinates.
        speed : float
            Linear speed [m/s].
        heading : float
            Heading in degrees (GUI uses degrees; if your source is radians, convert first).
        acceleration : float
            Longitudinal acceleration [m/s^2].
        """
        # Ensure the vehicle exists
        vehicle_ids = traci.vehicle.getIDList()

        ego_id = veh_id
        # if ego_id not in vehicle_ids:
        #     print(vehicle_ids)
        #     raise ValueError(f'Cannot find replay vehicle {ego_id}!')

        # State to apply
        ego_x = x
        ego_y = y
        ego_speed = speed
        ego_angle = heading
        ego_accel = acceleration

        # MoveToXY with minimal routing requirements
        xy_edge_id = ""     # unknown edge allowed
        xy_lane = -1        # unknown lane allowed
        xy_angle = ego_angle
        xy_keep_route = int('000', 2)

        traci.vehicle.moveToXY(ego_id, xy_edge_id, xy_lane, ego_x, ego_y, xy_angle, xy_keep_route)
        traci.vehicle.setSpeed(ego_id, ego_speed)
        traci.vehicle.setAcceleration(ego_id, ego_accel, 1.0)

        # Disable internal models and lane changes
        traci.vehicle.setSpeedFactor(ego_id, 1.0)
        traci.vehicle.setSpeedMode(ego_id, int('1100000', 2))
        traci.vehicle.setLaneChangeMode(ego_id, int('000000000000', 2))

        # Visual color by type
        color = self.colors[veh_type]
        traci.vehicle.setColor(ego_id, color)

    def get_end_of_lane_distance(self, ego_id):
        """
        Return distance to end of the current lane (if applicable) and whether the lane is ending.

        This is primarily used to constrain controller look-ahead in merging/on-ramp scenarios.

        Returns
        -------
        s_max : float
            Remaining forward travel distance allowed [m].
        is_lane_ending : bool
            Whether the current lane ends (e.g., a merge taper).
        """
        s_max = 5000.0  # Default large forward distance [m]
        is_lane_ending = False

        # Road-geometry example for the 'onramp' scenario
        if self.args.scenario == 'onramp':
            lane_id = traci.vehicle.getLaneID(ego_id)

            if self.is_on_onramp(lane_id):  # Hard-coded for this scenario
                link_length = traci.lane.getLength(lane_id)
                link_distance = traci.vehicle.getLanePosition(ego_id)

                s_max = link_length - link_distance
                is_lane_ending = True

        return s_max, is_lane_ending

    def get_lane_speed_limit(self, ego_id):
        """
        Return the effective speed limit for the current edge/lane.

        Notes
        -----
        Placeholder behavior is used here. To query true limits:
        - Retrieve the edge via `traci.vehicle.getRoadID(ego_id)`
        - Use `traci.edge.getMaxSpeed(edge_id)` for the posted limit
        - Consider vehicle-specific engine/governor limits if applicable

        Returns
        -------
        v_max : float
            Maximum allowed speed [m/s] for the controller to respect.
        """
        # Example: edge-based speed limit (disabled placeholder)
        # edge_id = traci.vehicle.getRoadID(ego_id)
        # speed_limit = traci.edge.getMaxSpeed(edge_id)
        speed_limit = 30.0  # [m/s] placeholder

        # Example physical/engine limit (important for trucks)
        speed_engine = 60.0  # [m/s] placeholder for traci.vehicle.getMaxSpeed(ego_id)

        # Controller cap
        v_max = max(0, min(speed_engine, speed_limit))
        return v_max

    def is_on_junction(self, ego_lane_id: str):
        """Return True if `ego_lane_id` refers to a junction (SUMO lane IDs starting with ':')."""
        return ego_lane_id.startswith(":")

    def is_on_onramp(self, ego_lane_id: str):
        """
        Return True if the ego is currently on a known on-ramp lane for merging.

        TODO
        ----
        Currently hard-coded to lane 'E2_0' for the 'onramp' scenario.
        """
        return ego_lane_id == 'E2_0'  # TODO: make lane filtering more robust

    def get_upcoming_traffic_light_states(self, ego_id):
        """
        Retrieve upcoming traffic-light states near the ego vehicle (within CONN_RANGE),
        based on the signal program (phases) of each controller.

        Parameters
        ----------
        ego_id : str
            Ego vehicle ID.

        Returns
        -------
        dict
            Mapping {tl_id: {sim_time, cycle_time, offset, distance_to_current_tl,
                              current_state, next_states, ring1, ring2, link_index}}.
            `next_states` contains the next N phase descriptors for each TL.
        """
        next_tls = traci.vehicle.getNextTLS(ego_id)  # list of (tlsID, tlsIndex, distance, state)
        traffic_light_states = {}

        for tls in next_tls:
            tl_id = tls[0]
            tl_ind = tls[1]
            distance = tls[2]
            current_state = tls[3]

            # Ignore far-away intersections
            if distance > CONN_RANGE:
                continue

            # Basic parameters (may be None depending on program configuration)
            cycle_time = traci.trafficlight.getParameter(tl_id, "cycleTime")
            offset = traci.trafficlight.getParameter(tl_id, "offset")

            logics = traci.trafficlight.getAllProgramLogics(tl_id)
            current_phase_i = traci.trafficlight.getPhase(tl_id)

            if logics:
                logic = logics[0]  # default program
                ring1 = logic.getParameter('ring1')
                ring2 = logic.getParameter('ring2')

                n_phases = len(logic.phases)
                next_states = {}
                for i in range(n_phases):
                    phase_index = (current_phase_i + i) % n_phases
                    name = logic.phases[phase_index].name
                    is_turning = name in ('2', '4', '6', '8')

                    next_states[i] = {
                        "phase_state": logic.phases[phase_index].state,
                        "cycle_duration": logic.phases[phase_index].duration,
                        "min_duration": logic.phases[phase_index].minDur,
                        "max_duration": logic.phases[phase_index].maxDur,
                        "phase_name": logic.phases[phase_index].name,
                        # Heuristics for yellow/red duration when not directly available
                        "yellow_guess": 3,
                        "red_guess": 2 if is_turning else 0
                    }

                traffic_light_states[tl_id] = {
                    "sim_time": self.sim_time,
                    "cycle_time": cycle_time,
                    "offset": offset,
                    "distance_to_current_tl": distance,
                    "current_state": current_state,
                    "next_states": next_states,
                    "ring1": ring1,
                    "ring2": ring2,
                    "link_index": -1,
                }
            else:
                warnings.warn(f"No Traffic light logic set for id {tl_id}?")

        return traffic_light_states

    def get_routewise_upcoming_traffic_light_states(self, ego_id):
        """
        Retrieve upcoming traffic-light states along the ego's planned route only
        (i.e., filter phases/links that do not belong to the current planned edges).

        Parameters
        ----------
        ego_id : str

        Returns
        -------
        dict
            Mapping {tl_id: {...}} similar to `get_upcoming_traffic_light_states`,
            but with `link_index` set to the active approach index on the TL controller.
        """
        traffic_light_states = {}

        if self.args.scenario == 'onramp':  # The 'onramp' scenario is expected to have no TL control on the ramp
            return traffic_light_states

        next_tls = traci.vehicle.getNextTLS(ego_id)
        if not next_tls:
            return traffic_light_states

        ego_planned_route = traci.vehicle.getRoute(ego_id)

        for tls in next_tls:
            tl_id = tls[0]
            tl_ind = tls[1]
            distance = tls[2]
            current_state = tls[3]

            if distance > CONN_RANGE:
                continue

            logics = traci.trafficlight.getAllProgramLogics(tl_id)
            current_phase_i = traci.trafficlight.getPhase(tl_id)

            if logics:
                logic = logics[0]
                tl_controlled_links = []

                tl_controlled_lanes = traci.trafficlight.getControlledLanes(tl_id)
                for lane in tl_controlled_lanes:
                    split = lane.split('_')
                    tl_controlled_links.append(split[0])
                    if len(split) > 2:
                        raise ValueError(f'TL Lane ID found as {lane}. Was expecting a format with one underscore <>_<>?')

                # Keep only the lanes present in the ego's planned route
                active_links = [link for link in tl_controlled_links if link in ego_planned_route]
                if not active_links:
                    raise ValueError("No active lanes?")

                link_index = tl_controlled_links.index(active_links[0])

                ring1 = logic.getParameter('ring1')
                ring2 = logic.getParameter('ring2')

                n_phases = len(logic.phases)
                next_states = {}
                for i in range(n_phases):
                    phase_index = (current_phase_i + i) % n_phases
                    name = logic.phases[phase_index].name
                    is_turning = name in ('2', '4', '6', '8')

                    next_states[i] = {
                        "phase_state": logic.phases[phase_index].state,
                        "cycle_duration": logic.phases[phase_index].duration,
                        "min_duration": logic.phases[phase_index].minDur,
                        "max_duration": logic.phases[phase_index].maxDur,
                        "phase_name": logic.phases[phase_index].name,
                        "yellow_guess": 3,
                        "red_guess": 2 if is_turning else 0,
                    }

                    # Sanity checks
                    if not len(logic.phases[phase_index].state) == len(tl_controlled_links):
                        raise ValueError('Phase state length does not match number of controlled links.')
                    elif not link_index and link_index != 0:
                        raise ValueError('Invalid link index computed.')

                traffic_light_states[tl_id] = {
                    "sim_time": self.sim_time,
                    "distance_to_current_tl": distance,
                    "current_state": current_state,
                    "next_states": next_states,
                    "ring1": ring1,
                    "ring2": ring2,
                    "link_index": link_index,
                }
            else:
                warnings.warn(f"No Traffic light logic set for id {tl_id}?")

        return traffic_light_states

    def step(self):
        """Advance the simulation by one TraCI step and run CAV control logic for vehicles of type 'cav'."""
        # Run one simulation step
        traci.simulationStep()

        # Update simulation time
        self.sim_time = traci.simulation.getTime()

        # Current vehicles in network
        vehicle_ids = traci.vehicle.getIDList()

        # Track newly spawned vehicles
        for id in vehicle_ids:
            if id not in self.spawned_vehs:
                vehicle_type = traci.vehicle.getTypeID(id)
                self.spawned_vehs[id] = vehicle_type
                # One-time: color HDVs on spawn so they use the configured palette
                if vehicle_type == 'hdv':
                    traci.vehicle.setColor(id, self.colors['hdv'])
                elif vehicle_type == 'cav':
                    traci.vehicle.setColor(id, self.colors['cav'])

        # Optionally keep only currently active vehicles
        for id in list(self.spawned_vehs.keys()):
            if id not in vehicle_ids:
                del self.spawned_vehs[id]

        # Run the virtual CAV controller for vehicles labeled 'cav'
        type_gen = (ego_id for ego_id, vtype in self.spawned_vehs.items() if vtype == 'cav')

        for ego_id in type_gen:
            # Controller constraints: end-of-lane distance
            s_max, is_lane_onramp = self.get_end_of_lane_distance(ego_id)

            if is_lane_onramp:
                # If on ramp, let the internal model handle it; keep CAV color consistent
                traci.vehicle.setColor(ego_id, self.colors['cav'])
                continue

            # Effective speed limit
            v_max = self.get_lane_speed_limit(ego_id)

            # Upcoming traffic signals along route
            tl_states = self.get_routewise_upcoming_traffic_light_states(ego_id)

            # Ego kinematics
            ego_distance = 0.0  # placeholder; could use traci.vehicle.getDistance(ego_id)
            ego_speed = traci.vehicle.getSpeed(ego_id)
            ego_accel = traci.vehicle.getAcceleration(ego_id)

            # Preceding vehicle
            leader = traci.vehicle.getLeader(ego_id, dist=RADAR_RANGE)
            if leader is not None and leader[0] != "":
                lead_id = leader[0]
                lead_rel_distance = leader[1]
                lead_speed = traci.vehicle.getSpeed(lead_id)
                lead_accel = traci.vehicle.getAcceleration(lead_id)
                lead_type = traci.vehicle.getTypeID(lead_id)
                headway = lead_rel_distance / ego_speed if ego_speed > 0 else 0
            else:
                lead_id = "-1"
                lead_rel_distance = 2000.0
                lead_speed = 0.0
                lead_accel = 0.0
                lead_type = 'None'
                headway = -1

            # V2V info for the leader
            lead_t_comms, lead_s_comms, lead_v_comms = self.comms.get_veh_comms(
                lead_id, self.sim_time, distance_from_sv=lead_rel_distance
            )

            # Query the controller
            desired_acceleration, time_traj, pos_traj, vel_traj, acc_traj = self.ego.getCommand(
                self.sim_time, ego_accel, ego_speed, ego_distance, s_max, v_max,
                lead_accel, lead_speed, lead_rel_distance,
                lead_t_comms, lead_s_comms, lead_v_comms
            )

            # Update V2V broadcast for the ego
            self.comms.update_veh_comms(ego_id, time_traj, pos_traj, vel_traj)

            # One-time vehicle parameters
            if ego_id not in self.has_set_vehs:
                traci.vehicle.setSpeedFactor(ego_id, 1.0)
                traci.vehicle.setSpeedMode(ego_id, int('1100000', 2))
                traci.vehicle.setLaneChangeMode(ego_id, int('011001010101', 2))
                traci.vehicle.setColor(ego_id, self.colors['cav'])
                self.has_set_vehs.append(ego_id)

            # Apply longitudinal command
            acc_duration = 1.0 # 3.64   [s] internal low-pass "propagation" time for realized acceleration
            traci.vehicle.setAcceleration(ego_id, desired_acceleration, acc_duration)

            # Optional per-step debug
            if self.args.debug:
                print(f"Time: {self.sim_time:.2f}, Vehicle: {ego_id}, Leader: {lead_id}, Gap: {lead_rel_distance:.2f} m, Headway: {headway:.2f} s")

        # Example: set others to 'ghost' color if needed
        # for ego_id in vehicle_ids:
        #     traci.vehicle.setColor(ego_id, self.colors['ghost'])

        # Periodic heartbeat
        if abs(fmod(self.sim_time, 10)) < 1e-6:
            print('t: {:0.1f}'.format(self.sim_time))

        # HDV debugging (kept as-is)
        if self.args.debug:
            type_gen = (ego_id for ego_id in vehicle_ids if traci.vehicle.getTypeID(ego_id) == 'hdv')
            for ego_id in type_gen:
                leader = traci.vehicle.getLeader(ego_id)
                if leader is not None and leader[0] == "ext_2":
                    lead_id = leader[0]
                    lead_rel_distance = leader[1]
                    lead_speed = traci.vehicle.getSpeed(lead_id)
                    lead_accel = traci.vehicle.getAcceleration(lead_id)
                    lead_type = traci.vehicle.getTypeID(lead_id)
                    print(f"Time: {self.sim_time:.2f}, Vehicle: {ego_id}, Gap: {lead_rel_distance:.2f} m, PV Vel: {lead_speed:.2f}, PV Acc: {lead_accel:.2f}")

    def rate(self):
        """
        Honor the requested real-time step rate (if `--realtime` is enabled) by sleeping
        the remainder of the frame budget.
        """
        if self.args.realtime:
            self.rt_iter += 1
            sleep_duration = self.dt * self.rt_iter - (counter() - self.rt_start_time)
            if sleep_duration > 0:
                sleep(sleep_duration)
            if sleep_duration < 1e-3:
                warnings.warn("Broadcast update too slow to maintain desired update rate")

    def is_running(self, lower_bound=0):
        """
        Return True while there are active agents (vehicles/pedestrians) in the network.

        Parameters
        ----------
        lower_bound : int
            Minimum expected number of remaining entities below which the sim stops.
        """
        return traci.simulation.getMinExpectedNumber() > lower_bound

    def sim(self, max_wall_time=99999, max_sim_time_guard=99999):
        """
        Three-guard simulation loop (no output format changes, fixed step, no 'fast' mode):

        1) Natural completion: stop when `getMinExpectedNumber() == 0`.
        2) Wall-clock guard: optional `max_wall_time` (seconds).
        3) Simulation-time guard: explicit `max_sim_time_guard`. If not provided,
           attempt to infer a planned end time from sumocfg/rou and use 2× that value.

        The loop periodically prints RTF (real-time factor) and small telemetry
        to help monitor performance.
        """
        # If not provided externally, fall back to args (if present, else None)
        if max_wall_time is None and hasattr(self.args, 'max_wall_time'):
            max_wall_time = self.args.max_wall_time
        if max_sim_time_guard is None and hasattr(self.args, 'max_sim_time_guard'):
            max_sim_time_guard = self.args.max_sim_time_guard

        try:
            # ---------- Guard #3: determine the simulation-time upper bound ----------
            guard_limit = None  # Final simulation-time limit (seconds)
            if max_sim_time_guard is not None:
                try:
                    if float(max_sim_time_guard) > 0:
                        guard_limit = float(max_sim_time_guard)
                        print(f"[INFO] using explicit sim guard: {guard_limit:.1f}s")
                except Exception:
                    pass

            if guard_limit is None:
                planned_end = None
                # 1) Try from sumocfg via TraCI (often <= 0 if not set)
                try:
                    et = traci.simulation.getEndTime()
                    if et and et > 0:
                        planned_end = float(et)
                except Exception:
                    planned_end = None

                # 2) Fallback: infer from the generated route file by scanning <flow>/<vehicle>
                if planned_end is None:
                    route_path = getattr(self, "route_file", None)
                    if route_path and os.path.exists(route_path):
                        try:
                            tree = ET.parse(route_path)
                            root = tree.getroot()
                            max_end = 0.0
                            for flow in root.iter('flow'):
                                end_attr = flow.get('end')
                                begin_attr = flow.get('begin')
                                if end_attr is not None:
                                    max_end = max(max_end, float(end_attr))
                                elif begin_attr is not None:
                                    max_end = max(max_end, float(begin_attr))
                            for veh in root.iter('vehicle'):
                                dep = veh.get('depart')
                                if dep is not None:
                                    max_end = max(max_end, float(dep))
                            if max_end > 0:
                                planned_end = max_end
                        except Exception as e:
                            print(f"[WARN] failed to infer planned end from route file: {e}")

                if planned_end is not None:
                    guard_limit = 2.0 * planned_end
                    print(f"[INFO] auto sim guard: planned_end={planned_end:.1f}s -> guard={guard_limit:.1f}s (x2)")
                else:
                    print("[INFO] no sim guard: neither explicit nor inferable from cfg/route")

            # ---------- Guard #2: wall-clock guard ----------
            wall_start = time.perf_counter()
            use_wall_guard = (max_wall_time is not None)
            if use_wall_guard:
                try:
                    max_wall_time = float(max_wall_time)
                    if max_wall_time <= 0:
                        use_wall_guard = False
                except Exception:
                    use_wall_guard = False

            # ---------- Main loop ----------
            debug_dump_interval = 10.0
            next_dump = 0.0

            print("Starting SUMO simulation.")
            while True:
                self.step()

                sim_time = traci.simulation.getTime()
                min_expected = traci.simulation.getMinExpectedNumber()

                # Periodic summary (incl. RTF)
                if sim_time >= next_dump:
                    wall_elapsed = time.perf_counter() - wall_start
                    rtf = sim_time / max(wall_elapsed, 1e-9)
                    inNet = len(traci.vehicle.getIDList())
                    loaded = len(traci.vehicle.getLoadedIDList())
                    tele = len(traci.vehicle.getTeleportingIDList())
                    print(f"[DBG] t={sim_time:.1f}s | wall={wall_elapsed:.1f}s | RTF={rtf:.3f} | "
                          f"minExpected={min_expected} | inNet={inNet} | loaded={loaded} | tele={tele}")
                    next_dump += debug_dump_interval

                # Guard #1: natural end
                if min_expected == 0:
                    print("[INFO] getMinExpectedNumber()==0 -> All vehicles have been processed. Simulation complete.")
                    break

                # Guard #2: wall clock
                if use_wall_guard:
                    wall_elapsed = time.perf_counter() - wall_start
                    if wall_elapsed >= max_wall_time:
                        print(f"[WARN] wall-clock exceeded {max_wall_time:.1f}s (elapsed={wall_elapsed:.1f}s) -> forcing stop")
                        break

                # Guard #3: simulation clock
                if guard_limit is not None and sim_time >= guard_limit:
                    print(f"[WARN] sim_time reached guard {guard_limit:.1f}s -> forcing stop")
                    break

                # Respect --realtime if enabled
                self.rate()

        except KeyboardInterrupt:
            print("[INFO] KeyboardInterrupt, exiting sim loop")
        except Exception as e:
            print(f"[ERROR] Unexpected simulation error {str(e)}")
        finally:
            self.stop()

# -------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    # Run simulation
    sumo = simulation()
    sumo.sim()