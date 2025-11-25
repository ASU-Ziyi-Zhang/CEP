#!/usr/bin/env python3

#######################################
### Tyler Ard                       ###
### Jongryeol Jeong                 ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

# python -m cProfile -o profile_results.prof script.py
# snakeviz profile_results.prof

import os
import warnings
import argparse
import traceback
from math import atan2, pi, fmod
from time import perf_counter as counter, sleep
import xml.etree.ElementTree as ET

from main import simulation

from src.agents import EXT
from src.settings import *

import parsers.sumo

import traci # traci has full API options
from traci import TraCIException

class sumoWrapper():
    ''' 
    Wrapper class that integrates a microsimulation framework for FCD playback. 
    It manages simulation settings, file loading, and execution flow.
    '''

    def __init__(self):
        ''' 
        Initializes the simulation environment, sets up argument parsing, and establishes a UDP server for communication.
        - Parses command-line arguments to configure simulation settings.
        - Initializes an empty dictionary for tracking external vehicles.
        - Sets up the SUMO microsimulation framework.
        - Configures a UDP server to handle messages from connected clients.
        - Ensures only a single external client is supported in this implementation.
        '''

        ### Handle input arguments
        parser = argparse.ArgumentParser('Python SUMO Simulation replay from FCD file')
    
        parser = parsers.sumo.register_parser(parser)
        parser.add_argument('--output_dir',
            help='Select output directory to analyze from with <scenario_folder>/<scenario>/. Default "output"',
            default="output", nargs="?", type=str)
        
        parser.add_argument('--file',
            help='Set file to plot from <scenario_folder>/<scenario>/<output_dir>/ directory. Default "fcd.xml"',
            default="fcd.xml", nargs="?", type=str)

        # Parse
        self.args = parser.parse_args()
        self.args.no_inflow = True
        self.args.gui = True

        # Select the output data folder
        output_dir = os.path.join(self.args.scenario_folder, self.args.scenario, self.args.output_dir) # Directory where output file to read is located
        
        assert os.path.isdir(output_dir), f'Cannot find output directory! {output_dir}'

        # Select the output data file
        self.fcd_path = os.path.join( output_dir, self.args.file ) # Absolute path to the output file for analysis

        # Initialize metrics with stats file if exists
        assert os.path.exists(self.fcd_path), f'Cannot find fcd file! {self.fcd_path}'
    
        ### Ego vehicle/light states and controls
        # Object placed into microsimulation framework
        self.vehs = {}

        ### Microsimulation
        self.microsim = simulation(args=self.args)

        ### Timing
        framerate = 10 # HZ

        self.update_interval = 1./framerate  # [s] Time between sending updates
        self.time_at_last_update = counter()  # [s] The timestamp that the last update was sent - will not send a new one until the update interval has passed
        self.time_at_start = counter()
        self.update_iter = 0 # Number of rate updates done


    def stop(self):
        ''' 
        Terminates the microsimulation and communication server.
        - Stops the SUMO microsimulation process.
        - Closes the UDP server to halt external communications.
        '''

        self.microsim.stop()

    def rate(self):
        '''Starts a timer to pause until next update interval'''
        # Calculate Sleep duration and sleep
        self.update_iter += 1
        
        sleep_duration = self.update_interval*self.update_iter - (counter()-self.time_at_start)
        if sleep_duration > 0:
            sleep(sleep_duration) # Control broadcast update rate

        # Error check
        if sleep_duration < 1e-3:
            warnings.warn("Broadcast update too slow to maintain desired update rate")

        # The time when the last update was made
        self.time_at_last_update = counter()


    def sim(self):
        ''' 
        Runs the main simulation loop within the XIL interface.
        - Adds external vehicle types to the microsimulation.
        - Waits for necessary client connections before proceeding.
        - Iterates through simulation steps, updating external vehicles and publishing data.
        - Ensures consistency between microsimulation step size and communication framerate.
        - Manages error handling, capturing exceptions related to simulation runtime failures.
        '''
        try:
            # Create a new external vehicle type
            template_type='hdv'
            self.microsim.add_external_vehicle_type(template_type, EXT().type_id, self.microsim.ext_color)

            ### Simulation
            t_start = counter()

            tree = ET.parse(self.fcd_path)
            root = tree.getroot()

            seen_vehicles = set()
   
            for timestep in root.iter('timestep'):
                sim_time = float(timestep.get('time'))
        
                ### Step microsim
                self.microsim.step()

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
                        print(vehicle_ids)
                        self.microsim.init_vehicle(veh_id, route_id="mainlane", type_id=veh_type)

                        # Set the camera if an external vehicle detected
                        if veh_type == EXT().type_id:
                            self.microsim.set_camera(veh_id, 650)

                    ### Step the replay of microsim
                    self.microsim.step_replay_vehicle(veh_id, veh_type, x, y, speed, angle, accel)

                    seen_vehicles.add(veh_id)
                
                # Iterate
                if abs(fmod(self.microsim.sim_time, 1)) < 1e-6: # Print output every n seconds of simulation time
                    print(f'real time={(counter()-t_start):0.2f}, sim time={self.microsim.sim_time:0.2f}')

                ### Error Check
                assert abs(self.microsim.dt - self.update_interval) < 1e-3, 'Microsimulation stepsize and server framerate set at mismatched time intervals!'

                ### Rollover and control runtime rate
                self.rate()

            print('Microsim shutdown automatically.')

        except KeyboardInterrupt as e:
            print('Ctrl-C detected. Stopping.')

        except ValueError as e:
            print(f'A value error occurred: {e}')

        except AssertionError as e:
            print(f'An assertion error occurred: {e}')

        except TraCIException as e:
            print(f'SUMO exception occurred: {e}')

        except IndexError as e:
            print(traceback.format_exc())
            print(f'Index error occurred: {e}')

        except KeyError as e:
            print(traceback.format_exc())
            print(f'Key error occurred: {e}')

        except Exception as e:
            print(traceback.format_exc())
            print(f'An unknown exception occurred: {e}')

        # Shutdown
        try:
            self.stop()

        except Exception as e:
            print(f'An exception occurred: {e}')

def main():
    wrapper = sumoWrapper()
    wrapper.sim()

if __name__ == '__main__':
    # Run simulation
    main()

    # Exit
    print('Exited simulation replay.')
