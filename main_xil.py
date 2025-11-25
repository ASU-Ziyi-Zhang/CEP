#!/usr/bin/env python3

#######################################
### Tyler Ard                       ###
### Yaozhong Zhang                  ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

# python -m cProfile -o profile_results.prof script.py
# snakeviz profile_results.prof

import warnings
import argparse
import traceback
from math import atan2, pi, fmod
from time import perf_counter as counter, sleep

from main import simulation

from src.agents import EXT
from src.settings import *
from src.xil_server import UDPServer
from src.messages import BSM, SPAT, SIM

import parsers.xil
import parsers.server
import parsers.sumo

from traci import TraCIException

MAX_N_RESETS = 10
RESETS = 0

class sumoWrapper():
    ''' 
    Wrapper class that integrates a microsimulation framework for Vehicle-in-the-Loop (VIL) or Software-in-the-Loop (SIL) environments. 
    It manages simulation settings, communication with external agents, and execution flow.
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
        parser = argparse.ArgumentParser('Python SUMO XIL Simulation with external vehicles')
        
        parser = parsers.sumo.register_parser(parser)
        parser = parsers.xil.register_parser(parser)
        parser = parsers.server.register_parser(parser)

        # Parse
        self.args = parser.parse_args()
        self.args.timestamp_output = True

        ### Ego vehicle/light states and controls
        # Object placed into microsimulation framework
        self.exts = {}

        ### Microsimulation
        self.microsim = simulation(args=self.args)

        ### Set up server to listen for multiple client messages
        self.server = UDPServer(server_ip=self.args.server_ip, server_port=self.args.server_port, framerate=self.args.framerate, is_debugging=self.args.debug)

        ### Error Check
        assert self.args.num_clients == 1, 'Only support 1 Num exts currently.'

    def reset(self):
        '''
        Resets the microsimulation and communication server.
        '''

        if self.server.is_running():
            self.publish_status(SIM.SimStatus.RESET.value)
            self.server.stop()

        self.microsim.stop()

        # Restart
        sleep(5)
        main()

    def stop(self):
        ''' 
        Terminates the microsimulation and communication server.
        - Stops the SUMO microsimulation process.
        - Closes the UDP server to halt external communications.
        '''

        if self.server.is_running():
            self.publish_status(SIM.SimStatus.OFFLINE.value)
            self.server.stop()

        self.microsim.stop()

    def wait(self):
        ''' 
        Waits for expected clients to connect before beginning the simulation.
        - Continuously checks if the required number of clients have connected.
        - Prints a status message when waiting for connections.
        - Updates external vehicle states while waiting.
        - Maintains the server update rate.
        '''
        first = True
        
        while self.server.is_running() and (not self.server.is_connected(self.args.num_clients) or not self.server.is_simming()):
            if first:
                print(f'Waiting for all {self.args.num_clients} clients to connect...')
                first = False

            # Initialize external agent
            self.update_exts()

            # Send update status
            self.publish_status(SIM.SimStatus.WAITING.value)

            # Wait
            self.server.rate()

        self.publish_status(SIM.SimStatus.RUNNING.value)

    def update_exts(self):
        ''' 
        Retrieves external vehicle motion data from connected clients and updates their state in the simulation.
        - Requests motion data from all connected clients.
        - Initializes new external vehicles if they are not already tracked.
        - Updates existing external vehicle states with received data.
        - Warns about error flags received from clients.
        '''

        bsm_data = self.server.get_all_data('bsm')

        if not bsm_data:
            return
        
        for client_ip, data in bsm_data.items():
            # Unpack communicated data from vehicle
            id = data['id']
            current_utc_time = data['current_utc_time']

            longitude = data['longitude']
            latitude = data['latitude']
            speed = data['speed']
            acceleration = data['acceleration']
            heading = data['heading']
            heading_rate_change = data['heading_rate_change']
            error_flag = data['error_flag']

            times = data['path_intentions_time']
            rel_long_pos = data['path_intentions_rel_long_pos']
            rel_lat_pos = data['path_intentions_rel_lat_pos']

            # Update external vehicle 
            if client_ip not in self.exts.keys():
                self.exts[client_ip] = EXT() # Make new vehicle associated with this client id
                self.exts[client_ip].veh_id = id

                # Initialize sim
                ext = self.exts[client_ip]
                self.microsim.init_vehicle(ext.veh_id, route_id=ext.route_id, type_id=ext.type_id)

                self.microsim.set_camera(ext.veh_id, 650)
            
            self.exts[client_ip].update_states(latitude, longitude, speed, acceleration, heading, heading_rate_change, times=times, rel_long_pos=rel_long_pos)

            # Check error
            if error_flag != BSM.Error.OK.value:
                warnings.warn(f'Error flag of {error_flag} received from client')

    def publish_status(self, sim_status):
        '''
        Publish the readiness status of the XIL server.
        '''

        sim_data = SIM.make_sim('SUMO-XIL', sim_status, SIM.Error.OK.value)

        self.server.broadcast_update_to_all(sim_data)

    def publish_microsim(self):
        ''' 
        Collects microsimulation data and publishes relevant updates to external clients.
        - Extracts information about external vehicles and their neighboring vehicles.
        - Formats motion data into standardized messages.
        - Sends simulation updates to each connected client.
        '''
        for client, ext in self.exts.items():
            bsm_data = {}

            # Send neighboring vehicle information
            if ext.nv_states:
                for nv_id, nv in ext.nv_states.items():
                    # States
                    rel_long_gap = nv.rel_long_gap
                    rel_lat_gap = nv.rel_lat_gap

                    latitude = nv.latitude
                    longitude = nv.longitude
                    speed = nv.speed
                    acceleration = nv.acceleration
                    heading = nv.heading
                    heading_rate_change = nv.heading_rate_change

                    error_flag = BSM.Error.OK.value

                    # Make BSM
                    bsm_data = {nv_id:
                        BSM.make_bsm(nv_id, latitude, longitude, speed, acceleration, heading, heading_rate_change, error_flag,
                                     rel_long_gap=rel_long_gap, rel_lat_gap=rel_lat_gap)
                    }

                    # Intentions
                    if nv.intended_trajectory:
                        t, s = nv.get_trajectory()

                        bsm_data[nv_id] = BSM.Trajectory.make_and_add_trajectory(
                            bsm_data[nv_id], t, s
                        )
            
            for id, message in bsm_data.items():
                self.server.send_update(message, client)

    def sim(self):
        ''' 
        Runs the main simulation loop within the XIL interface.
        - Adds external vehicle types to the microsimulation.
        - Waits for necessary client connections before proceeding.
        - Iterates through simulation steps, updating external vehicles and publishing data.
        - Ensures consistency between microsimulation step size and communication framerate.
        - Manages error handling, capturing exceptions related to simulation runtime failures.
        '''
        global RESETS

        try:
            # Create a new external vehicle type
            self.microsim.add_external_vehicle_type("hdv", EXT().type_id, self.microsim.ext_color)

            # Wait for other nodes/clients
            self.wait()

            ### Simulation
            t_start = counter()

            while self.server.is_running() and self.server.is_simming() and self.microsim.is_running():
                ### Step microsim
                self.microsim.step()

                ### Step the VIL portion of microsim
                # Get client data
                self.update_exts()
                        
                # Step external vehicles
                self.microsim.step_external_vehicles(self.exts)
                
                # Get server data update for clients
                self.publish_microsim()

                if abs(fmod(self.microsim.sim_time, 1)) < 1e-6: # Publish every n seconds of simulation time
                    self.publish_status(SIM.SimStatus.RUNNING.value)

                # Iterate
                if abs(fmod(self.microsim.sim_time, 1)) < 1e-6: # Print output every n seconds of simulation time
                    print(f'real time={(counter()-t_start):0.2f}, sim time={self.microsim.sim_time:0.2f}')

                ### Error Check
                assert abs(self.microsim.dt - 1./self.args.framerate) < 1e-3, 'Microsimulation stepsize and server framerate set at mismatched time intervals!'

                ### Rollover and control runtime rate
                self.server.rate()

            if self.microsim.sim_time + self.microsim.dt >= TEND:
                print('Microsim shutdown automatically due to t > t_end.')

            if not self.server.is_connected(self.args.num_clients):
                print('Microsim shutdown automatically due to some clients disconnecting.')
            
            if self.server.is_resetting():
                if RESETS >= MAX_N_RESETS:
                    print('Max number reset requests exceeded.')

                if RESETS < MAX_N_RESETS:
                    print('Reset request detected.')
                    RESETS += 1
                    
                    self.reset()
            
            if not self.server.is_simming():
                print('Microsim shutdown automatically due to some clients requesting sim shutdown.')

        except KeyboardInterrupt as e:
            RESETS = MAX_N_RESETS

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
    # Run simulation in ROS node
    main()

    # Exit
    print('Exited XIL simulation.')
