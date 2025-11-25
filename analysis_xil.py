
import xml.etree.ElementTree as ET
import numpy as np
from math import nan, isnan, inf, fmod
from collections import defaultdict, deque
import os
import argparse
import warnings
import sys

import pandas as pd
from scipy.io import savemat
from scipy.interpolate import interp1d

import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as colors

import scripts.utils_vis as vis
from src.settings import *
from analysis import TrafficMetrics

class MeanStream:
    def __init__(self, bounds=(0,1)):
        self.iter = 0

        self.inds = None
        self.mean = None

        self.bounds = bounds

    def update(self, x, y):
        # For first update call
        if self.mean is None:
            self.inds = np.linspace(self.bounds[0], self.bounds[1], len(x))
            self.mean = np.zeros_like(self.inds, dtype=float)

        # Get consistent independent variable
        x = np.array(x)
        y = np.array(y)

        interp_func = interp1d(x, y, bounds_error=False, fill_value=np.nan)
        vec = interp_func(self.inds)
        
        self.iter += 1
        self.mean += (vec - self.mean) / self.iter
        
        return self.mean

class TrafficVisualizationXIL:
    def get_lane(self, edge_id):
        # Return if not a string edge_id
        if not isinstance(edge_id, str):
            return edge_id
        
        # Special cases
        if 'ramp' in edge_id:
            return '9'
        
        # Split string to get id
        split_ids = edge_id.split('_')

        assert len(split_ids) > 1, f'Edge_id in expected format of <>_<>? {edge_id}'

        return split_ids[-1]
    
    def process_fcd(self, fcd_file, lanes, direction):
        # Parse the FCD XML file
        tree = ET.parse(fcd_file)
        root = tree.getroot()
        
        # Extract vehicle data
        data = []

        y_min, y_max, x_min, x_max, v_min, v_max = inf, -inf, inf, -inf, inf, -inf
        ego_lanes = {}

        for timestep in root.findall('timestep'):
            # Simulation data
            time = float(timestep.get('time'))

            for vehicle in timestep.findall('vehicle'):
                vehicle_id = vehicle.get('id')
                
                x = float(vehicle.get('x'))
                y = float(vehicle.get('y'))
                angle = float(vehicle.get('angle'))
                type = vehicle.get('type')
                speed = float(vehicle.get('speed'))
                pos = float(vehicle.get('pos'))
                lane_id = self.get_lane( vehicle.get('lane') )
                acceleration = float(vehicle.get('acceleration'))
                
                leader_id = vehicle.get('leaderID')
                leader_speed = vehicle.get('leaderSpeed')
                leader_gap = vehicle.get('leaderGap')
                if leader_id != '' and float(leader_gap) <= 100.:
                    leader_speed = float(leader_speed)
                    leader_gap = float(leader_gap)
                else:
                    leader_id = nan
                    leader_speed = nan
                    leader_gap = nan
                
                # Iterate    
                y_min = min(y_min, y)
                y_max = max(y_max, y)
                x_min = min(x_min, x)
                x_max = max(x_max, x)
                v_min = min(v_min, speed)
                v_max = max(v_max, speed)

                # Check the ego lane
                if type in self.ego_types:
                    if vehicle_id not in ego_lanes:
                        ego_lanes[vehicle_id] = []

                    ego_lanes[vehicle_id].append(lane_id)

                data.append([time, vehicle_id, x, y, angle, type, speed, pos, lane_id, acceleration, leader_id, leader_speed, leader_gap])

        # Create a DataFrame
        df = pd.DataFrame(data, columns=['time', 'vehicle_id', 'x', 'y', 'angle', 'type', 'speed', 'pos', 'lane_id', 'acceleration', 'leader_id', 'leader_speed', 'leader_gap'])
        
        # Filter data for specific lanes if provided
        if lanes is not None:
            df = df[df['lane'].isin(lanes)]
        
        # Filter data based on direction
        if direction is not None: 
            angle_range = 60
            dir_map = {
                'N': [360-angle_range/2, 360] + [0, angle_range/2],  # Split range
                'NE': [45-angle_range/2,45+angle_range/2],
                'E': [90-angle_range/2,90+angle_range/2],
                'SE': [135-angle_range/2,135+angle_range/2],
                'S': [180-angle_range/2, 180+angle_range/2],
                'SW': [225-angle_range/2,225+angle_range/2],
                'W': [270-angle_range/2, 270+angle_range/2],
                'NW':  [315-angle_range/2, 315+angle_range/2]
            }
            angles = dir_map[direction.upper()] # (min, max) to select
            if len(angles)>2:
                # Handle wrapped ranges (e.g., North = [330,360] + [0,30])
                condition = (
                    (df['angle'] >= angles[0]) & (df['angle'] <= angles[1]) |
                    (df['angle'] >= angles[2]) & (df['angle'] <= angles[3])
                )
            else:
                # Standard single range
                condition = (
                    (df['angle'] >= angles[0]) & 
                    (df['angle'] <= angles[1])
                )
            
            df = df[condition]

        ### Save dataframe
        self.fcd_df = df
        
        self.y_min, self.y_max, self.x_min, self.x_max, self.v_min, self.v_max = y_min, y_max, x_min, x_max, v_min, v_max
        self.ego_lanes = ego_lanes

    def __init__(self, traffic_metrics, output_dir, fcd_file, net_file='', lanes=None, direction=None):
        
        ### Settings
        self.NUM_LANES = 4
        self.LINEWIDTH = 2

        self.is_plotting_position = False
        
        self.is_plotting_vehgap = True
        self.is_plotting_tlgap = False

        # Independent variable
        if self.is_plotting_position:
            self.XLABEL = 'Position [m]'
        else:
            self.XLABEL = 'Time [s]'

        # Ego types
        self.ego_types = (
            ControlType.EXT.value,
            ControlType.SAS.value,
            'ext',
        )

        ### Store traffic metrics
        self.traffic_metrics = traffic_metrics

        ### Process FCD
        self.process_fcd(fcd_file, lanes, direction)

        # Export dataframe to .mat file
        try:
            data_dict = {col: self.fcd_df[col].values for col in self.fcd_df.columns}
            matname = os.path.join(output_dir, 'fcd_data.mat')
            
            savemat(matname, data_dict)
            print(f'  Saved dataframe as {matname}.')

        except:
            print('Was not able to save dataframe as .mat file! Moving on.')

    def getZorder(self, veh_type, n = 1):
        type_zorder = {
            ControlType.EXT.value: 3,
            ControlType.SAS.value: 3,
            'ext': 3
        }

        if veh_type in type_zorder:
            return type_zorder[veh_type]
        else:
            return n
    
    def getColor(self, veh_type):
        type_color = {
            ControlType.NONE.value: 'k',
            'none': 'k',
            ControlType.MOBIL.value: 'k',
            'mobil': 'k',
            ControlType.HDV.value: 'k',
            'hdv': 'k',

            ControlType.CAV.value: 'b',
            'cav': 'b',
            
            ControlType.PCC.value: 'g',
            'pcc': 'g',
            
            ControlType.EXT.value: 'r',
            ControlType.SAS.value: 'r',
            'ext': 'r',
        }

        if veh_type in type_color:
            return type_color[veh_type]
        else:
            raise ValueError(f'Unknown veh type {veh_type} in getColor!')
    
    def getColorFade(self, color, alphas):
        # Create a fade
        rgb = colors.to_rgb(color)
        fade = [rgb + (a,) for a in alphas] # Add a transparency
        
        return fade
    
    def getColorMap(self, veh_type):
        # Get base vehicle color
        c = self.getColor(veh_type)
        
        # Create a colormap
        fade = colors.to_rgb(c) + (0.1,) # Add a transparency
        colormap = colors.LinearSegmentedColormap.from_list('my_cm', [fade, c])
        
        return colormap
    
    def getLabel(self, veh_type, label_dict):
        # Get label based on type
        label = None

        type_label = {
            ControlType.NONE.value: None,
            'none': None,

            ControlType.MOBIL.value: 'HV',
            'mobil': 'HV',
            ControlType.HDV.value: 'HV',
            'hdv': 'HV',

            ControlType.CAV.value: 'CAV',
            'cav': 'CAV',
            
            ControlType.PCC.value: 'AV',
            'pcc': 'AV',
            
            ControlType.EXT.value: 'EGO',
            ControlType.SAS.value: 'EGO',
            'ext': 'EGO',
        }

        if veh_type in type_label:
            label = type_label[veh_type]
        else:
            raise ValueError(f'Unknown veh type {veh_type} in getLabel!')

        # Only add label if not used previously in legend dict
        if veh_type in label_dict.keys():
            label = None

        else:
            label_dict[veh_type] = label

        return label, label_dict
    

    def plot_postime(self, color_by_type=False, color_by_lane=False):
        fig, ax = plt.subplots()

        labels = dict()
        sm = None

        for veh_id, data in self.fcd_df.groupby('vehicle_id'):
            veh_type = data['type'].iloc[0]

            t, s, y, lane_id = data['time'], data['x'], data['y'], data['lane_id']

            # Zorder for ego vehicle
            zorder = self.getZorder(veh_type)

            # Label
            label, labels = self.getLabel(veh_type, labels)

            # Plot vehicle trajectory
            if color_by_type:
                # Get color based on type and lateral position
                c = self.getColor(veh_type)

                cmap = self.getColorMap(veh_type)
                
                y_min = self.y_max-LANEWIDTH
                norm = colors.Normalize(vmin=y_min-0.01, vmax=self.y_max+0.01) # Color from lane 1 to lane 2

                if label:
                    plt.plot([], [], color=c, label=label, linewidth=self.LINEWIDTH)
                
                plt.plot(t, s,
                     color = c,
                     label=None, linewidth = 0.25*self.LINEWIDTH, alpha=0.1,
                     zorder=zorder) # Cant use colormap for the lane

                plt.scatter(t, s,
                     c = y, cmap=cmap, norm=norm,
                     label=None, s=0.25*self.LINEWIDTH,
                     zorder=zorder) # Colormap to indicate which lane the vehicle is in
            
            elif color_by_lane:
                # Get color based on type and matching lane with ego
                c = self.getColor(veh_type)
                
                a_min, a_max = 0.01, 1.00
                alphas = [a_max if lid in ego_lid else a_min for lid, ego_lid in zip( lane_id, list(self.ego_lanes.values())[0] )]
                fade = self.getColorFade(c, alphas)

                if label:
                    plt.plot([], [], color=c, label=label, linewidth=self.LINEWIDTH)
                
                plt.plot(t, s,
                     color = c,
                     label=None, linewidth = 0.25*self.LINEWIDTH, alpha=a_min,
                     zorder=zorder) # Cant use colormap for the lane

                plt.scatter(t, s,
                    c = fade, # Color with transparency to indicate which lane the vehicle is in
                    label=None, s=0.25*self.LINEWIDTH,
                    zorder=zorder)

            else:
                # Identify ego types
                is_ego = np.isin(data['type'], self.ego_types)

                # Use speed only for non-ego, set ego color to zero for consistency
                color_values = np.where(is_ego, 0.0, data['speed'])

                # Normalize and map to colormap
                norm = colors.Normalize(vmin=color_values.min(), vmax=color_values.max())#vmin=self.v_min, vmax=self.v_max)
                cmap = plt.get_cmap('viridis')
                rgba = cmap(norm(color_values))

                sm = cm.ScalarMappable(norm=norm, cmap=cmap)
                sm.set_array([])  # Required for the colorbar to work even if scatter uses RGBA
                
                # Apply transparency: 1.0 for ego, 0.3 for non-ego
                rgba[:, -1] = np.where(is_ego, 1.0, 0.3)

                # Scatter plot with RGBA colors
                ax.scatter(t, s, 
                            c=rgba, s=0.1, 
                            zorder=zorder)

        if not color_by_type and not color_by_lane and sm is not None:
            cbar = fig.colorbar(sm, ax=ax)  # Use fig.colorbar() instead of plt.colorbar()
            cbar.set_label('Speed (m/s)')

        plt.xlabel('Time [s]')
        plt.ylabel('Position [m]')
        
        if color_by_type or color_by_lane:
            plt.legend()

        plt.draw()

    
    def plot_velacc(self):
        fig, axs = plt.subplots(2, 1, sharex=True)

        labels = dict()
        for veh_id, data in self.fcd_df.groupby('vehicle_id'):
            veh_type = data['type'].iloc[0]

            t, s, lane_id = data['time'], data['x'], data['lane_id']
            v, a = data['speed'], data['acceleration']

            # Get independent variable of t or s
            if self.is_plotting_position:
                x = s
            else:
                x = t

            # Zorder for ego vehicle
            zorder = self.getZorder(veh_type)

            # Select line color based on vehicle type
            c = self.getColor(veh_type)

            a_min, a_max = 0.01, 1.00
            alphas = [a_max if lid in ego_lid else a_min for lid, ego_lid in zip( lane_id, list(self.ego_lanes.values())[0] )]
            fade = self.getColorFade(c, alphas)

            label, labels = self.getLabel(veh_type, labels)

            # Plot vehicle velocity trajectory
            if label:
                plt.plot([], [], color=c, label=label, linewidth=self.LINEWIDTH)
            
            ax = axs[0]
            
            ax.scatter(x, v, 
                     color = fade, label=None, s = 0.2*self.LINEWIDTH,
                     zorder=zorder)

            ax.set_ylabel('Velocity [m/s]')

            # Plot vehicle acceleration trajectory
            ax = axs[1]

            ax.scatter(x, a, 
                     color = fade, label=None, s = 0.2*self.LINEWIDTH,
                     zorder=zorder)

            ax.set_ylabel('Accel [m/s2]')
            ax.set_xlabel(self.XLABEL)

        plt.legend()

        ### Plot Mean trajectories
        v_means, a_means = {}, {}
        x_limits = plt.xlim()

        for veh_id, data in self.fcd_df.groupby('vehicle_id'):
            veh_type = data['type'].iloc[0]

            t, s, lane_id = data['time'], data['x'], data['lane_id']
            v, a = data['speed'], data['acceleration']

            # Get independent variable of t or s
            if self.is_plotting_position:
                x = s
            else:
                x = t
                
            # Start calculating mean trajectory
            v_mean_stream = v_means.get(veh_type, MeanStream(bounds=x_limits))
            a_mean_stream = a_means.get(veh_type, MeanStream(bounds=x_limits))

            v_mean_stream.update(x,v)
            a_mean_stream.update(x,a)

            v_means[veh_type] = v_mean_stream
            a_means[veh_type] = a_mean_stream
    
        #
        labels = dict()
        
        ax = axs[0]
        for veh_type, stream in v_means.items():
            
            c = self.getColor(veh_type)
            label, labels = self.getLabel(veh_type, labels)
            zorder = self.getZorder(veh_type)

            if label:
                label += '_mean'

            ax.plot(stream.inds, stream.mean, 
                     color = c, label=label, linewidth = self.LINEWIDTH,
                     zorder=zorder)
            
        ax = axs[1]
        for veh_type, stream in a_means.items():
            
            c = self.getColor(veh_type)
            label, labels = self.getLabel(veh_type, labels)
            zorder = self.getZorder(veh_type)

            if label:
                label += '_mean'

            ax.plot(stream.inds, stream.mean, 
                     color = c, label=label, linewidth = self.LINEWIDTH,
                     zorder=zorder)
            
        plt.draw()


    def plot_gap(self):
        labels = dict()
        
        fig, axs = plt.subplots(2, 1, sharex=True)

        for veh_id, data in self.fcd_df.groupby('vehicle_id'):
            veh_type = data['type'].iloc[0]

            if veh_type not in self.ego_types:            
                continue

            # Get rest of ego vehicle data
            t, s, lane_id = data['time'], data['x'], data['lane_id']
            v, a = data['speed'], data['acceleration']
            pv_gap, pv_v = data['leader_gap'], data['leader_speed']

            # Get independent variable of t or s
            if self.is_plotting_position:
                x = s
            else:
                x = t

            # Zorder for ego vehicle
            zorder = self.getZorder(veh_type)

            # Select line color based on vehicle type
            c = self.getColor(veh_type)

            cmap = self.getColorMap(veh_type)
            norm = colors.Normalize(vmin=1, vmax=self.NUM_LANES) # Color from lane 1 to lane 2

            label, labels = self.getLabel(veh_type, labels)

            # Plot vehicle velocity trajectory
            ax = axs[0]
            ax.plot(x, pv_gap, 
                    color = c, label=label, linewidth = self.LINEWIDTH,
                    zorder=zorder)

            ax.set_ylabel('Gap [m]')

            # Plot vehicle acceleration trajectory
            ax = axs[1]
            ax.plot(x, v,
                    color = c, label=label, linewidth = self.LINEWIDTH,
                    zorder=zorder)

            ax.plot(x, pv_v,
                    color = 'k', label='PV', linewidth = self.LINEWIDTH,
                    zorder=zorder)

            ax.set_ylabel('Speed [m/s]')
            ax.set_xlabel(self.XLABEL)

        plt.legend()
        plt.draw()


    def plot_xil(self, save_path=None, save_ext='.png'):
        """
        Plot xil-related visualizations.
        If save_path is provided, save the figure to the specified path.
        """
        ############################
        ### POS TRAJECTORIES     ###
        ############################
        ### Plot time-position trajectories through corridor
        self.plot_postime()
        
        # Save
        if save_path:
            save_name = save_path+'_pos'+save_ext
            plt.savefig(save_name, dpi=300, bbox_inches='tight')
            print(f"  Figure saved to {save_name}")

        ### Plot time-position trajectories through corridor
        self.plot_postime(color_by_type=True)
        
        # Save
        if save_path:
            save_name = save_path+'_pos_color'+save_ext
            plt.savefig(save_name, dpi=300, bbox_inches='tight')
            print(f"  Figure saved to {save_name}")

        ### Plot time-position trajectories through corridor
        self.plot_postime(color_by_lane=True)
        # Save
        if save_path:
            save_name = save_path+'_pos_lane'+save_ext
            plt.savefig(save_name, dpi=300, bbox_inches='tight')
            print(f"  Figure saved to {save_name}")

        ############################
        ### V AND A TRAJECTORIES ###
        ############################
        ### Plot individual velocity, acceleration trajectories through corridor
        self.plot_velacc()

        # Save
        if save_path:
            save_name = save_path+'_va'+save_ext
            plt.savefig(save_name, dpi=300, bbox_inches='tight')
            print(f"  Figure saved to {save_name}")

        ####################
        ### GAP VELOCITY ###
        ####################
        ### Plot gap velocity trajectories through corridor
        if self.is_plotting_vehgap:
            self.plot_gap()

            # Save
            if save_path:
                save_name = save_path+'_gap'+save_ext
                plt.savefig(save_name, dpi=300, bbox_inches='tight')
                print(f"  Figure saved to {save_name}")

        #######################
        ### TL GAP VELOCITY ###
        #######################
        ### Plot light gap velocity trajectories through corridor
        if self.is_plotting_tlgap:
            fig, axs = plt.subplots(2, 1, sharex=True)        

            # Save
            if save_path:
                save_name = save_path+'_tl'+save_ext
                plt.savefig(save_name, dpi=300, bbox_inches='tight')
                print(f"  Figure saved to {save_name}")

        ####################
        ### DONE         ###
        ####################
        ### Show plots and wait to exit
        plt.show(block=False)

        # input("Press Enter to continue...") # raw_input in python2
        plt.close('all')

## ---------------------------------------------------------- ##

if __name__ == "__main__":
    ### Handle input arguments
    parser = argparse.ArgumentParser('Python SUMO data analysis and visualization tool')
    
    parser.add_argument('--scenario_folder',
        help='Set directory path to search for output file to later plot: ["sumo_scenarios", "dev/sumo_scenarios"]. Default "sumo_scenarios"',
        default="sumo_scenarios", nargs="?", type=str)
    
    parser.add_argument('--scenario',
        help='Select SUMO scenario to analyze from with <scenario_folder>/: ["onramp", i24", ...]. Default "onramp"',
        default="onramp", nargs="?", type=str)
    
    parser.add_argument('--output_dir',
        help='Select output directory to analyze from with <scenario_folder>/<scenario>/. Default "output"',
        default="output", nargs="?", type=str)
    
    parser.add_argument('--file',
        help='Set file to plot from <scenario_folder>/<scenario>/<output_dir>/ directory. Default "fcd.xml"',
        default="fcd.xml", nargs="?", type=str)
    
    parser.add_argument('--plotting',
        help='Flag to create plots from output file. Default true.', 
            default=True, action='store_true')
    
    parser.add_argument('--saving',
        help='Flag to save plots from output file. Default true.', 
            default=True, action='store_true')
    
    args = parser.parse_args()

    # Select the plot settings
    is_plotting = args.plotting # True/False flag to create plot of fcd file
    is_saving = args.saving # True/False flag to save plot

    # Select the save settings
    save_dir = f'figures/{args.output_dir}' # Directory where plots are to be saved

    ### Select the scenario folder and the output folder for analysis
    scenario_dir = os.path.join(args.scenario_folder, args.scenario)

    assert os.path.isdir(scenario_dir), f'Cannot find scenario directory! {scenario_dir}'

    net_path = os.path.join( scenario_dir, args.scenario + '.net.xml' )

    if not os.path.exists(net_path):
        print(f'Cannot find net file! {net_path}')
        net_path=''

    # Select the output data folder
    output_dir = os.path.join(args.scenario_folder, args.scenario, args.output_dir) # Directory where output file to read is located
    
    assert os.path.isdir(output_dir), f'Cannot find output directory! {output_dir}'

    # Select the output data file
    fcd_path = os.path.join( output_dir, args.file ) # Absolute path to the output file for analysis

    # Initialize metrics with stats file if exists
    assert os.path.exists(fcd_path), f'Cannot find fcd file! {fcd_path}'
    
    ### Initialize the reader class for the XML file
    stats_file = os.path.join(output_dir, 'stats.xml')

    if os.path.exists(stats_file):
        metric = TrafficMetrics(fcd_path, stats_file=stats_file)

    else:
        metric = TrafficMetrics(fcd_path)
        warnings.warn("stats.xml not found, some statistics will be missing!")

    # # Visualize the data
    visualizer = TrafficVisualizationXIL(metric, output_dir, fcd_path, net_file=net_path)

    # Save the figure if is_saving is True
    if is_saving:
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, f'xil_traffic_metrics')
        
        visualizer.plot_xil(save_path=save_path)

    # At least, display the figure if is_plotting is True
    elif is_plotting:
        visualizer.plot_xil()

