#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

import argparse

def register_parser(parser):
    ## Handle command line input arguments
    # SUMO backend DLL options
    MODULES = [
        "traci", # traci has full API options
        "libsumo", # libsumo is faster but no GUI support and select API options are limited
        #
        "polarislibsumo", # polaris has a DLL for SUMO API we can link against
        "polaristraci", # polaris has a DLL for SUMO API we can link against
    ]

    parser.add_argument("--backend", choices=MODULES, default="traci")

    # Scenario components
    parser.add_argument('--scenario_folder',
        help='Select scenarios folder: ["dev/sumo_scenarios", "sumo_scenarios"]. Default "sumo_scenarios"',
        default="sumo_scenarios", nargs="?", type=str)
    
    parser.add_argument('--scenario',
        help='Select SUMO scenario to run from scenarios folder: ["onramp", "i24", "roosevelt"]. Default "onramp"',
        default="onramp", nargs="?", type=str)

    parser.add_argument('--penetration',
        help='Set CAV penetration rate with respect to total flow, between 0 and 1. Default 0.0',
        default=0.0, nargs="?", type=float)
    
    parser.add_argument('--seed',
        help='Select the random seed used in SUMO MersenneTwister rng. Default 23423',
        default=23423, nargs='?', type=int) # 23423 is the SUMO default random seed

    parser.add_argument('--time_to_teleport',
        help='Override SUMO time-to-teleport in seconds. Use -1 to disable teleport. Default: 300 for traffic-light scenarios, -1 otherwise.',
        default=None, nargs='?', type=float)

    parser.add_argument('--max_sim_time',
        help='Hard cap on simulation time in seconds. Simulation exits when exceeded, even if vehicles remain. Default: 0 (disabled).',
        default=0, nargs='?', type=float)

    # Control components
    parser.add_argument('--no_cav_onramp',
        help='Flag to disable CAV controls when vehicles are at an onramp. Default false to avoid switching to human driver model on onramps.',
        default=False, action='store_true')
    
    parser.add_argument('--no_cav_lanechange',
        help='Flag to disable CAV lane change. Default false to allow lane changing using human driver model.',
        default=False, action='store_true')
    
    # Replay components
    parser.add_argument('--replay',
        help='Flag to replay a previous SUMO simulation. Default false to run a new simulation.',
        default=False, action='store_true')
    
    parser.add_argument('--replay_dir',
        help='Select output directory to analyze from with <scenario_folder>/<scenario>/<replay_dir>. Default "output"',
        default="output", nargs="?", type=str)

    parser.add_argument('--replay_file',
        help='Set file to plot from <scenario_folder>/<scenario>/<replay_dir>/<replay_file> directory. Default "fcd.xml"',
        default="fcd.xml", nargs="?", type=str)
    
    # Other components
    parser.add_argument('--gui',
        help='Flag to launch GUI of SUMO. Default false.',
        default=False, action='store_true')

    parser.add_argument('--realtime',
        help='Flag to run real-time SUMO simulation. Default false to instead compute simulation as fast as possible.', 
        default=False, action='store_true')
    
    parser.add_argument('--no_inflow',
        help='Flag to run SUMO simulation with no inflow traffic. Default false to allow inflows.', 
        default=False, action='store_true')
    
    parser.add_argument('--timestamp_output',
        help="Flag to timestamp the output folders of SUMO simulation. Default false to dump to single 'output' folder.", 
        default=False, action='store_true')
    
    parser.add_argument('--log_tl',
        help='Flag to log SUMO simulation in traffic light scenarios. Default false to turn off logging features.', 
        default=False, action='store_true')
    
    parser.add_argument('--debug',
        help='Flag to debug SUMO simulation. Default false to turn off debugging features.', 
        default=False, action='store_true')
    
    return parser
