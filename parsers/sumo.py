#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

import argparse

def register_parser(parser):
    ## Handle command line input arguments
    # Scenario components
    parser.add_argument('--scenario_folder',
        help='Select scenarios folder: ["dev/sumo_scenarios", "sumo_scenarios"]. Default "sumo_scenarios"',
        default="sumo_scenarios", nargs="?", type=str)
    
    parser.add_argument('--scenario',
        help='Select SUMO scenario to run from scenarios folder: ["onramp", "i24", "chi_clinton_canal"]. Default "onramp"',
        default="onramp", nargs="?", type=str)

    parser.add_argument('--penetration',
        help='Set CAV penetration rate with respect to total flow, between 0 and 1. Default 0.0',
        default=0.0, nargs="?", type=float)
    
    parser.add_argument('--seed',
        help='Select the random seed used in SUMO MersenneTwister rng. Default 23423',
        default=23423, nargs='?', type=int) # 23423 is the SUMO default random seed

    # Other components
    parser.add_argument('--gui',
        help='Flag to launch GUI of SUMO. Default false.',
        default=False, action='store_true')

    parser.add_argument('--realtime',
        help='Flag to run real-time SUMO simulation. Default false so as to compute sim as fast as possible.', 
        default=False, action='store_true')
    
    parser.add_argument('--debug',
        help='Flag to debug SUMO simulation. Default false to turn off debugging features.', 
        default=False, action='store_true')
    
    parser.add_argument('--no_inflow',
        help='Flag to run SUMO simulation with no inflow traffic. Default false to turn off debugging features.', 
        default=False, action='store_true')
    
    parser.add_argument('--timestamp_output',
        help='Flag to timestamp the output folders of SUMO simulation. Default false to turn off debugging features.', 
        default=False, action='store_true')
    

    
    return parser