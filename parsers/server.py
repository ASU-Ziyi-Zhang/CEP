#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

import argparse

def register_parser(parser):
    # Handle command line input arguments
    parser.add_argument("--server_ip", type=str, default="127.0.0.1", help="Server IP address. Default LOCALHOST")
    parser.add_argument("--server_port", type=int, default=12345, help="Server port for listening. Default 12345")
    parser.add_argument("--framerate", type=int, default=10, help="Message check frequency. Default 10")
    parser.add_argument("--num_clients", type=int, default=1, help="Expected number of clients to listen for. Default 1")

    return parser