#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

import argparse

def register_parser(parser):
    # Handle command line input arguments
    parser.add_argument("--server_ip", type=str, default="127.0.0.1", help="Server IP address to connect to. Default LOCALHOST")
    parser.add_argument("--server_port", type=int, default=12345, help="Server port. Default 12345")
    parser.add_argument("--framerate", type=int, default=10, help="Message send frequency. Default 10")

    parser.add_argument("--client_ip", type=str, default="127.0.0.1", help="Client IP address. Default LOCALHOST")

    return parser