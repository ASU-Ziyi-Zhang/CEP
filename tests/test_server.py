################################
### Tyler Ard                ###
### Argonne National Lab     ###
### Vehicle Mobility Systems ###
### tard(at)anl(dot)gov      ###
################################

import unittest
import socket
import json
import time
from src.udp_server import UDPServer

class TestUDPServer(unittest.TestCase):
    def setUp(self):
        """Set up UDP server instance."""
        self.server = UDPServer(ip="127.0.0.1", port=12345, timeout=10, framerate=5)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def test_receive_motion_data(self):
        """Test receiving motion data through UDP."""
        sample_motion_data = json.dumps({
            "x_position": 10.5,
            "y_position": 20.3,
            "speed": 5.6,
            "acceleration": 0.8,
            "heading": 45.0,
            "heading_rate_change": 0.1,
            "current_time": time.time(),
            "error_flag": False
        }).encode("utf-8")

        self.client_socket.sendto(sample_motion_data, ("127.0.0.1", 12345))
        self.assertIsNotNone(self.server.client_last_active)

    def tearDown(self):
        """Close client socket after tests."""
        self.client_socket.close()

if __name__ == "__main__":
    unittest.main()