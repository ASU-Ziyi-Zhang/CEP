#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

import unittest
import socket
import json
import time
from src.udp_client import UDPClient

class TestUDPClient(unittest.TestCase):
    def setUp(self):
        """Set up UDP client instance."""
        self.client = UDPClient(server_ip="127.0.0.1", server_port=12345)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.bind(("127.0.0.1", 12345))

    def test_send_motion_data(self):
        """Test sending motion data over UDP."""
        motion_data = {
            "x_position": 10.5,
            "y_position": 20.3,
            "speed": 5.6,
            "acceleration": 0.8,
            "heading": 45.0,
            "heading_rate_change": 0.1,
            "current_time": time.time(),
            "error_flag": False
        }

        serialized_data = json.dumps(motion_data).encode("utf-8")
        self.client.client_socket.sendto(serialized_data, ("127.0.0.1", 12345))

        received_data, _ = self.server_socket.recvfrom(1024)
        deserialized_data = json.loads(received_data.decode("utf-8"))

        self.assertEqual(deserialized_data["x_position"], motion_data["x_position"])
        self.assertEqual(deserialized_data["speed"], motion_data["speed"])

    def tearDown(self):
        """Close server socket after tests."""
        self.server_socket.close()

if __name__ == "__main__":
    unittest.main()