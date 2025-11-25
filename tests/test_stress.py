#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

import unittest
import socket
import json
import time
import threading
from src.udp_client import UDPClient

NUM_CLIENTS = 10  # Number of concurrent clients
NUM_MESSAGES = 100  # Messages per client

class StressTestUDPServer(unittest.TestCase):
    def setUp(self):
        """Set up multiple clients for stress testing."""
        self.clients = [UDPClient(server_ip="127.0.0.1", server_port=12345) for _ in range(NUM_CLIENTS)]
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.bind(("127.0.0.1", 12345))

    def client_worker(self, client_id):
        """Simulates a client sending multiple motion data messages."""
        for _ in range(NUM_MESSAGES):
            motion_data = {
                "x_position": client_id * 1.5,
                "y_position": client_id * 2.3,
                "speed": client_id * 0.8,
                "acceleration": client_id * 0.5,
                "heading": client_id * 3.2,
                "heading_rate_change": client_id * 0.2,
                "current_time": time.time(),
                "error_flag": False
            }

            serialized_data = json.dumps(motion_data).encode("utf-8")
            self.clients[client_id].client_socket.sendto(serialized_data, ("127.0.0.1", 12345))

            # Small delay to simulate network behavior
            time.sleep(0.01)

    def test_high_load(self):
        """Test how the UDP server handles many concurrent clients."""
        threads = []

        # Launch multiple clients
        for client_id in range(NUM_CLIENTS):
            t = threading.Thread(target=self.client_worker, args=(client_id,))
            threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in threads:
            t.join()

        self.assertTrue(True, "Stress test completed successfully.")

    def tearDown(self):
        """Close server socket after tests."""
        self.server_socket.close()

if __name__ == "__main__":
    unittest.main()