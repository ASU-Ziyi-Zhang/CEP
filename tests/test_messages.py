#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

import unittest
import json
import time
from src.messages import BSM  # Import motion data handler

class TestBSM(unittest.TestCase):
    def setUp(self):
        """Set up sample motion data."""
        self.sample_data = {
            "x_position": 10.5,
            "y_position": 20.3,
            "speed": 5.6,
            "acceleration": 0.8,
            "heading": 45.0,
            "heading_rate_change": 0.1,
            "current_time": time.time(),
            "error_flag": False
        }

    def test_serialization(self):
        """Ensure motion data serializes correctly to JSON."""
        json_data = json.dumps(self.sample_data)
        self.assertIsInstance(json_data, str)

    def test_deserialization(self):
        """Ensure motion data deserializes correctly."""
        json_data = json.dumps(self.sample_data)
        deserialized_data = json.loads(json_data)
        self.assertEqual(deserialized_data["x_position"], self.sample_data["x_position"])
        self.assertEqual(deserialized_data["y_position"], self.sample_data["y_position"])

if __name__ == "__main__":
    unittest.main()