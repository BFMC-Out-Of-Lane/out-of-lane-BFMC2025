import unittest
from gps import GPS
from networkx import graphml

class TestGPS(unittest.TestCase):
    def testClosestNode(self):
        g = GPS(path2graphml="./racetrack/test_track_w_roundabout_new.graphml", start="14", end="70")
        initial_x = 600
        initial_y = 3550
        max_distance = 100.0 
        node = g._closest_node(initial_x, initial_y, max_distance)


        self.assertIsNotNone(node, "Expected a node to be returned")

        
if __name__ == '__main__':
    unittest.main()