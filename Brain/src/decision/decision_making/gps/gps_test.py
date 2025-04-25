import unittest
from networkx import graphml

class TestGPS(unittest.TestCase):
    def testCurrentNode(self):
        nodeMap =  graphml.read_graphml(path2graphml="./racetrack/test_track_w_roundabout.graphml")


        
if __name__ == '__main__':
    unittest.main()