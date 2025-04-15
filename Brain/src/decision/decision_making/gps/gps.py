from networkx import graphml

class GPS:
    """
    Graph Positioning System (GPS) class for pathfinding on a graph.
    """

    # TODO: This path should be calculated based on the start and end nodes
    PATH = [6,7,8,9,10,11,12,13,37,38,39,40,41,42,29,30,31,32,33,34]

    def __init__(self, path2graphml, start, end):
        self.G = graphml.read_graphml(path2graphml)
        self.start = start
        self.end = end
