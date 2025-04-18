from networkx import graphml

class GPS:
    """
    Graph Positioning System (GPS) class for pathfinding on a graph.
    """

    def __init__(self, path2graphml, start, end):
        self.G = graphml.read_graphml(path2graphml)
        self.start = start
        self.end = end

    def get_node(self, x, y):
        """
        Get the node closest to the given coordinates (x, y).
        """
        closest_node = None
        min_distance = float('inf')

        for node in self.G.nodes(data=True):
            node_x = node[1]['x']
            node_y = node[1]['y']
            distance = ((node_x - x) ** 2 + (node_y - y) ** 2) ** 0.5

            if distance < min_distance:
                min_distance = distance
                closest_node = node[0]

        return closest_node
