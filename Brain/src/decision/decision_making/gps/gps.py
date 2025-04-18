from networkx import graphml, bfs_successors

class GPS:
    """
    Graph Positioning System (GPS) class for pathfinding on a graph.
    """

    def __init__(self, path2graphml, start, end):
        self.G = graphml.read_graphml(path2graphml)
        self.start = start
        self.end = end

    def get_node_by_pos(self, x, y):
        """
        Get the node closest to the given coordinates (x, y).
        """

        closest_node = None
        min_distance = float('inf')

        for node in self.G.nodes(data=True):
            node_x = node[1]['x'] * 1000
            node_y = node[1]['y'] * 1000
            distance = ((node_x - x) ** 2 + (node_y - y) ** 2) ** 0.5

            if distance < min_distance:
                min_distance = distance
                closest_node = node[0]

        return closest_node

    def get_node(self, current_node_id, x, y):
        """
        Get the node closest to the given coordinates (x, y).
        """

        closest_node = None
        min_distance = float('inf')

        for node in self.G.neighbors(self.G.nodes[current_node_id]):
            node_x = node[1]['x'] * 1000
            node_y = node[1]['y'] * 1000
            distance = ((node_x - x) ** 2 + (node_y - y) ** 2) ** 0.5

            if distance < min_distance:
                min_distance = distance
                closest_node = node[0]

        return closest_node
