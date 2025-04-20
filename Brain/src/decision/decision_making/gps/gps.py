from networkx import graphml, bfs_successors

class GPS:
    """
    Graph Positioning System (GPS) class for pathfinding on a graph.
    """

    def __init__(self, path2graphml, start, end):
        self.G = graphml.read_graphml(path2graphml)
        self.start = start
        self.end = end

        self.path = ["14", "37", "38", "39", "79", "70", "40", "41", "42"]

    def current_node(self, x, y, max_distance=float('inf')):
        """
        Get the current node based on the given coordinates (x, y).
        """

        current_node_id = self._get_node_id_by_pos(x, y, max_distance)
        if current_node_id is None:
            return None
        else:
            self.path.remove(str(current_node_id))
            return self._get_node_w_data_by_id(current_node_id)

    def last(self):
        return len(self.path) == 1
    
    def get_next_node(self, x, y, max_distance=float('inf')):
        """
        Get the next node in the path from the current node.
        """

        return self._get_node_w_data_by_id(self.path[0])

    def distance_to(self, node_id, x, y):
        node = self.G.nodes[node_id]
        node_x = node['x'] * 1000
        node_y = node['y'] * 1000
        
        return ((node_x - x) ** 2 + (node_y - y) ** 2) ** 0.5
        
    def _get_node_w_data_by_id(self, node_id):
        """
        Get the data of a node by its ID.
        """

        if node_id in self.G.nodes:
            node = self.G.nodes[node_id]
            node_x = node['x'] * 1000
            node_y = node['y'] * 1000

            return (node_id, {'x': node_x, 'y': node_y})
        else:
            return None

    def _get_node_id_by_pos(self, x, y, max_distance=float('inf')):
        """
        Get the node closest to the given coordinates (x, y).
        """

        closest_node = None
        min_distance = float('inf')

        for node in self.G.nodes(data=True):
            distance = self.distance_to(node[0], x, y)

            if distance < min_distance and node[0] in self.path and distance < max_distance:
                min_distance = distance
                closest_node = node[0]

        return closest_node
        