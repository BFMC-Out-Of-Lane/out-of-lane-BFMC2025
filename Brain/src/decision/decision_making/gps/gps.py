from networkx import graphml, bfs_successors

class GPS:
    """
    Graph Positioning System (GPS) class for pathfinding on a graph.
    """

    PATH = ["14", "37", "38", "39", "79", "70", "40", "41", "42"]

    def __init__(self, path2graphml, start, end):
        self.G = graphml.read_graphml(path2graphml)
        self.start = start
        self.end = end

    def current_node(self, x, y, max_distance=float('inf')):
        """
        Get the current node based on the given coordinates (x, y).
        """

        current_node_id = self._get_node_id_by_pos(x, y, max_distance)
        return self._get_node_w_data_by_id(current_node_id)

    def last(self):
        return self.PATH[len(self.PATH) - 1]
    
    def get_next_node(self, x, y):
        """
        Get the next node in the path from the current node.
        """

        current_node_id = self._get_node_id_by_pos(x, y)
        print(current_node_id)
        idx_of_cn = self.PATH.index(current_node_id)
        if idx_of_cn == len(self.PATH) - 1:
            return self._get_node_w_data_by_id(self.PATH[idx_of_cn])
        else:
            return self._get_node_w_data_by_id(self.PATH[idx_of_cn + 1])

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

            if distance < min_distance and node[0] in self.PATH and distance < max_distance:
                min_distance = distance
                closest_node = node[0]

        return closest_node
        