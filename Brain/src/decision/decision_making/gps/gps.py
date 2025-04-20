from networkx import graphml, bfs_successors

class GPS:
    """
    Graph Positioning System (GPS) class for pathfinding on a graph.
    """

    PATH = ["14", "42"]

    def __init__(self, path2graphml, start, end):
        self.G = graphml.read_graphml(path2graphml)
        self.start = start
        self.end = end

    def _get_node_id_by_pos(self, x, y):
        """
        Get the node closest to the given coordinates (x, y).
        """

        closest_node = None
        min_distance = float('inf')

        for node in self.G.nodes(data=True):
            node_x = node[1]['x'] * 1000
            node_y = node[1]['y'] * 1000
            distance = ((node_x - x) ** 2 + (node_y - y) ** 2) ** 0.5

            if distance < min_distance and node[0] in self.PATH:
                min_distance = distance
                closest_node = node[0]

        return closest_node
    
    def get_next_node(self, x, y):
        """
        Get the next node in the path from the current node.
        """

        current_node_id = self._get_node_id_by_pos(x, y)
        idx_of_cn = self.PATH.index(current_node_id)
        if idx_of_cn == len(self.PATH) - 1:
            return self._get_node_w_data_by_id(self.PATH[idx_of_cn])
        else:
            return self._get_node_w_data_by_id(self.PATH[idx_of_cn + 1])
        
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

    def distance_to(self, node_id, x, y):
        node = self.G.nodes[node_id]
        node_x = node['x'] * 1000
        node_y = node['y'] * 1000
        
        return ((node_x - x) ** 2 + (node_y - y) ** 2) ** 0.5
        