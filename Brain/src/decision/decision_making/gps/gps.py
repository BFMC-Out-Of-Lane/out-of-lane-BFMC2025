import time

import networkx as nx
from src.decision.decision_making.gps.node import Node


class GPS:
    """
    Graph Positioning System (GPS) class for pathfinding on a graph.
    """

    def __init__(self, path2graphml: str, mandatory_nodes: list):
        self.G = nx.graphml.read_graphml(path2graphml)
        for node_index, node in self.G.nodes(data=True):
            self.G.nodes[node_index]['x'] = node['x'] * 1000
            self.G.nodes[node_index]['y'] = node['y'] * 1000

        self.path = []
        for i in range(len(mandatory_nodes) - 1):
            if mandatory_nodes[i] in self.G.nodes and mandatory_nodes[i + 1] in self.G.nodes:
                self.path += nx.shortest_path(self.G, mandatory_nodes[i], mandatory_nodes[i + 1])
            else:
                raise ValueError(f"Node {mandatory_nodes[i]} not found in the graph.")

        self.original_path = self.path.copy()
        self.current_time = time.time()

    def current_node(self, x: float, y: float, max_distance=float('inf')) -> (Node | None):
        """
        Get the current node based on the given coordinates (x, y).
        """
        current_node_id = self._closest_node(x, y, max_distance)
        if current_node_id is not None:
            self.path.remove(str(current_node_id))
            return self._get_node_by_id(current_node_id)
        else:
            return None

    def last(self):
        return len(self.path) == 0

    def get_next_node(self) -> (Node | None):
        """
        Get the next node in the path from the current node.
        """

        if len(self.path) != 0:
            return self._get_node_by_id(self.path[0])
        else:
            return None

    def vector_to_node(self, x, y, node_id):
        """
        Get the difference between two nodes.
        """

        node = self.G.nodes[node_id]

        return (node['x'] - x, node['y'] - y)

    def distance_to(self, node_id, x, y):
        node = self.G.nodes[node_id]
        node_x = node['x']
        node_y = node['y']

        return ((node_x - x) ** 2 + (node_y - y) ** 2) ** 0.5

    def reset(self):
        """
        Reset the map to its original state.
        """

        self.path = self.original_path.copy()
        self.current_time = time.time()

    def _get_node_by_id(self, node_id) -> (Node | None):
        """
        Get the data of a node by its ID.
        """

        if node_id in self.G.nodes:
            node = self.G.nodes[node_id]
            node_x = node['x']
            node_y = node['y']

            return Node(node_id, node_x, node_y)
        else:
            return None

    def _closest_node(self, x: float, y: float, max_distance=float('inf')):
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

    def _actual_closest_node(self, x: float, y: float, max_distance=float('inf')):
        """
        Get the node closest to the given coordinates (x, y) without considering the path.
        """

        closest_node = None
        for node in self.G.nodes(data=True):
            distance = self.distance_to(node[0], x, y)
            if distance < max_distance:
                max_distance = distance
                closest_node = node[0]

        if time.time() - self.current_time > 1:
            self.current_time = time.time()

        return closest_node
