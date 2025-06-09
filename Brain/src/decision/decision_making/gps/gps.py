import time

import networkx as nx
import numpy as np
from src.decision.decision_making.gps.node import Node
from src.utils.messages import messageHandlerSender


class GPS:
    """
    Graph Positioning System (GPS) class for pathfinding on a graph.
    """

    def __init__(self, path2graphml: str, mandatory_nodes: list, raw_config_sender: messageHandlerSender):
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

        self.raw_config_sender = raw_config_sender

    def send_to_nucleo(self):
        """
        Send the path to the Nucleo board.
        """

        for i in range(len(self.path)):
            node = self.path[i]
            node_data = self.G.nodes[node]
            x = node_data['x']
            y = node_data['y']
            self.raw_config_sender.send(f"#setNodeList:{i};{int(x)};{int(y)};;\r\n")
            time.sleep(0.01)

    def is_last(self, index: int, offset: int = 1) -> bool:
        """
        Check if the current node is the last node in the path.
        """
        return index == len(self.path) - offset

    def calc_angle_between_previous_n_next_nodes(self, current_node_index: int) -> float:
        prev_node = self.G.nodes[self.path[current_node_index - 1]]
        next_node = self.G.nodes[self.path[current_node_index + 2]]

        return np.arctan2(
            next_node['y'] - prev_node['y'],
            next_node['x'] - prev_node['x']
        ) * 180 / np.pi

    def get_node(self, index: int) -> Node:
        """
        Get the Node object for the given index in the path.
        """
        if 0 <= index < len(self.path):
            node_data = self.G.nodes[self.path[index]]
            return Node(
                id=self.path[index],
                x=node_data['x'],
                y=node_data['y'],
            )
        else:
            raise IndexError("Index out of bounds for the path.")

    def cut_path_from(self, index: int) -> None:
        """
        Cut the path from the given index.
        """
        if 0 <= index < len(self.path):
            self.path = self.path[index:]

    def get_current_node(self, position) -> Node:
        """
        Get the current node based on the position.
        """
        self.min_distance = float('inf')
        self.current_node = None
        for i, node in enumerate(self.path):
            map_node = self.G.nodes[node]
            dist = self.distance(position, map_node)
            if dist < self.min_distance:
                self.min_distance = dist
                self.current_node_index = i
        return self.current_node_index

    def distance(self, current_pos, node_pos) -> float:
        """
        Calculate the distance between the current position and a node position.
        """
        return np.sqrt(
            (node_pos['x'] - float(current_pos['x'])) ** 2 +
            (node_pos['y'] - float(current_pos['y'])) ** 2
        )
