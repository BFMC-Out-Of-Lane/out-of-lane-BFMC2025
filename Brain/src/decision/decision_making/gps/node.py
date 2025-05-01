class Node:
    """
    Represents a node in the graph
    """

    def __init__(self, id, x, y):
        self.id: str = id
        self.x: float = x
        self.y: float = y
        self.edges = []

    # def distance_to(self, other_node):
    #     """
    #     Calculate the distance to another node
    #     """
    #     return ((self.x - other_node.x) ** 2 + (self.y - other_node.y) ** 2) ** 0.5


# class Map:
#     """
#     Represents the network of nodes
#     """

#     def __init__(self, path2graphml):
#         """
#         Initialize the map with a graphml file
#         """
#         graphML = graphml.read_graphml(path2graphml)
#         self.nodes = {} # Dictionary to hold nodes with id as key

#         for node_id, data in graphML.nodes(data=True):
#             x = data['x'] * 1000
#             y = data['y'] * 1000
#             self.nodes[node_id] = Node(node_id, x, y)

#         self.current_node = Node()
#         self.path = []
#         self.path.append(Node(),Node()) ## Example of path structure

#     def add_node(self, id, x, y):
#         """
#         Add a node to the map
#         """
#         if id not in self.nodes:
#             self.nodes[id] = Node(id, x, y)

#     def get_closest_node(self, x, y):
#         """
#         Get the closest node to a given point (x, y)
#         """
#         closest_node = Node()
#         min_distance = float('inf')

#         for node in self.nodes.values():
#             distance = ((node.x - x) ** 2 + (node.y - y) ** 2) ** 0.5
#             if distance < min_distance:
#                 min_distance = distance
#                 closest_node = node

#         return closest_node
