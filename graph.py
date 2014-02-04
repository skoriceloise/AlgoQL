class Node:
    def __init__(self, idNode, x, y):
        self.idNode = idNode
        self.x = x
        self.y = y

class Graph:
    def __init__(self):
        self.nodes = {}
        self.edges = {}
        self.distances = {}

    def addNode(self, x, y):
        idNode = len(self.nodes)
        self.nodes[idNode] = Node(idNode, x, y)

    def addEdge(self, node1, node2, dist):
        self._addEdge(node1, node2, dist)
        self._addEdge(node2, node1, dist)

    def _addEdge(self, node1, node2, dist):
        self.edges.setdefault(node1, [])
        self.edges[node1].append(node2)
        self.distances[(node1, node2)] = dist