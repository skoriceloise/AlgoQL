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

    def addEdge(self, idNode1, idNode2, dist):
        self._addEdge(idNode1, idNode2, dist)
        self._addEdge(idNode2, idNode1, dist)

    def _addEdge(self, idNode1, idNode2, dist):
        self.edges.setdefault(idNode1, [])
        self.edges[idNode1].append(idNode2)
        self.distances[(idNode1, idNode2)] = dist