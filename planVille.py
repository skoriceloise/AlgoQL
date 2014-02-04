import math
import graph

grapheVille = graph.Graph()

grapheVille.addNode(5.0, 0.0)
grapheVille.addNode(6.0, 1.0)
grapheVille.addNode(7.0, 3.0)
grapheVille.addNode(6.0, 4.0)
grapheVille.addNode(4.0, 2.0)
grapheVille.addNode(3.0, 1.0)
grapheVille.addNode(2.0, 4.0)
grapheVille.addNode(1.0, 6.0)

grapheVille.addEdge(0,4, 2.5)
grapheVille.addEdge(0,5, 2.24)
grapheVille.addEdge(1,2, 2.24)
grapheVille.addEdge(2,3, 1.42)
grapheVille.addEdge(3,4, 3.0)
grapheVille.addEdge(2,4, 4.0)
grapheVille.addEdge(4,5, 2.0)
grapheVille.addEdge(5,6, 3.2)
grapheVille.addEdge(6,7, 2.3)

entrepot = 0

clients = (1,2,7)

reseau = (0,4,5)

stations = (4,5)

"""Calcul de la distance euclidienne (heuristique A*) """
def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
"""
def Astar()
    closedset = set()
    openset = set()
    openset.add(start)
    
    came_from = {}
    
    g_score[start] = 0
    f_score[start] = g_score[start] + distance(start, goal)
    
    while openset:
        current = min(openset)
        if current == goal
            return reconstructPath(cameFrom, goal)
        
        openset.remove(current)
               
    

class Client:
    def __init__(self, ident):
        self.ident = ident
        self.distStation = float("inf")

class PlanVille:
    def __init__(self):
    self.plan = grapheVille
    self.calculDijkstra()

    def calculDistance(self):
    for i in range(0,len(grapheVille)-2):
        for j in range(1,len(grapheVille)-3):
            AStar(self.plan, i,j)
"""
for n1 in grapheVille.nodes.values():
    for n2 in grapheVille.nodes.values():
        print("%s - %s : %s" % (n1.idNode, n2.idNode, distance(n1,n2)))