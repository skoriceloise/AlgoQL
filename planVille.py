import math
import graph
import tsp

grapheVille = graph.Graph()

grapheVille.addNode(5.0, 0.0) #noeud 0
grapheVille.addNode(6.0, 1.0)
grapheVille.addNode(7.0, 3.0)
grapheVille.addNode(6.0, 4.0)
grapheVille.addNode(4.0, 2.0)
grapheVille.addNode(3.0, 1.0)
grapheVille.addNode(2.0, 4.0)
grapheVille.addNode(1.0, 6.0) #noeud 7

grapheVille.addEdge(0,4, 2.5, 30)
grapheVille.addEdge(0,5, 2.24, 30)
grapheVille.addEdge(1,2, 2.24, 20)
grapheVille.addEdge(2,3, 1.42, 20)
grapheVille.addEdge(3,4, 3.0, 20)
grapheVille.addEdge(2,4, 4.0, 20)
grapheVille.addEdge(4,5, 2.0, 30)
grapheVille.addEdge(5,6, 3.2, 20)
grapheVille.addEdge(6,7, 2.3, 20)

entrepot = 0

clients = [1,2,7]

reseau = [0,4,5]

stations = [4,5]

"""Calcul de la distance euclidienne (heuristique A*) """
def distance(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def AStar(graph, start, goal):
    closedset = set()
    openset = set()
    openset.add(start)
    
    came_from = {}
    
    visited = {}
    g_score = {start : 0}
    f_score = {start : distance(start, goal)}
    
    while openset:
        
        current = None
        for node in openset:
            if current is None:
                current = node
            elif f_score[node] < f_score[current]:
                current = node

        openset.remove(current)
        if current == goal:
            return (g_score[goal], visited)

        closedset.add(current)

        for idNeighbor in graph.edges[current.idNode]:
            neighbor = graph.nodes[idNeighbor]
            if neighbor in closedset:
                continue
            tentative_g_score = g_score[current] + graph.distances[(current.idNode, idNeighbor)]

            if neighbor not in openset or tentative_g_score < g_score[neighbor]:
                openset.add(neighbor)
                visited[neighbor] = current

                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + distance(neighbor, goal)

    return False
    
def plusCourtChemin(graphe, depart, arrivee):
    nodeDepart = graphe.nodes[depart]
    nodeArrivee = graphe.nodes[arrivee]
    (longueur, chemin) = AStar(graphe, nodeDepart, nodeArrivee)
    route = [nodeArrivee]

    while nodeArrivee != nodeDepart:
        route.append(chemin[nodeArrivee])
        nodeArrivee = chemin[nodeArrivee]

    route.reverse()
    return (longueur, route)

class Client:
    def __init__(self, idClient):
        self.idClient = idClient
        self.distStation = float("inf")
        self.stationProche = None

class Plan:
    def __init__(self):
        #Graphe general de la ville
        self.plan = grapheVille

        #Graphe du reseau urbain
        self.idEntrepot = entrepot
        self.reseau = graph.Graph()
        for idReseau in reseau:
            self.reseau.addNodeObject(self.plan.nodes[idReseau])

        #Creation des clients a livrer
        self.clients = {}
        for idClient in clients:
            self.clients[idClient] = Client(idClient)
        self.stations = stations 

        #Calcul des distances entre les noeuds du graphe de la ville
        nbNodes = len(self.plan.nodes)
        print nbNodes
        self.mDistances = [[0.0 for x in range(nbNodes)] for y in range(nbNodes)]
        for i in range(0, nbNodes):
            for j in range(i + 1, nbNodes):
                if i != j:
                    l = (plusCourtChemin(self.plan, i, j))[0]
                    print str(i) + "-" + str(j) + " : " + str(l)
                    self.mDistances[i][j] = l
                    self.mDistances[j][i] = l

                    #Enregistrement de la station la plus proche pour un client
                    if i in self.clients and j in self.stations:
                        if self.clients[i].distStation > l:
                            self.clients[i].stationProche = j
                            self.clients[i].distStation = l
                    elif i in self.stations and j in self.clients:
                            self.clients[j].stationProche = i
                            self.clients[j].distStation = l
                
if __name__ == '__main__':

    """
    for n1 in grapheVille.nodes.values():
        for n2 in grapheVille.nodes.values():
            print("%s - %s : %s" % (n1.idNode, n2.idNode, distance(n1,n2)))
    """

    (longueur, chemin) = plusCourtChemin(grapheVille, 1, 6)
    print longueur
    print "chemin"
    for n in chemin:
        print n.idNode

    plan = Plan()
    print "distances"
    for ligne in plan.mDistances:
        print str(ligne).strip('[]')

    print "clients : stations"
    for client in plan.clients:
        print str(plan.clients[client].idClient) + " : " + str(plan.clients[client].stationProche)

    cycle = tsp.greedyTSP(plan.mDistances, [0,2,1,4,5,6,7])
    print "cycle tsp"
    print cycle
