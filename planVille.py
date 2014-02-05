#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import graph
import tsp
import random
from numpy import vstack
from scipy.cluster.vq import kmeans,vq
import pygame, sys
from pygame.locals import *
import colorsys

# INTIALISATION
coleur_station = (0,191,255)
coleur_ecran = (240,255,255)
coleur_arrete = (173,216,230)
couleur_entrepot = (255,255,0)

WIDTH = 1024
HEIGHT = 768

decalage_w = 0.1 * WIDTH
decalage_h = 0.1 * HEIGHT

screen = pygame.display.set_mode((WIDTH, HEIGHT))
screen.fill(coleur_ecran)

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

class Commande:
    def __init__(self, noeud, vol, poids, heure):
        self.noeud = noeud
        self.vol = vol
        self.poids = poids
        self.heure = heure

class Tournee :
    def __init__(self):
        self.chemin = []
        self.distance = float("inf")

    def addNode(self, node):
        self.chemin[node.idNode] = node

class Drone :
    def __init__(self):
        self.tournee = Tournee()
        self.depart = None
        self.retour = None
        self.poids = None
        self.volume = None

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

        self.mDistances = [[0.0 for x in range(nbNodes)] for y in range(nbNodes)]
        for i in range(0, nbNodes):
            for j in range(i + 1, nbNodes):
                if i != j:
                    l = (plusCourtChemin(self.plan, i, j))[0]
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

    nbDrones = 0
    drones = []

    (longueur, chemin) = plusCourtChemin(grapheVille, 1, 6)
    print longueur,
    print " chemin ",
    for n in chemin:
        print n.idNode,
    print

    plan = Plan()
    """
    print "distances"
    for ligne in plan.mDistances:
        print str(ligne).strip('[]')
    """

    print "clients : stations"
    for client in plan.clients:
        print str(plan.clients[client].idClient) + " : " + str(plan.clients[client].stationProche)


    cycle = tsp.greedyTSP(plan.mDistances, [0,2,1,4,5,6,7])
    print "cycle tsp : ",
    print cycle

    #commandes initiales
    commandes = []
    for i in range(10) :
        y = random.choice(clients)
        n = grapheVille.nodes[y]
        poids = random.randrange(50)
        vol = random.randrange(50)
        heure = 0
        c = Commande(n, vol, poids, heure)
        commandes.append(c)


    #kmeans
    nbDrones = 2
    listCoord = vstack((c.noeud.x, c.noeud.y) for c in commandes)
    centroids,_ = kmeans(listCoord,nbDrones)
    idx,_ = vq(listCoord,centroids)



    #affichages
    max_x = max([n.x for k,n in grapheVille.nodes.iteritems()])
    max_y = max([n.y for k,n in grapheVille.nodes.iteritems()])

    propor_x = (WIDTH - 2 * decalage_w) / max_x
    propor_y = (HEIGHT - 2 * decalage_h) / max_y

    #dessin des arrêtes
    for k, n2 in grapheVille.edges.iteritems() :
        c = grapheVille.nodes[k]
        pos1 = (int(decalage_w + c.x * propor_x) , int(decalage_h + c.y * propor_y))
        for n in n2 :
            d = grapheVille.nodes[n]
            pos2 = (int(decalage_w + d.x * propor_x) , int(decalage_h + d.y * propor_y))
            pygame.draw.line(screen, coleur_arrete, pos1, pos2, 3)

    #dessin des noeuds
    for id, n in grapheVille.nodes.iteritems() :
        position = (int(decalage_w + n.x * propor_x) , int(decalage_h + n.y * propor_y))
        pygame.draw.circle(screen, coleur_station, position , 10, 5)

    #dessin entrêpot
    x = int(decalage_w + grapheVille.nodes[entrepot].x * propor_x) - 10
    y = int(decalage_h + grapheVille.nodes[entrepot].y * propor_y) - 10
    rect = pygame.Rect(x ,y , 20, 20)
    pygame.draw.rect(screen, couleur_entrepot, rect, 10)

    #génération de nbDrones couleurs différentes
    HSV_tuples = [(x*1.0/nbDrones, 0.5, 0.5) for x in range(nbDrones)]
    RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)
    RGB_tuples = map(lambda x: tuple(map(lambda y: int(y * 255),x)),RGB_tuples)

    #répartition livraisons
    for i in range(len(listCoord)) :
        y = idx[i]
        print listCoord[i]
        couleur = RGB_tuples[y]
        print couleur
        position = (int(decalage_w + listCoord[i,0] * propor_x) , int(decalage_h + listCoord[i,1] * propor_y))
        pygame.draw.circle(screen, couleur, position , 10, 10)

    pygame.display.update()

    while 1 :
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                 pygame.quit(); sys.exit();






