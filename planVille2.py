#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import graph
import tsp
import readXML
import random
import copy
from numpy import vstack
import datetime
from scipy.cluster.vq import kmeans,vq
import pygame, sys
from pygame.locals import *
import colorsys
from pprint import pprint #pour le DEBUG

# INTIALISATION

#constantes affichages
coleur_station = (0,191,255)
coleur_ecran = (240,255,255)
coleur_arrete = (173,216,230)
couleur_entrepot = (255,255,0)
couleur_client = (0,0,255)

WIDTH = 1024
HEIGHT = 768
LIGNE = 10

pygame.font.init()
myfont = pygame.font.SysFont("monospace", 15)

max_x = 0
max_y = 0

propor_x = 0
propor_y = 0

decalage_w = 0.1 * WIDTH
decalage_h = 0.1 * HEIGHT

screen = pygame.display.set_mode((WIDTH, HEIGHT))
screen.fill(coleur_ecran)

#constantes plan
XML_PLAN = 'plan10x10.xml'
XML_LIVR = 'livraison10x10-1.xml'

#clients = [13, 56, 14, 68, 43, 98, 67, 73, 50, 45, 6, 80]

#reseau = list(set(range(100))-set(clients))

stations = [12,7,67,97]

DISTMAX = 25000.0

VOLMAX = 50.0

POIDSMAX = 50.0

NB_DRONES = 5

CLUSTERS = 4

#variables globales
grapheVille = graph.Graph()

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

#Distance d'une commande a une tournee (definie par le min)
def distanceTournee(mDistances, drone, commande):
    minDist = float("inf")
    for c in drone.commandes:
        distance = mDistances[c.noeud][commande.noeud]
        if distance < minDist:
            minDist = distance

    return minDist

def myKmeans(drones,commandes) :
    nbClusters = CLUSTERS
    listCoord = vstack((grapheVille.nodes[c.noeud].x, grapheVille.nodes[c.noeud].y) for c in commandes)
    centroids,_ = kmeans(listCoord,nbClusters)
    idx,_ = vq(listCoord,centroids)
    return (idx, listCoord)

def repartKMeans(drones,commandes, plan) :
    bufferCommandes = []
    nonAffectees = []
    (idx, _) = myKmeans(drones,plan.commandes)

    for c in commandes :
        d = drones[idx[commandes.index(c)]]
        (dist, _, vol, poids) = d.tournee.tryAddCommande(plan,c)
        if dist <= DISTMAX and poids <= POIDSMAX and vol <= VOLMAX :
            d.tournee.addCommande(plan,c)
            d.updateCommandes(c)
        else :
            print "enregistrement dans le buffer "+str(c.noeud)
            bufferCommandes.append(c)

    #repartition des commandes non affectees aux drones (pour les charger au max)
    print "repartitionBuffer"
    repartitionBuffer(drones, plan, bufferCommandes, nonAffectees)
    return nonAffectees

def repartitionBuffer(drones, plan, commandes, nonAffectees) :
    for c in commandes :
        optimum = None
        opt_diffDist = DISTMAX

        for d in drones :
            (dist, diffDist, vol, poids) = d.tournee.tryAddCommande(plan,c)
            if dist < DISTMAX and vol < VOLMAX and poids < POIDSMAX :
                if  diffDist < opt_diffDist : 
                    optimum = d
                    opt_diffDist = diffDist

        if optimum == None : 
            print "impossible d'ajouter"+str(c.noeud)
            if len(drones) < NB_DRONES:
                drones.append(Drone(plan))
                repartitionBuffer(drones, plan, [c], nonAffectees)
            else :
                print "plus assez de drones"
                nonAffectees.append(c)

        else :
            optimum.tournee.addCommande(plan,c)
            d.updateCommandes(c)

def effacer(plan) :
    #dessin des arrêtes
    for k, n2 in plan.plan.edges.iteritems() :
        c = plan.plan.nodes[k]
        pos1 = (int(decalage_w + c.x * propor_x) , int(decalage_h + c.y * propor_y))
        for n in n2 :
            d = plan.plan.nodes[n]
            pos2 = (int(decalage_w + d.x * propor_x) , int(decalage_h + d.y * propor_y))
            pygame.draw.line(screen, coleur_arrete, pos1, pos2, 3)

    #dessin des noeuds
    for id, n in plan.plan.nodes.iteritems() :
        couleur = coleur_station
        if id in plan.clients : couleur = couleur_client
        position = (int(decalage_w + n.x * propor_x) , int(decalage_h + n.y * propor_y))
        pygame.draw.circle(screen, couleur, position , 10, 5)

    #dessin entrêpot
    x = int(decalage_w + plan.plan.nodes[plan.idEntrepot].x * propor_x) - 10
    y = int(decalage_h + plan.plan.nodes[plan.idEntrepot].y * propor_y) - 10
    rect = pygame.Rect(x ,y , 20, 20)
    pygame.draw.rect(screen, couleur_entrepot, rect, 10)

def dessinLivraisons(drones, plan):
    nbDrones = len(drones)
    #génération de nbDrones couleurs différentes
    HSV_tuples = [(x*1.0/nbDrones, 0.5, 0.5) for x in range(nbDrones)]
    RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)
    RGB_tuples = map(lambda x: tuple(map(lambda y: int(y * 255),x)),RGB_tuples)

    #affichage livraisons
    y = 0
    for d in drones :
        couleur = RGB_tuples[y]
    
        """  
        text = 'Drone ' + str(y) + ' : '
        label = myfont.render(text, 1, couleur)
        screen.blit(label, (400 * y, LIGNE))

        text = 'commandes = ' + " - ".join(map(str,[j.noeud for j in d.commandes]))
        label = myfont.render(text, 1, couleur)
        screen.blit(label, (400 * y , LIGNE  * 3))
        """
        
        start = plan.idEntrepot

        #affichage de la tournee sur le reseau
        for idN in d.tournee.cheminReseau:
            n = plan.reseau.nodes[idN]
            position = (int(decalage_w + n.x * propor_x) , int(decalage_h + n.y * propor_y))
            pygame.draw.circle(screen, couleur, position , 10, 10)

            (_, visited) = plusCourtChemin(plan.reseau, start, idN)
            for v in visited :
                if visited.index(v) >= len(visited) - 1 : break
                pos1 = (int(decalage_w + v.x * propor_x) , int(decalage_h + v.y * propor_y))
                node = visited[visited.index(v) + 1]
                pos2 = (int(decalage_w + node.x * propor_x) , int(decalage_h + node.y * propor_y))
                pygame.draw.line(screen, couleur, pos1, pos2, 3)
            start = visited[-1].idNode

        #retour à l'entrêpot
        (_, visited) = plusCourtChemin(plan.reseau, start, plan.idEntrepot)
        for v in visited :
            if visited.index(v) >= len(visited) - 1 : break
            pos1 = (int(decalage_w + v.x * propor_x) , int(decalage_h + v.y * propor_y))
            node = visited[visited.index(v) + 1]
            pos2 = (int(decalage_w + node.x * propor_x) , int(decalage_h + node.y * propor_y))
            pygame.draw.line(screen, couleur, pos1, pos2, 3)

        #affichage des sous-tournees
        for (st, trajet) in d.tournee.cheminStations.iteritems() :
            start = st
            for idN in trajet:
                n = plan.plan.nodes[idN]
                position = (int(decalage_w + n.x * propor_x) , int(decalage_h + n.y * propor_y))
                pygame.draw.circle(screen, couleur, position , 10, 10)

                (_, visited) = plusCourtChemin(plan.plan, start, idN)
                for v in visited :
                    if visited.index(v) >= len(visited) - 1 : break
                    pos1 = (int(decalage_w + v.x * propor_x) , int(decalage_h + v.y * propor_y))
                    d = visited[visited.index(v) + 1]
                    pos2 = (int(decalage_w + d.x * propor_x) , int(decalage_h + d.y * propor_y))
                    pygame.draw.line(screen, couleur, pos1, pos2, 3)
                start = visited[-1].idNode

            #retour à la station
            (_, visited) = plusCourtChemin(plan.plan, start, plan.idEntrepot)
            for v in visited :
                if visited.index(v) >= len(visited) - 1 : break
                pos1 = (int(decalage_w + v.x * propor_x) , int(decalage_h + v.y * propor_y))
                d = visited[visited.index(v) + 1]
                pos2 = (int(decalage_w + d.x * propor_x) , int(decalage_h + d.y * propor_y))
                pygame.draw.line(screen, couleur, pos1, pos2, 3)


            """
            for idN,_ in d.tournee.cheminStations.iteritems() :
                n = plan.plan.nodes[idN]
                position = (int(decalage_w + n.x * propor_x) , int(decalage_h + n.y * propor_y))
                pygame.draw.circle(screen, couleur, position , 10, 10)

                (_, visited) = plusCourtChemin(plan.reseau, start, idN)
                for v in visited :
                    if visited.index(v) >= len(visited) - 1 : break
                    pos1 = (int(decalage_w + v.x * propor_x) , int(decalage_h + v.y * propor_y))
                    d = visited[visited.index(v) + 1]
                    pos2 = (int(decalage_w + d.x * propor_x) , int(decalage_h + d.y * propor_y))
                    pygame.draw.line(screen, couleur, pos1, pos2, 3)
                start = visited[-1].idNode
            #retour à l'entrêpot
            (_, visited) = plusCourtChemin(plan.plan, start, plan.idEntrepot)
            for v in visited :
                if visited.index(v) >= len(visited) - 1 : break
                pos1 = (int(decalage_w + v.x * propor_x) , int(decalage_h + v.y * propor_y))
                d = visited[visited.index(v) + 1]
                pos2 = (int(decalage_w + d.x * propor_x) , int(decalage_h + d.y * propor_y))
                pygame.draw.line(screen, couleur, pos1, pos2, 3)
                """
        y += 1

    pygame.display.update()

class Client:
    def __init__(self, idClient):
        self.idClient = idClient
        self.distStation = float("inf")
        self.stationProche = None

class Commande:
    def __init__(self, noeud, vol, poids, heure):
        self.noeud = noeud #id noeud
        self.vol = vol
        self.poids = poids
        self.heure = heure

class Tournee :
    def __init__(self, idEntrepot):
        self.cheminReseau = [idEntrepot] #tournee des stations
        self.cheminStations = {plan.idEntrepot : [plan.idEntrepot]} #sous-tournee des livraisons pour chaque station
        self.distance = 0.0
        self.poids = 0.0
        self.volume = 0.0

    def addCommande(self, plan, commande):
        #Recherche si la station est deja dans la tournee
        station = plan.clients[commande.noeud].stationProche 
        if station in self.cheminReseau :
            #Si la station est dans la tournee, ajout de la commande a la 
            #sous-tournee
            self.distance = tsp.insertNodeTSP(plan.mDistancesVille, commande.noeud, self.cheminStations[station], self.distance)
        else:
            #Sinon
            #ajout de la station a la tournee
            self.distance = tsp.insertNodeTSP(plan.mDistancesReseau, station, self.cheminReseau, self.distance)
            #creation de la sous-tournee
            (self.cheminStations[station], d) = tsp.greedyTSP(plan.mDistancesVille, [station, commande.noeud])
            self.distance += d
        self.poids += commande.poids
        self.volume += commande.vol
                

    def tryAddCommande(self, plan, commande):
        #Recherche si la station est deja dans la tournee
        station = plan.clients[commande.noeud].stationProche 

        #copie de la tournee pour tester l'ajout
        cpCheminReseau = copy.copy(self.cheminReseau)
        cpCheminStations = copy.deepcopy(self.cheminStations)


        if station in cpCheminReseau :
            #Si la station est dans la tournee, ajout de la commande a la 
            #sous-tournee
            distance = tsp.insertNodeTSP(plan.mDistancesVille, commande.noeud, cpCheminStations[station], self.distance)
        else:
            #Sinon
            #ajout de la station a la tournee
            distance = tsp.insertNodeTSP(plan.mDistancesReseau, station, cpCheminReseau, self.distance)
            #creation de la sous-tournee
            (cpCheminStations[station], d) = tsp.greedyTSP(plan.mDistancesVille, [station, commande.noeud])
            distance += d
        diffDist = distance - self.distance
        poids = self.poids + commande.poids
        volume = self.volume + commande.vol
        """
        print "resultat de l'ajout try"
        print cpCheminReseau
        print pprint(cpCheminStations)
        print pprint(self.cheminStations)
        """
        return (distance, diffDist, poids, volume)

    def annulerTournee(self, idEntrepot):
        self.cheminReseau = [idEntrepot] 
        self.cheminStations = {idEntrepot : idEntrepot}
        self.distance  = 0
        self.poids = 0
        self.volume = 0

class Drone :
    def __init__(self, plan):
        self.tournee = Tournee(plan.idEntrepot)
        self.commandes = []

    def updateCommandes(self, commande):
        self.commandes.append(commande)

class Plan:
    def __init__(self):
        #Graphe general de la ville
        self.plan = grapheVille
        self.reseau = grapheReseau
        self.clients = {}
        (self.commandes, entrepot) = readXML.lectureCommandesXML(XML_LIVR, grapheVille)
        self.idEntrepot = entrepot


        nbNodesRes = len(self.reseau.nodes)
        self.mDistancesReseau = [[float("inf") for x in range(100)] for y in range(100)]
        for n1 in self.reseau.nodes:
            for n2 in self.reseau.nodes:
                    l = (plusCourtChemin(self.reseau, n1, n2))[0]
                    self.mDistancesReseau[n1][n2] = l

    def createClients(self) :
        #Creation des clients a livrer
        noeudsComm = [c.noeud for c in self.commandes]
        for idClient in noeudsComm:
            if idClient not in self.clients.keys():
                self.clients[idClient] = Client(idClient)
        self.stations = stations 
        print self.stations

        #Calcul des distances entre les noeuds du graphe de la ville
        nbNodes = len(self.plan.nodes)

        self.mDistancesVille = [[0.0 for x in range(nbNodes)] for y in range(nbNodes)]
        print 'station plus proche'
        for i in range(nbNodes):
            for j in range(i, nbNodes):
                    l = (plusCourtChemin(self.plan, i, j))[0]
                    self.mDistancesVille[i][j] = l
                    self.mDistancesVille[j][i] = l
                    
                    #Enregistrement de la station la plus proche pour un client
                    if i in self.clients.keys() and j in self.stations: 
                        if self.clients[i].distStation > l:
                            self.clients[i].stationProche = j
                            self.clients[i].distStation = l

                    elif i in self.stations and j in self.clients.keys():
                        if self.clients[j].distStation > l:
                            self.clients[j].stationProche = i
                            self.clients[j].distStation = l

    def addReseauUrbain(self, reseau):
        #Graphe du reseau urbain
        self.reseau = graph.Graph()
        for idReseau in reseau:
            self.reseau.addNodeObject(self.plan.nodes[idReseau])
                
if __name__ == '__main__':

    #recuperation du plan et des commandes des fichiers XML
    (grapheVille, grapheReseau) = readXML.lecturePlanXML(XML_PLAN)

    """
    for n1 in grapheVille.nodes.values():
        for n2 in grapheVille.nodes.values():
            print("%s - %s : %s" % (n1.idNode, n2.idNode, distance(n1,n2)))
    """
    plan = Plan()

    #reseau = list(set(range(100))-set(plan.clients))
    #plan.addReseauUrbain(reseau)

    drones = [Drone(plan) for x in range(0,CLUSTERS)]
 
    stations = (11,41,70,72)
    plan.createClients()
    (longueur, chemin) = plusCourtChemin(grapheVille, 0, 30)

    """
    print "distances"
    for ligne in plan.mDistances:
        print str(ligne).strip('[]')
    """

    print "clients : stations"
    for client in plan.clients:
        print str(plan.clients[client].idClient) + " : " + str(plan.clients[client].stationProche)


    cycle = tsp.greedyTSP(plan.mDistancesVille, [0,2,1,4,5,6,7])
    print "cycle tsp : ",
    print cycle

    max_x = max([n.x for k,n in plan.plan.nodes.iteritems()])
    max_y = max([n.y for k,n in plan.plan.nodes.iteritems()])

    propor_x = (WIDTH - 3 * decalage_w) / max_x
    propor_y = (HEIGHT - 3 * decalage_h) / max_y

    effacer(plan)
    while 1 :
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit();
            if event.type == KEYDOWN and event.key == K_RETURN :
                effacer(plan)
                cRestantes = repartKMeans(drones, plan.commandes, plan)
                dessinLivraisons(drones,plan)
                print "resultat final"
                for d in drones:
                    if len(d.commandes) != 0:
                        print "drone : "+str(d)
                        print "charge : " + str(d.tournee.poids),
                        print " volume : " + str(d.tournee.volume),
                        print " distance : " + str(d.tournee.distance),
                        print d.tournee.cheminReseau
                        print pprint(d.tournee.cheminStations)
                print "Commandes restantes "+str(list(c.noeud for c in cRestantes))



