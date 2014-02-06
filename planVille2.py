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

NB_DRONES = 150

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
    nbDrones = len(drones)
    listCoord = vstack((grapheVille.nodes[c.noeud].x, grapheVille.nodes[c.noeud].y) for c in commandes)
    centroids,_ = kmeans(listCoord,nbDrones)
    idx,_ = vq(listCoord,centroids)
    return (idx, listCoord)

def repartKMeans(drones,commandes, plan) :
    bufferCommandes = []
    (idx, _) = myKmeans(drones,plan.commandes)

    for c in commandes :
        d = drones[idx[commandes.index(c)]]
        print "j'essaie d'ajouter"
        (dist, _, vol, poids) = d.tournee.tryAddCommande(plan,c)
        if dist <= DISTMAX and poids <= POIDSMAX and vol <= VOLMAX :
            print "j'ajoute pour de vrai"
            d.tournee.addCommande(plan,c)
            d.updateCommandes(c)
        else :
            print "j'enregistre dans le buffer"
            bufferCommandes.append(c)

    #repartition des commandes non affectees aux drones (pour les charger au max)
    repartitionBuffer(drones, plan, bufferCommandes)
    return bufferCommandes

def repartitionBuffer(drones, plan, commandes) :
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

        else :
            print "veritable ajout dans repartition"
            optimum.tournee.addCommande(plan,c)
            d.updateCommandes(c)


def dessinLivraisons(drones, plan):
    nbDrones = len(drones)
    #génération de nbDrones couleurs différentes
    HSV_tuples = [(x*1.0/nbDrones, 0.5, 0.5) for x in range(nbDrones)]
    RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples)
    RGB_tuples = map(lambda x: tuple(map(lambda y: int(y * 255),x)),RGB_tuples)

    #affichage livraisons
    y = 0
    for d in drones :
        for idN,_ in d.tournee.cheminStations.iteritems() :
            couleur = RGB_tuples[y]
            n = plan.plan.nodes[idN]
            position = (int(decalage_w + n.x * propor_x) , int(decalage_h + n.y * propor_y))
            pygame.draw.circle(screen, couleur, position , 10, 10)
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
        self.cheminStations = {plan.idEntrepot : plan.idEntrepot} #sous-tournee des livraisons pour chaque station
        self.distance = 0.0
        self.poids = 0.0
        self.volume = 0.0

    def addCommande(self, plan, commande):
        #Recherche si la station est deja dans la tournee
        station = plan.clients[commande.noeud].stationProche 
        if station in self.cheminReseau :
            #Si la station est dans la tournee, ajout de la commande a la 
            #sous-tournee
            self.distance = tsp.insertNodeTSP(plan.mDistances, commande.noeud, self.cheminStations[station], self.distance)
        else:
            #Sinon
            #ajout de la station a la tournee
            self.distance = tsp.insertNodeTSP(plan.mDistances, station, self.cheminReseau, self.distance)
            #creation de la sous-tournee
            (self.cheminStations[station], d) = tsp.greedyTSP(plan.mDistances, [station, commande.noeud])
            self.distance += d
        self.poids += commande.poids
        self.volume += commande.vol
        
        print "resultat de l'ajout veritable"
        print self.cheminReseau
        print pprint(self.cheminStations)
        

    def tryAddCommande(self, plan, commande):
        #Recherche si la station est deja dans la tournee
        station = plan.clients[commande.noeud].stationProche 

        #copie de la tournee pour tester l'ajout
        cpCheminReseau = copy.copy(self.cheminReseau)
        cpCheminStations = copy.deepcopy(self.cheminStations)


        #affichage debug
        print "ajout de "+str(commande.noeud)+", s="+str(station)+" chemin reseau actuel: ",
        print cpCheminReseau
        print pprint(cpCheminStations)


        if station in cpCheminReseau :
            #Si la station est dans la tournee, ajout de la commande a la 
            #sous-tournee
            distance = tsp.insertNodeTSP(plan.mDistances, commande.noeud, cpCheminStations[station], self.distance)
        else:
            #Sinon
            #ajout de la station a la tournee
            distance = tsp.insertNodeTSP(plan.mDistances, station, cpCheminReseau, self.distance)
            #creation de la sous-tournee
            (cpCheminStations[station], d) = tsp.greedyTSP(plan.mDistances, [station, commande.noeud])
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
        self.clients = {}
        (self.commandes, entrepot) = readXML.lectureCommandesXML(XML_LIVR, grapheVille)
        self.idEntrepot = entrepot

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

        self.mDistances = [[0.0 for x in range(nbNodes)] for y in range(nbNodes)]
        print 'station plus proche'
        for i in range(nbNodes):
            for j in range(i + 1, nbNodes):
                if i != j:
                    l = (plusCourtChemin(self.plan, i, j))[0]
                    self.mDistances[i][j] = l
                    self.mDistances[j][i] = l
                    
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
    grapheVille = readXML.lecturePlanXML(XML_PLAN)

    """
    for n1 in grapheVille.nodes.values():
        for n2 in grapheVille.nodes.values():
            print("%s - %s : %s" % (n1.idNode, n2.idNode, distance(n1,n2)))
    """
    plan = Plan()

    reseau = list(set(range(100))-set(plan.clients))
    plan.addReseauUrbain(reseau)

    drones = [Drone(plan) for x in range(0,150)]
    print len(drones)
    stations = list(set(reseau) - set([plan.idEntrepot]) - set((range(10,30))))
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


    cycle = tsp.greedyTSP(plan.mDistances, [0,2,1,4,5,6,7])
    print "cycle tsp : ",
    print cycle

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
        couleur = coleur_station
        if id in plan.clients : couleur = couleur_client
        position = (int(decalage_w + n.x * propor_x) , int(decalage_h + n.y * propor_y))
        pygame.draw.circle(screen, couleur, position , 10, 5)

    #dessin entrêpot
    x = int(decalage_w + grapheVille.nodes[plan.idEntrepot].x * propor_x) - 10
    y = int(decalage_h + grapheVille.nodes[plan.idEntrepot].y * propor_y) - 10
    rect = pygame.Rect(x ,y , 20, 20)
    pygame.draw.rect(screen, couleur_entrepot, rect, 10)

    while 1 :
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit();
            if event.type == KEYDOWN and event.key == K_RETURN :
                #(idx,listCoord) = myKmeans(drones,plan.commandes)

                repartKMeans(drones, plan.commandes, plan)
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



