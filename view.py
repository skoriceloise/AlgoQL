#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pygame, math, sys
from pygame.locals import *
import graph

# INTIALISATION
coleur_station = (0,191,255)
coleur_ecran = (240,255,255)
coleur_arrete = (173,216,230)

WIDTH = 1024
HEIGHT = 768

decalage_w = 0.1 * WIDTH
decalage_h = 0.1 * HEIGHT

screen = pygame.display.set_mode((WIDTH, HEIGHT))
screen.fill(coleur_ecran)

grapheVille = graph.Graph()

grapheVille.addNode(5.0, 0.0)
grapheVille.addNode(6.0, 1.0)
grapheVille.addNode(7.0, 3.0)
grapheVille.addNode(6.0, 4.0)
grapheVille.addNode(4.0, 2.0)
grapheVille.addNode(3.0, 1.0)
grapheVille.addNode(2.0, 4.0)
grapheVille.addNode(1.0, 6.0)

grapheVille.addEdge(0,4, 2.5, 30)
grapheVille.addEdge(0,5, 2.24, 30)
grapheVille.addEdge(1,2, 2.24, 20)
grapheVille.addEdge(2,3, 1.42, 20)
grapheVille.addEdge(3,4, 3.0, 20)
grapheVille.addEdge(2,4, 4.0, 20)
grapheVille.addEdge(4,5, 2.0, 30)
grapheVille.addEdge(5,6, 3.2, 20)
grapheVille.addEdge(6,7, 2.3, 20)

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

#dessin des neuds
for id, n in grapheVille.nodes.iteritems() :
    position = (int(decalage_w + n.x * propor_x) , int(decalage_h + n.y * propor_y))
    pygame.draw.circle(screen, coleur_station, position , 10, 5)

pygame.display.update()

while 1 :
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
             pygame.quit(); sys.exit(); 
