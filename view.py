#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pygame, math, sys
from pygame.locals import *
import graph

# INTIALISATION
coleur_station = (255,0,0)

WIDTH = 1024
HEIGHT = 768

decalage_w = 0.1 * WIDTH
decalage_h = 0.1 * HEIGHT

screen = pygame.display.set_mode((WIDTH, HEIGHT))

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

max_x = max([n.x for k,n in grapheVille.nodes.iteritems()])
max_y = max([n.y for k,n in grapheVille.nodes.iteritems()])

propor_x = (WIDTH - 2 * decalage_w) / max_x
propor_y = (HEIGHT - 2 * decalage_h) / max_y

for id, n in grapheVille.nodes.iteritems() :
    position = (int(decalage_w + n.x * propor_x) , int(decalage_h + n.y * propor_y))
    pygame.draw.circle(screen, coleur_station, position , 10, 1)

while 1 :
    pygame.display.update() 
