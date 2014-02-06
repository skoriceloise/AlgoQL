import graph
import planVille
from xml.dom import minidom
from datetime import datetime

def lecturePlanXML(ficPlan):
    newGraph = graph.Graph()

    xmldoc = minidom.parse(ficPlan)
    nodesXML = xmldoc.getElementsByTagName('Noeud') 
    #Ajout de tous les noeuds au graphe
    for node in nodesXML:
        newGraph.addNode(float(node.attributes['x'].value) * 8.0, float(node.attributes['y'].value) * 8.0)

    #Ajout des aretes au graphe
    for node in nodesXML:
        for edge in node.getElementsByTagName('TronconSortant'):
            idNode1 = int(node.attributes['id'].value)
            idNode2 = int(edge.attributes['destination'].value)
            if idNode2 not in newGraph.edges.keys() or idNode1 not in newGraph.edges[idNode2]:
                    longueur = edge.attributes['longueur'].value
                    longueur = longueur.replace(',', '.')
                    vitesse = edge.attributes['vitesse'].value
                    vitesse = vitesse.replace(',','.')
                    newGraph.addEdge(idNode1, idNode2, float(longueur), float(vitesse))

    return newGraph

def lectureCommandesXML(ficLivr, graph):
    commandes = []
    xmldoc = minidom.parse(ficLivr)
    commandesXML = xmldoc.getElementsByTagName('Livraison')
    for c in commandesXML:
        att = c.attributes

        heure = datetime.strptime(att['heure'].value, '%d-%m-%Y %H:%M:%S')
        commObject = planVille.Commande(int(att['adresse'].value), float(att['volume'].value), float(att['poids'].value), heure)
        commandes.append(commObject)
        print commObject.noeud

    entrepot = int(xmldoc.getElementsByTagName('Entrepot')[0].attributes['adresse'].value)

    return (commandes, entrepot)
