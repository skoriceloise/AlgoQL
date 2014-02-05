import graph
from xml.dom import minidom

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
                    newGraph.addEdge(idNode1, idNode2, float(edge.attributes['longueur'].value), float(edge.attributes['vitesse'].value))

    return newGraph


