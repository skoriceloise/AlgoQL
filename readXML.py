import graph
import planVille
from xml.dom import minidom
from datetime import datetime

def lecturePlanXML(ficPlan):
    newGraphVille = graph.Graph()
    newGraphReseau = graph.Graph()

    xmldoc = minidom.parse(ficPlan)
    nodesXML = xmldoc.getElementsByTagName('Noeud') 
    #Ajout de tous les noeuds au graphe
    for node in nodesXML:
            newGraphVille.addNode(float(node.attributes['x'].value) * 8.0, float(node.attributes['y'].value) * 8.0)
        
    #refaire proprement
    newGraphReseau.addNodeObject(newGraphVille.nodes[40])
    newGraphReseau.addNodeObject(newGraphVille.nodes[30])
    newGraphReseau.addNodeObject(newGraphVille.nodes[50])
    newGraphReseau.addNodeObject(newGraphVille.nodes[20])
    newGraphReseau.addNodeObject(newGraphVille.nodes[21])
    newGraphReseau.addNodeObject(newGraphVille.nodes[11])
    newGraphReseau.addNodeObject(newGraphVille.nodes[31])
    newGraphReseau.addNodeObject(newGraphVille.nodes[41])
    newGraphReseau.addNodeObject(newGraphVille.nodes[51])
    newGraphReseau.addNodeObject(newGraphVille.nodes[60])
    newGraphReseau.addNodeObject(newGraphVille.nodes[61])
    newGraphReseau.addNodeObject(newGraphVille.nodes[62])
    newGraphReseau.addNodeObject(newGraphVille.nodes[70])
    newGraphReseau.addNodeObject(newGraphVille.nodes[71])
    newGraphReseau.addNodeObject(newGraphVille.nodes[72])


    #Ajout des aretes au graphe
    for node in nodesXML:
        for edge in node.getElementsByTagName('TronconSortant'):
            idNode1 = int(node.attributes['id'].value)
            idNode2 = int(edge.attributes['destination'].value)
            if idNode2 not in newGraphVille.edges.keys() or idNode1 not in newGraphVille.edges[idNode2]:
                    longueur = edge.attributes['longueur'].value
                    longueur = longueur.replace(',', '.')
                    vitesse = edge.attributes['vitesse'].value
                    vitesse = vitesse.replace(',','.')                  
                    newGraphVille.addEdge(idNode1, idNode2, float(longueur), float(vitesse))

            if(edge.attributes['reseau'].value == '1'):
                if idNode2 not in newGraphReseau.edges.keys() or idNode1 not in newGraphReseau.edges[idNode2]:
                        longueur = edge.attributes['longueur'].value
                        longueur = longueur.replace(',', '.')
                        vitesse = edge.attributes['vitesse'].value
                        vitesse = vitesse.replace(',','.')       
                        newGraphReseau.addEdge(idNode1, idNode2, float(longueur), float(vitesse))

    return (newGraphVille, newGraphReseau)

def lectureCommandesXML(ficLivr, graph):
    commandes = []
    xmldoc = minidom.parse(ficLivr)
    commandesXML = xmldoc.getElementsByTagName('Livraison')
    for c in commandesXML:
        att = c.attributes
        heure = datetime.strptime(att['heure'].value, '%d-%m-%Y %H:%M:%S')
        commObject = planVille.Commande(int(att['adresse'].value), float(att['volume'].value), float(att['poids'].value), heure)
        commandes.append(commObject)

    entrepot = int(xmldoc.getElementsByTagName('Entrepot')[0].attributes['adresse'].value)

    return (commandes, entrepot)
