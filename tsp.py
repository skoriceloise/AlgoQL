def greedyTSP(distMat, idNodes):
	#Resolution du TSP par methode gloutonne (meilleure insertion)
	cycle = []
	added = []
	for idNode in idNodes:
		insertNodeTSP(distMat, idNode, cycle)

	return cycle

def insertNodeTSP(distMat, idNodeInsert, cycle):
	#Choix de la meilleure insertion dans le cycle existant
	if len(cycle) < 3:
		cycle.append(idNodeInsert)
	else:
		bestInsert = (float("inf"), None)
		for i, idNode in enumerate(cycle[:-1]):
			d = distMat[idNode][idNodeInsert] + distMat[idNodeInsert][cycle[i+1]]
			if d < bestInsert[0]:
				bestInsert = (d, i)
		d = distMat[cycle[-1]][idNodeInsert] + distMat[idNodeInsert][cycle[0]]
		if d < bestInsert[0]:
			bestInsert = (d, len(cycle) - 1)
		cycle.insert(bestInsert[1] + 1, idNodeInsert)

