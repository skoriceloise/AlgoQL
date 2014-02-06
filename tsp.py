def greedyTSP(distMat, idNodes):
	#Resolution du TSP par methode gloutonne (meilleure insertion)
	cycle = []
	added = []
	distance = 0.0
	for idNode in idNodes:
		distance = insertNodeTSP(distMat, idNode, cycle, distance)

	return (cycle, distance)

def insertNodeTSP(distMat, idNodeInsert, cycle, distance):
	#Choix de la meilleure insertion dans le cycle existant
	lenCycle = len(cycle)
	d = 0.0
	if lenCycle == 0:
		cycle.append(idNodeInsert)
	elif lenCycle < 3:
		d = distMat[cycle[0]][idNodeInsert] + distMat[idNodeInsert][cycle[-1]]
		if lenCycle == 2:
			d -= distMat[cycle[0]][cycle[1]]
		cycle.append(idNodeInsert)
	else:
		bestInsert = (float("inf"), None)
		for i, idNode in enumerate(cycle[:-1]):
			d = distMat[idNode][idNodeInsert] + distMat[idNodeInsert][cycle[i+1]]
			d -= distMat[idNode][cycle[i+1]]
			if d < bestInsert[0]:
				bestInsert = (d, i)
		d = distMat[cycle[-1]][idNodeInsert] + distMat[idNodeInsert][cycle[0]]
		d -= distMat[cycle[-1]][cycle[0]]
		if d < bestInsert[0]:
			bestInsert = (d, len(cycle) - 1)
		cycle.insert(bestInsert[1] + 1, idNodeInsert)

	return distance + d