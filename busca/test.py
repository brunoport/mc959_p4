import Path

pPlanner = Path.PathPlanner()

# graph.printArcs()
source = raw_input("Posicao atual ?\n")
target = raw_input("Posicao desejada ?\n")
lastPos,solution = pPlanner.getPath(source,target)
print "solution = "
for el in solution:
	print str(el.value)
print "lastPos = "+str(lastPos)
