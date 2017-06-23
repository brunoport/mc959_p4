from Vertex import Vertex 
from Arc import Arc
from Constant import Constant
from Graph import Graph

cst = Constant()
vertices = []
marks = {}
transitionMark = {}
transitionCorridor = {}
arcs = []
graph = Graph()
#graph.printArcs()
source = raw_input("Posicao atual ?\n")
target = raw_input("Posicao desejada ?\n")
lastPos,solution = graph.actionsToObjective(source,target)
print solution
print "lastPos : "+lastPos.getLabel()
#markCUp1
#markBDown3
#markBUp3