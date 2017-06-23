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
graph.printArcs()
# for ty in cst.typeNames:
# 	mType = cst.types[ty]
# 	for xPos in range(cst.xPositionRange[ty]):
# 		for yPos in range(cst.yPositionRange[ty]):
# 			for direction in range(cst.directionRange[ty]):
# 				vertex = Vertex(mType,xPos,yPos,direction)
# 				vertices.append(vertex)
# 				print vertex.getLabel()
# 				if(mType == cst.mark):
# 					marks[vertex.getLabel()] = vertex
# 				elif(mType == cst.transitionMark):
# 					transitionMark[vertex.getLabel()] = vertex
# 				else:
# 					transitionCorridor[vertex.getLabel()] = vertex
# ## Create arc for mark / transition mark
# for i in range(len(cst.numbers)-1):
# 		labelDownDLeft = "markCDown"+str(cst.numbers[i])
# 		labelDownULeft = "markCUp"+str(cst.numbers[i])
# 		labelDownDRight = "markCDown"+str(cst.numbers[i]+1)
# 		labelDownURight = "markCUp"+str(cst.numbers[i]+1)
# 		labeltransDownRight = "transitionCorridor"+str(cst.numbers[i])+"-"+str(cst.numbers[i]+1)+"DownRight"
# 		labeltransDownLeft = "transitionCorridor"+str(cst.numbers[i])+"-"+str(cst.numbers[i]+1)+"DownLeft"
# 		newArc1 = Arc(marks[labelDownDLeft],transitionCorridor[labeltransDownRight],"left")
# 		newArc2 = Arc(transitionCorridor[labeltransDownLeft],marks[labelDownULeft],"right")
# 		newArc3 = Arc(transitionCorridor[labeltransDownRight],marks[labelDownURight],"left")
# 		newArc4 = Arc(marks[labelDownDRight],transitionCorridor[labeltransDownLeft],"right")
# 		arcs.append(newArc1)
# 		arcs.append(newArc2)
# 		arcs.append(newArc3)
# 		arcs.append(newArc4)
# 		labelUpDLeft = "markADown"+str(cst.numbers[i])
# 		labelUpULeft = "markAUp"+str(cst.numbers[i])
# 		labelUpDRight = "markADown"+str(cst.numbers[i]+1)
# 		labelUpURight = "markAUp"+str(cst.numbers[i]+1)
# 		labeltransUpRight = "transitionCorridor"+str(cst.numbers[i])+"-"+str(cst.numbers[i]+1)+"UpRight"
# 		labeltransUpLeft = "transitionCorridor"+str(cst.numbers[i])+"-"+str(cst.numbers[i]+1)+"UpLeft"
# 		newArc1 = Arc(marks[labelUpURight],transitionCorridor[labeltransUpLeft],"left")
# 		newArc2 = Arc(transitionCorridor[labeltransUpLeft],marks[labelUpDLeft],"left")
# 		newArc3 = Arc(marks[labelUpULeft],transitionCorridor[labeltransUpRight],"right")
# 		newArc4 = Arc(transitionCorridor[labeltransUpRight],marks[labelUpDRight],"right")
# 		arcs.append(newArc1)
# 		arcs.append(newArc2)
# 		arcs.append(newArc3)
# 		arcs.append(newArc4)
# 		# ADD NOT TURN ACTION FOR BORDERS
# for arc in arcs:
# 	print arc.toString()
# print len(arcs)

		