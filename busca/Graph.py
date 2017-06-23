from Constant import Constant 
from Vertex import Vertex
from Arc import Arc
class Graph:
	def __init__(self):
		cst = Constant()
		self.vertices = []
		self.marks = {}
		self.transitionMark = {}
		self.transitionCorridor = {}
		self.arcs = []
		for ty in cst.typeNames:
			mType = cst.types[ty]
			for xPos in range(cst.xPositionRange[ty]):
				for yPos in range(cst.yPositionRange[ty]):
					for direction in range(cst.directionRange[ty]):
						vertex = Vertex(mType,xPos,yPos,direction)
						self.vertices.append(vertex)
						if(mType == cst.mark):
							self.marks[vertex.getLabel()] = vertex
						elif(mType == cst.transitionMark):
							self.transitionMark[vertex.getLabel()] = vertex
						else:
							self.transitionCorridor[vertex.getLabel()] = vertex
		self.createArcs()
	def printVert(self):
		for vertice in self.vertices:
			vertice.toString()
	def printArcs(self):
		print len(self.arcs)
		for arc in self.arcs:
			print arc.toString()
	def count(self):
		print len(arcs)
		print len(vertices)
	def createArcs(self):
		self.createArcBetweenTransitionMark()
		self.createArcBetweenTransitionCorridor()
		self.createArcBetweenMarkAndTransitionMark()
		self.createArcBetweenMarkAndTransitionCorridor()
	def createArcBetweenMarkAndTransitionMark(self):
		cst = Constant()
		for number in [1,2,3,4]:
			for direc in ["Up", "Down"]:
				for i in range(len(cst.letters)-1):
					labelTop = "mark"+cst.letters[i]+direc+str(number)
					labelMid = "transitionMark"+cst.letters[i]+"-"+cst.letters[i+1]+direc+str(number)
					labelBottom = "mark"+cst.letters[i+1]+direc+str(number)
					if(direc == "Up"):
						newArc1 = Arc(self.marks[labelBottom],self.transitionMark[labelMid],"nothing")
						newArc2 = Arc(self.transitionMark[labelMid],self.marks[labelTop],"nothing")
					else:
						newArc1 = Arc(self.marks[labelTop],self.transitionMark[labelMid],"nothing")
						newArc2 = Arc(self.transitionMark[labelMid],self.marks[labelBottom],"nothing")
					self.arcs.append(newArc1)
					self.arcs.append(newArc2)
	def createArcBetweenTransitionMark(self):
		cst = Constant()
		for number in [1,2,3,4]:
			for i in range(len(cst.letters)-1):
				labelUp = "transitionMark"+cst.letters[i]+"-"+cst.letters[i+1]+"Up"+str(number)
				labelDown = "transitionMark"+cst.letters[i]+"-"+cst.letters[i+1]+"Down"+str(number)
				newArc1 = Arc(self.transitionMark[labelUp],self.transitionMark[labelDown],"180")
				newArc2 = Arc(self.transitionMark[labelDown],self.transitionMark[labelUp],"180")
				self.arcs.append(newArc1)
				self.arcs.append(newArc2)

	def createArcBetweenTransitionCorridor(self):
		for direc in ["Up","Down"]:
			label1Right = "transitionCorridor1-2"+str(direc)+"Right"
			label1Left = "transitionCorridor1-2"+str(direc)+"Left"
			label2Right = "transitionCorridor2-3"+str(direc)+"Right"
			label2Left = "transitionCorridor2-3"+str(direc)+"Left"
			label3Right = "transitionCorridor3-4"+str(direc)+"Right"
			label3Left = "transitionCorridor3-4"+str(direc)+"Left"
			newArc1 = Arc(self.transitionCorridor[label1Right],self.transitionCorridor[label1Left],"180")
			newArc2 = Arc(self.transitionCorridor[label1Left],self.transitionCorridor[label1Right],"180")
			newArc3 = Arc(self.transitionCorridor[label2Right],self.transitionCorridor[label2Left],"180")
			newArc4 = Arc(self.transitionCorridor[label2Left],self.transitionCorridor[label2Right],"180")
			newArc5 = Arc(self.transitionCorridor[label3Right],self.transitionCorridor[label2Left],"180")
			newArc6 = Arc(self.transitionCorridor[label2Left],self.transitionCorridor[label3Right],"180")
			self.arcs.append(newArc1)
			self.arcs.append(newArc2)
			self.arcs.append(newArc3)
			self.arcs.append(newArc4)
			self.arcs.append(newArc5)
			self.arcs.append(newArc6)
			newArc1 = Arc(self.transitionCorridor[label1Right],self.transitionCorridor[label2Right],"nothing")
			newArc2 = Arc(self.transitionCorridor[label2Right],self.transitionCorridor[label3Right],"nothing")
			newArc3 = Arc(self.transitionCorridor[label3Left],self.transitionCorridor[label2Left],"nothing")
			newArc4 = Arc(self.transitionCorridor[label2Left],self.transitionCorridor[label1Left],"nothing")
			self.arcs.append(newArc1)
			self.arcs.append(newArc2)
			self.arcs.append(newArc3)
			self.arcs.append(newArc4)
	def createArcBetweenMarkAndTransitionCorridor(self):
		cst = Constant()
		for i in range(len(cst.numbers)-1):
			labelDownDLeft = "markCDown"+str(cst.numbers[i])
			labelDownULeft = "markCUp"+str(cst.numbers[i])
			labelDownDRight = "markCDown"+str(cst.numbers[i]+1)
			labelDownURight = "markCUp"+str(cst.numbers[i]+1)
			labeltransDownRight = "transitionCorridor"+str(cst.numbers[i])+"-"+str(cst.numbers[i]+1)+"DownRight"
			labeltransDownLeft = "transitionCorridor"+str(cst.numbers[i])+"-"+str(cst.numbers[i]+1)+"DownLeft"
			newArc1 = Arc(self.marks[labelDownDLeft],self.transitionCorridor[labeltransDownRight],"left")
			newArc2 = Arc(self.transitionCorridor[labeltransDownLeft],self.marks[labelDownULeft],"right")
			newArc3 = Arc(self.transitionCorridor[labeltransDownRight],self.marks[labelDownURight],"left")
			newArc4 = Arc(self.marks[labelDownDRight],self.transitionCorridor[labeltransDownLeft],"right")
			self.arcs.append(newArc1)
			self.arcs.append(newArc2)
			self.arcs.append(newArc3)
			self.arcs.append(newArc4)
			labelUpDLeft = "markADown"+str(cst.numbers[i])
			labelUpULeft = "markAUp"+str(cst.numbers[i])
			labelUpDRight = "markADown"+str(cst.numbers[i]+1)
			labelUpURight = "markAUp"+str(cst.numbers[i]+1)
			labeltransUpRight = "transitionCorridor"+str(cst.numbers[i])+"-"+str(cst.numbers[i]+1)+"UpRight"
			labeltransUpLeft = "transitionCorridor"+str(cst.numbers[i])+"-"+str(cst.numbers[i]+1)+"UpLeft"
			newArc1 = Arc(self.marks[labelUpURight],self.transitionCorridor[labeltransUpLeft],"left")
			newArc2 = Arc(self.transitionCorridor[labeltransUpLeft],self.marks[labelUpDLeft],"left")
			newArc3 = Arc(self.marks[labelUpULeft],self.transitionCorridor[labeltransUpRight],"right")
			newArc4 = Arc(self.transitionCorridor[labeltransUpRight],self.marks[labelUpDRight],"right")
			self.arcs.append(newArc1)
			self.arcs.append(newArc2)
			self.arcs.append(newArc3)
			self.arcs.append(newArc4)
			# ADD "NOTHING" ACTION FOR BORDERS