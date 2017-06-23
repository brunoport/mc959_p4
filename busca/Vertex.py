from Constant import Constant 
class Vertex:
	def __init__(self, mType, xPosition, yPosition, direction):
		cst = Constant()
		self.mType = mType
		self.color = cst.white
		self.xPosition = xPosition
		self.yPosition = yPosition
		self.direction = direction
		self.label = self.createLabel(mType,xPosition,yPosition,direction)
		self.leavingArcs = []
		self.comingArcs = []
		self.arcs = []
		self.location = self.getLocation(mType,xPosition,yPosition,direction)

	def addLeavingArc(self, leavingArc):
		self.leavingArcs.append(leavingArc)
		self.arcs.append(leavingArc)
	def addComingArc(self, comingArc):
		self.comingArcs.append(comingArc)
		self.arcs.append(comingArc)
	def toString(self):
		return self.label+"("+str(self.location)+")"
	def getLocation(self,mType,xPosition,yPosition,direction):
		cst = Constant()
		if(mType != cst.mark):
			return -1
		else:
			locationGetter = ""
			if(yPosition == cst.a):
				locationGetter += "a"
				if(direction == cst.up):
					locationGetter += "U"
					locationGetter += str(xPosition+1)
				if(direction == cst.down):
					locationGetter += "D"
					locationGetter += str(xPosition+1)
			if(yPosition == cst.b):
				locationGetter += "b"
				if(direction == cst.up):
					locationGetter += "U"
					locationGetter += str(xPosition+1)
				if(direction == cst.down):
					locationGetter += "D"
					locationGetter += str(xPosition+1)
			if(yPosition == cst.c):
				locationGetter += "c"
				if(direction == cst.up):
					locationGetter += "U"
					locationGetter += str(xPosition+1)
				if(direction == cst.down):
					locationGetter += "D"
					locationGetter += str(xPosition+1)
			return cst.positionCorrespondance[locationGetter]
	def getLabel(self):
		return self.label
	def createLabel(self, mType, xPosition, yPosition, direction):
		cst = Constant()
		if(mType == cst.mark):
			label = "mark"
			if(yPosition == cst.a):
				label += "A"
				if(direction == cst.up):
					label += "Up"
					label += str(xPosition+1)
				elif(direction == cst.down):
					label += "Down"
					label += str(xPosition+1)
			elif(yPosition == cst.b):
				label += "B"
				if(direction == cst.up):
					label += "Up"
					label += str(xPosition+1)
				elif(direction == cst.down):
					label += "Down"
					label += str(xPosition+1)
			elif(yPosition == cst.c):
				label += "C"
				if(direction == cst.up):
					label += "Up"
					label += str(xPosition+1)
				elif(direction == cst.down):
					label += "Down"
					label += str(xPosition+1)
		if(mType == cst.transitionMark):
			label = "transitionMark"
			if(yPosition == cst.ab):
				label += "A-B"
				if(direction == cst.up):
					label += "Up"
					label += str(xPosition+1)
				elif(direction == cst.down):
					label += "Down"
					label += str(xPosition+1)
			elif(yPosition == cst.bc):
				label += "B-C"
				if(direction == cst.up):
					label += "Up"
					label += str(xPosition+1)
				elif(direction == cst.down):
					label += "Down"
					label += str(xPosition+1)
		if(mType == cst.transitionCorridor):
			label = "transitionCorridor"
			if(xPosition == cst.x12):
				label += "1-2"
				if(yPosition == cst.up):
					label += "Up"
					if(direction == cst.right):
						label += "Right"
					elif(direction == cst.left):
						label += "Left"
				if(yPosition == cst.down):
					label += "Down"
					if(direction == cst.right):
						label += "Right"
					elif(direction == cst.left):
						label += "Left"
			if(xPosition == cst.x23):
				label += "2-3"
				if(yPosition == cst.up):
					label += "Up"
					if(direction == cst.right):
						label += "Right"
					elif(direction == cst.left):
						label += "Left"
				if(yPosition == cst.down):
					label += "Down"
					if(direction == cst.right):
						label += "Right"
					elif(direction == cst.left):
						label += "Left"
			if(xPosition == cst.x34):
				label += "3-4"
				if(yPosition == cst.up):
					label += "Up"
					if(direction == cst.right):
						label += "Right"
					elif(direction == cst.left):
						label += "Left"
				if(yPosition == cst.down):
					label += "Down"
					if(direction == cst.right):
						label += "Right"
					elif(direction == cst.left):
						label += "Left"
		return label
					
