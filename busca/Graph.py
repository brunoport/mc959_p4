from Constant import Constant
from Vertex import Vertex
from Arc import Arc
from enum import Enum


class Comando(Enum):
    ESQ = "left"
    RETO = "ahead"
    DIR = "right"
    ROT = "180"
    ROT_CAM = "rotateCamera"
    FOTO = "pic"
    NONE = "none"

class Graph:
    def __init__(self):
        cst = Constant()
        self.vertices = []
        self.verticesDict = {}
        self.marks = {}
        self.transitionMark = {}
        self.transitionCorridor = {}
        self.arcs = []
        for ty in cst.typeNames:
            mType = cst.types[ty]
            for xPos in range(cst.xPositionRange[ty]):
                for yPos in range(cst.yPositionRange[ty]):
                    for direction in range(cst.directionRange[ty]):
                        vertex = Vertex(mType, xPos, yPos, direction)
                        self.vertices.append(vertex)
                        self.verticesDict[vertex.getLabel()] = vertex
                        if (mType == cst.mark):
                            self.marks[vertex.getLabel()] = vertex
                        else:
                            self.transitionCorridor[vertex.getLabel()] = vertex
        self.createArcs()
        self.adjustActionForCorners()

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

    def constructSolution(self, source, target, fathers):
        solution = []
        current = target
        father = fathers[current.getLabel()]
        while father:
            arc = current.getArcFrom(father)
            solution.insert(0, Comando(arc.getAction()))
            current = father
            father = fathers[current.getLabel()]
        return solution

    def actionsToObjective(self, firstPositionLabel, objectiveLabel):
        source = self.verticesDict[firstPositionLabel]
        target = self.verticesDict[objectiveLabel]
        lastPos = target
        sett, fathers = self.bfs(source)
        solution = self.constructSolution(source, target, fathers)
        print solution[len(solution) - 1]
        if (solution[len(solution) - 1] == Comando.ROT):
            solution[len(solution) - 1] = Comando.ROT_CAM
            lastPos = fathers[target.getLabel()]
        return lastPos, solution

    # 	source = self.verticesDict[sourceLabel]
    # 	target = self.verticesDict[targetLabel]
    # 	sett,fathers = self.bfs(source)
    # 	print "FROM ",
    # 	print source.toString()
    # 	solution = self.constructSolution(source,target,fathers)
    # 	for arc in solution:
    # 		print arc.getAction()
    # 	print "TO ",
    # 	print target.toString()
    # def testBusca(self,sourceLabel, targetLabel):
    # 	source = self.verticesDict[sourceLabel]
    # 	target = self.verticesDict[targetLabel]
    # 	sett,fathers = self.bfs(source)
    # 	print "FROM ",
    # 	print source.toString()
    # 	solution = self.constructSolution(source,target,fathers)
    # 	for arc in solution:
    # 		print arc.getAction()
    # 	print "TO ",
    # 	print target.toString()
    # current = target
    # father = fathers[current.getLabel()]
    # while father:
    # 	print current.getLabel()
    # 	current = father
    # 	# 	father = fathers[current.getLabel()]
    # def test(self):
    # 	for arc in self.arcs:
    # 		print arc.toString()
    def bfs(self, start):
        visited, queue = set(), [(None, start)]
        fathers = {}
        while queue:
            previous, vertex = queue.pop(0)
            fathers[vertex.getLabel()] = previous
            if vertex not in visited:
                visited.add(vertex)
                vertex.mark()
                toExt = vertex.getUnvisitedNexts()
                queue.extend(toExt)
        return visited, fathers

    def adjustActionForCorners(self):
        markAUp1 = self.verticesDict["markAUp1"]  # markAUp1
        markADown1 = self.verticesDict["markADown1"]  # markADown1
        markCUp1 = self.verticesDict["markCUp1"]  # markCUp1
        markCDown1 = self.verticesDict["markCDown1"]  # markCDown1
        markAUp4 = self.verticesDict["markAUp4"]  # markAUp4
        markADown4 = self.verticesDict["markADown4"]  # markADown4
        markCUp4 = self.verticesDict["markCUp4"]  # markCUp4
        markCDown4 = self.verticesDict["markCDown4"]  # markCDown4
        transitionCorridor12UpRight = self.verticesDict["transitionCorridor1-2UpRight"]  # transitionCorridor1-2UpRight
        transitionCorridor12UpLeft = self.verticesDict["transitionCorridor1-2UpLeft"]  # transitionCorridor1-2UpLeft
        transitionCorridor12DownRight = self.verticesDict[
            "transitionCorridor1-2DownRight"]  # transitionCorridor1-2DownRight
        transitionCorridor12DownLeft = self.verticesDict[
            "transitionCorridor1-2DownLeft"]  # transitionCorridor1-2DownLeft
        transitionCorridor34UpRight = self.verticesDict["transitionCorridor3-4UpRight"]  # transitionCorridor3-4UpRight
        transitionCorridor34UpLeft = self.verticesDict["transitionCorridor3-4UpLeft"]  # transitionCorridor3-4UpLeft
        transitionCorridor34DownRight = self.verticesDict[
            "transitionCorridor3-4DownRight"]  # transitionCorridor3-4DownRight
        transitionCorridor34DownLeft = self.verticesDict[
            "transitionCorridor3-4DownLeft"]  # transitionCorridor3-4DownLeft
        arc = transitionCorridor12UpLeft.getArcTo(markADown1)
        arc.setAction("ahead")
        arc = transitionCorridor12UpRight.getArcFrom(markAUp1)
        arc.setAction("ahead")
        arc = transitionCorridor12DownLeft.getArcTo(markCUp1)
        arc.setAction("ahead")
        arc = transitionCorridor12DownRight.getArcFrom(markCDown1)
        arc.setAction("ahead")
        arc = transitionCorridor34UpRight.getArcTo(markADown4)
        arc.setAction("ahead")
        arc = transitionCorridor34UpLeft.getArcFrom(markAUp4)
        arc.setAction("ahead")
        arc = transitionCorridor34DownRight.getArcTo(markCUp4)
        arc.setAction("ahead")
        arc = transitionCorridor34DownLeft.getArcFrom(markCDown4)
        arc.setAction("ahead")

    def createArcs(self):
        # self.createArcBetweenTransitionMark()
        self.createArcBetweenTransitionCorridor()
        self.createArcBetweenMark()
        self.createArcBetweenMarkAndTransitionCorridor()

    def createArcBetweenMark(self):
        cst = Constant()
        for number in [1, 2, 3, 4]:
            for direc in ["Up", "Down"]:
                for i in range(len(cst.letters) - 1):
                    labelTop = "mark" + cst.letters[i] + direc + str(number)
                    labelBottom = "mark" + cst.letters[i + 1] + direc + str(number)
                    if (direc == "Up"):
                        newArc = Arc(self.marks[labelBottom], self.marks[labelTop], "ahead")
                    else:
                        newArc = Arc(self.marks[labelTop], self.marks[labelBottom], "ahead")
                    self.arcs.append(newArc)
            for i in range(len(cst.letters)):
                labelLeft = "mark" + cst.letters[i] + "Up" + str(number)
                labelRight = "mark" + cst.letters[i] + "Down" + str(number)
                newArc1 = Arc(self.marks[labelLeft], self.marks[labelRight], "180")
                newArc2 = Arc(self.marks[labelRight], self.marks[labelLeft], "180")
                self.arcs.append(newArc1)
                self.arcs.append(newArc2)

    def createArcBetweenTransitionCorridor(self):
        for direc in ["Up", "Down"]:
            label1Right = "transitionCorridor1-2" + str(direc) + "Right"
            label1Left = "transitionCorridor1-2" + str(direc) + "Left"
            label2Right = "transitionCorridor2-3" + str(direc) + "Right"
            label2Left = "transitionCorridor2-3" + str(direc) + "Left"
            label3Right = "transitionCorridor3-4" + str(direc) + "Right"
            label3Left = "transitionCorridor3-4" + str(direc) + "Left"
            newArc1 = Arc(self.transitionCorridor[label1Right], self.transitionCorridor[label2Right], "ahead")
            newArc2 = Arc(self.transitionCorridor[label2Right], self.transitionCorridor[label3Right], "ahead")
            newArc3 = Arc(self.transitionCorridor[label3Left], self.transitionCorridor[label2Left], "ahead")
            newArc4 = Arc(self.transitionCorridor[label2Left], self.transitionCorridor[label1Left], "ahead")
            self.arcs.append(newArc1)
            self.arcs.append(newArc2)
            self.arcs.append(newArc3)
            self.arcs.append(newArc4)

    def createArcBetweenMarkAndTransitionCorridor(self):
        cst = Constant()
        for i in range(len(cst.numbers) - 1):
            labelDownDLeft = "markCDown" + str(cst.numbers[i])
            labelDownULeft = "markCUp" + str(cst.numbers[i])
            labelDownDRight = "markCDown" + str(cst.numbers[i] + 1)
            labelDownURight = "markCUp" + str(cst.numbers[i] + 1)
            labeltransDownRight = "transitionCorridor" + str(cst.numbers[i]) + "-" + str(
                cst.numbers[i] + 1) + "DownRight"
            labeltransDownLeft = "transitionCorridor" + str(cst.numbers[i]) + "-" + str(cst.numbers[i] + 1) + "DownLeft"
            newArc1 = Arc(self.marks[labelDownDLeft], self.transitionCorridor[labeltransDownRight], "left")
            newArc2 = Arc(self.transitionCorridor[labeltransDownLeft], self.marks[labelDownULeft], "right")
            newArc3 = Arc(self.transitionCorridor[labeltransDownRight], self.marks[labelDownURight], "left")
            newArc4 = Arc(self.marks[labelDownDRight], self.transitionCorridor[labeltransDownLeft], "right")
            self.arcs.append(newArc1)
            self.arcs.append(newArc2)
            self.arcs.append(newArc3)
            self.arcs.append(newArc4)
            labelUpDLeft = "markADown" + str(cst.numbers[i])
            labelUpULeft = "markAUp" + str(cst.numbers[i])
            labelUpDRight = "markADown" + str(cst.numbers[i] + 1)
            labelUpURight = "markAUp" + str(cst.numbers[i] + 1)
            labeltransUpRight = "transitionCorridor" + str(cst.numbers[i]) + "-" + str(cst.numbers[i] + 1) + "UpRight"
            labeltransUpLeft = "transitionCorridor" + str(cst.numbers[i]) + "-" + str(cst.numbers[i] + 1) + "UpLeft"
            newArc1 = Arc(self.marks[labelUpURight], self.transitionCorridor[labeltransUpLeft], "left")
            newArc2 = Arc(self.transitionCorridor[labeltransUpLeft], self.marks[labelUpDLeft], "left")
            newArc3 = Arc(self.marks[labelUpULeft], self.transitionCorridor[labeltransUpRight], "right")
            newArc4 = Arc(self.transitionCorridor[labeltransUpRight], self.marks[labelUpDRight], "right")
            self.arcs.append(newArc1)
            self.arcs.append(newArc2)
            self.arcs.append(newArc3)
            self.arcs.append(newArc4)
            # ADD "NOTHING" ACTION FOR BORDERS
