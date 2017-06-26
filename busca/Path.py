# from Vertex import Vertex
# from Arc import Arc
# from Constant import Constant
from Graph import Graph
from Graph import Comando

# positions with marks
#      1    2    3    4
# A   1A   2A   3A   4A
# B   1B   2B   3B   4B
# C   1C   2C   3C   4C

mark = {
    '1A':'markADown1',
    '1B':'markBDown1',
    '1C':'markCDown1',
    '1D':'markAUp1',
    '1E':'markBUp1',
    '1F':'markCUp1',
    '2A':'markADown2',
    '2B':'markBDown2',
    '2C':'markCDown2',
    '2D':'markAUp2',
    '2E':'markBUp2',
    '2F':'markCUp2',
    '3A':'markADown3',
    '3B':'markBDown3',
    '3C':'markCDown3',
    '3D':'markAUp3',
    '3E':'markBUp3',
    '3F':'markCUp3',
    '4A':'markADown4',
    '4B':'markBDown4',
    '4C':'markCDown4',
    '4D':'markAUp4',
    '4E':'markBUp4',
    '4F':'markCUp4'
}

def getSection(pos):
    return mark.keys()[mark.values().index(pos)]


class PathPlanner:
    def __init__(self):
        self.graph = Graph()
        # graph.printArcs()
    def getPath(self,source,dest):
        mSource = mark[source]
        mDest = mark[dest]
        print "mSource = "+str(mSource)
        print "mDest = "+str(mDest)
        lastPos, solution = self.graph.actionsToObjective(mSource,mDest)
        lastPos = getSection(lastPos.getLabel())
        # print "solution = "+str(solution)
        # print "lastPos = "+str(lastPos)

        solution.append(Comando.FOTO)

        return lastPos,solution
