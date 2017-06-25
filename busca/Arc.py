class Arc:
    def __init__(self, source, target, action):
        self.source = source
        self.target = target
        self.action = action
        source.addLeavingArc(self)
        target.addComingArc(self)

    def getSource(self):
        return self.source

    def getTarget(self):
        return self.target

    def getAction(self):
        return self.action

    def setAction(self, action):
        self.action = action

    def toString(self):
        return "Arc from " + self.source.toString() + " to " + self.target.toString() + " w. action " + self.action
