class Constant:
	mark = 0
	white = 0
	grey = 1 
	black = 2
	transitionMark = 1
	transitionCorridor = 2
	a = 0
	b = 1
	c = 2
	up = 0
	down = 1
	right = 0
	left = 1
	ab = 0
	bc = 1
	x12 = 0
	x23 = 1
	x34 = 2
	letters = ['A','B','C']
	numbers = [1,2,3,4]
	positionCorrespondance = { 
		"aU1" : 0,
		"aU2" : 1,
		"aU3" : 2,
		"aU4" : 3,
		"bU1" : 4,
		"bU2" : 5,
		"bU3" : 6,
		"bU4" : 7,
		"cU1" : 8,
		"cU2" : 9,
		"cU3" : 10,
		"cU4" : 11,
		"aD1" : 12,
		"aD2" : 13,
		"aD3" : 14,
		"aD4" : 15,
		"bD1" : 16,
		"bD2" : 17,
		"bD3" : 18,
		"bD4" : 19,
		"cD1" : 20,
		"cD2" : 21,
		"cD3" : 22,
		"cD4" : 23
	}
	
	typeNames = ['mark', 'transitionCorridor']
	types =  {'mark': 0, 'transitionMark' : 1, 'transitionCorridor' : 2}
	xPositionRange =  {'mark': 4, 'transitionMark' : 4, 'transitionCorridor' : 3}
	yPositionRange = {'mark': 3, 'transitionMark' : 2, 'transitionCorridor' : 2}
	directionRange = {'mark': 2, 'transitionMark' : 2, 'transitionCorridor' : 2}
	def __init__(self):
		pass
