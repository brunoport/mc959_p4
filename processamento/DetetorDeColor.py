from scipy.spatial import distance as dist
from collections import OrderedDict
import numpy as np
import cv2
 
class DetetorDeColor:
	def __init__(self):
		colors = OrderedDict({
			"redProduct": (0, 0, 190),
			"darkBlueProduct": (190, 0, 0),
			"clearBlueProduct": (190, 190, 0),
			"yellowProduct": (0,190,190),
			"greenProduct" : (0,190,0),
			"purpleProduct":(190,0,190),
			"background":(190,255,190),
			"black":(0,0,0)})
		self.bgr = np.zeros((len(colors), 1, 3))
		self.colorNames = []
		for (i, (name, rgb)) in enumerate(colors.items()):
			self.bgr[i] = rgb
			self.colorNames.append(name)

	def find_color(self,block):
		self.count = np.zeros(len(self.colorNames))
		for x,y,z in block:
			minDist = (np.inf, None)
			for (i, row) in enumerate(self.bgr):
				d = (((row[0][0]-x))**2+((row[0][1]-y))**2+((row[0][2]-z))**2)
				if d < minDist[0]:
					minDist = (d, i)
			self.count[minDist[1]] = self.count[minDist[1]]+1
		return self.colorNames,self.count
