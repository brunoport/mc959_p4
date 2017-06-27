from DetetorDeColor import DetetorDeColor
import imutils
import cv2
import numpy as np

class DetetorDeProduto:
	def __init__(self):
		pass
	# YELLOW = 0
	# GREEN =  1
	# CLEAR BLUE = 2
	# RED = 4
	# DARK BLUE = 6
	# PURPLE = 7

	def detectColor(self,color):
		numberCubes = self.detect()
		return numberCubes[color]
	def detect(self):
		# FIND IMAGE
		image = cv2.imread("Heyh.jpeg")

		grayFull = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		resized = imutils.resize(image,width=300)
		ratio = image.shape[0]/float(resized.shape[0])
		gray = cv2.cvtColor(resized,cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (5, 5), 0)
		tresh = cv2.threshold(blurred, 20, 255, cv2.THRESH_BINARY)[1]
		cnts = cv2.findContours(tresh.copy(),cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[0] if imutils.is_cv2() else cnts[1]
		grayFull[grayFull == 0] = 1
		rectangles = []
		wToMean = 0
		coutoursCounted = 0
		for c in cnts:
			#print "hello"
			M = cv2.moments(c)
			if(M["m10"] != 0 and M["m00"] != 0 and M["m01"]!=0):
				cX = int((M["m10"] / M["m00"]) * ratio)
				cY = int((M["m01"] / M["m00"]) * ratio)
				c = c.astype("float")
				c *= ratio
				c = c.astype("int")
				x,y,w,h = cv2.boundingRect(c)
				if w > 250:
					wToMean+= w
					coutoursCounted += 1
					cv2.drawContours(grayFull, [c], -1, 0, -1)
					pts = np.argwhere(grayFull == 0)
					grayFull[grayFull == 0] = 1
					rectangles.append((cX,cY,image[pts[:,0],pts[:,1]]))
		wToMean = float(wToMean/coutoursCounted)
		rectangles.sort(key=lambda tup: tup[1])
		dc = DetetorDeColor()
		limiares = [1250,1300,1300,1600,1800] # para w = 380
		limiaresToUse = []
		for lim in limiares:
			limiaresToUse.append(int(lim*wToMean/450))
		numberCubes = [0,0,0,0,0,0,0,0]
		for i,tupl in enumerate(rectangles):
			rect = tupl[2]
			names,counts = dc.find_color(rect)
			for j in range(len(names)):
				if counts[j]>= 20:
					numberCubes[j] += int(counts[j]/limiaresToUse[i])+1
		return numberCubes
