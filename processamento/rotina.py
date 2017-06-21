from DetetorDeForma import DetetorDeForma
from DetetorDeColor import DetetorDeColor
import argparse
import imutils
import cv2
import numpy as np
from matplotlib import pyplot as plt

ap = argparse.ArgumentParser()
ap.add_argument("-i","--image", required=True,
	help="Path to the input image")
args = vars(ap.parse_args())
image = cv2.imread(args["image"])
imageClone = np.array(image)
grayFull = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
#Resize image
resized = imutils.resize(image,width=300)
#and keep the ratio
ratio = image.shape[0]/float(resized.shape[0])
#convert in gray blur and then treshold (?) 
gray = cv2.cvtColor(resized,cv2.COLOR_BGR2GRAY)
cv2.imshow("gray", gray)
cv2.waitKey(0)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
cv2.imshow("blurred", blurred)
cv2.waitKey(0)
tresh = cv2.threshold(blurred, 20, 255, cv2.THRESH_BINARY)[1]
cv2.imshow("Tresh", tresh)
cv2.waitKey(0)
cnts = cv2.findContours(tresh.copy(),cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if imutils.is_cv2() else cnts[1]
df = DetetorDeForma()
grayFull[grayFull == 0] = 1
rectangles = []
cv2.imshow("Image", imageClone)
cv2.waitKey(0)
for c in cnts:
	M = cv2.moments(c)
	if(M["m10"] != 0 and M["m00"] != 0 and M["m01"]!=0):
		cX = int((M["m10"] / M["m00"]) * ratio)
		cY = int((M["m01"] / M["m00"]) * ratio)
		shape = df.detect(c)
		c = c.astype("float")
		c *= ratio
		c = c.astype("int")
		x,y,w,h = cv2.boundingRect(c)
		if w > 250:
			cv2.drawContours(grayFull, [c], -1, 0, -1)
			cv2.drawContours(imageClone, [c], -1, (0,255,0), -1)
			pts = np.argwhere(grayFull == 0)
			grayFull[grayFull == 0] = 1
			rectangles.append((cX,cY,image[pts[:,0],pts[:,1]]))
			#cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
				#0.5, (255, 255, 255), 2)
			cv2.imshow("Image", imageClone)
			cv2.waitKey(0)
cv2.imshow("Image", image)
cv2.waitKey(0)
rectangles.sort(key=lambda tup: tup[1])
dc = DetetorDeColor()
#limiares = [1400,1300,1450,1800,1800]
limiares = [1250,1300,1300,1600,1800]
for i,tupl in enumerate(rectangles):
	print "Platereira "+str(i)
	rect = tupl[2]
	names,counts = dc.find_color(rect)
	print "total : ",
	print rect.shape[0]
	for j in range(len(names)):
		if counts[j]>= 20:
			print names[j]+" : ",
			print int(counts[j]/limiares[i])+1
	print ""
cv2.waitKey(0)