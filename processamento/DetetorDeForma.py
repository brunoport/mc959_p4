#pyimagesearch.com/2016/02/08/opencv-shape-detection
import cv2

class DetetorDeForma:
	def __init__(self):
		pass
	def detect(self,contorno):
		shape ="unknown"
		perimetro = cv2.arcLength(contorno,True)
		approx = cv2.approxPolyDP(contorno,0.005*perimetro,True)
		if len(approx) == 4:
			(x, y, w, h) = cv2.boundingRect(approx)
			ar = w / float(h)
			if(ar<0.95 or ar >= 1.05):
				shape = "rectangle"
			return shape
