import cv2
import numpy as np
from utility import *
import time


def pause():
	return cv2.waitKey(0) & 0xFF

class detector:
	maxItr = 100
	seqMask = []
	mask = None

	def __init__(self, low, high, cnt):
		self.low = low.copy()
		self.high = high.copy()
		self.cnt = cnt.copy()

	def filt(self, cameraItr):
		frame = cameraItr.next()
		frame = cv2.GaussianBlur(frame, (5, 5), 0)
		self.frame = frame
		hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		self.mask = cv2.inRange(hsv, self.low, self.high)
		# cv2.imshow('res',self.mask)
		# cv2.waitKey(1)
		if not self.seqMask:
			self.seqMask = seqFilter(self.mask, 3)
		else:
			self.mask = self.seqMask.refresh(self.mask)
			# print(self.mask.max())

	def getCnt(self):
		self.mask = np.array((self.mask > 250) * 255, dtype = np.uint8)
		kernel = np.ones((5,5),np.uint8)
		self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel, iterations = 2)
		_, self.contours, _ = cv2.findContours(self.mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		self.des = self.desCalc(self.contours)	

	def detect(self, cameraItr):
		try:
			self.seqMask.clear()
		except:
			pass

		for i in range(16):
			self.filt(cameraItr)
			# cv2.imshow('test',self.frame)
			# cv2.waitKey(1)

		self.getCnt()
		return self.contours, self.des, self.frame

	def preview(self, cameraItr):
		if self.seqMask:
			self.seqMask.clear()

		while(1):
			self.filt(cameraItr)
			self.getCnt()
			frame = cameraItr.next()
			self.draw(frame)
			cv2.imshow('res',frame)
			k = cv2.waitKey(1) & 0xFF
			if k == ord("q"):
				break

	def draw(self, img):
		num = len(self.contours)
		print(num)

		cv2.drawContours(img, self.contours, -1, (0,255,0), 1)
		font = cv2.FONT_HERSHEY_SIMPLEX
		for i in range(num):
			if self.des[i][2] != 0:
				cv2.putText(img,"%.3f" % self.des[i][3], (self.des[i][0], self.des[i][1]), font, 1, (255,255,255), 2, cv2.LINE_AA)
			
	def desCalc(self, cnt):
		num = len(self.contours)
		res = np.zeros((num, 4), dtype = np.float32)
		for i, item in enumerate(cnt):
			M = cv2.moments(item)
			if M['m00'] != 0:
				res[i] = np.array([M['m10']/M['m00'], M['m01']/M['m00'], sqrt(M['m00']), 
					cv2.matchShapes(self.cnt, self.contours[i], 1, 0.0)], dtype = np.float32)
			else:
				res[i] = np.array([0, 0, 0, 0], dtype = np.float32)
		return res	

	
# if __name__ ==  "__main__" :
# 	# cameraItr, _ = cameraInit()
# 	cameraItr = webcamWrap()
# 	# for image in cameraItr:
# 	# 	cv2.imshow('ff', image)
# 	# 	k = cv2.waitKey(1) & 0xFF
# 	# 	if k == ord("q"):
# 	# 		cv2.destroyAllWindows()
# 	# 		break

# 	# low, high = adjustHSV(image)
# 	# print(low, high)

# 	# for raspberry pi
# 	# low = np.array([ 32, 49, 86])
# 	# high = np.array([ 41, 203, 249]) 

# 	low = np.array([19, 119, 107])
# 	high = np.array([35, 255, 255])
	
# 	dante = detector(low, high, circleContour())
# 	# dante.preview(cameraItr)
# 	while(1):
# 		dante.detect(cameraItr)
# 		frame = cameraItr.next()
# 		dante.draw(frame)
# 		cv2.imshow('detected', frame)
# 		k = cv2.waitKey(1) & 0xFF
# 		if k == ord("q"):
# 			break
# 		time.sleep(0.5)

# 	cv2.destroyAllWindows()

