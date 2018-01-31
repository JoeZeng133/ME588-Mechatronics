import cv2
import numpy as np 
from matplotlib import pyplot as plt
from math import *
from utility import *

scale = 1

class camTracker:
	chnl = [0, 1]
	rg = [20, 70,  60, 170]
	rho = 7
	filt = None
	prevtmp = None

	def __init__(self, img, bbox):
		print(bbox)
		self.bbox = (int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]))
		self.img = cv2.medianBlur(img,3)
		self.getRoi()
		self.getHist()

	def getRoi(self):
		self.center = np.array([self.bbox[1] + self.bbox[3] / 2, self.bbox[0] + self.bbox[2] / 2])
		deltax = self.bbox[3] // self.rho
		deltay = self.bbox[2] // self.rho
		self.roi = self.img[max(self.bbox[1] - deltax, 0) : self.bbox[1] + self.bbox[3] + deltax, max(self.bbox[0] - deltay, 0): self.bbox[0] + self.bbox[2] + deltay]

	def getHist(self):
		self.hsv_roi = cv2.cvtColor(self.roi, cv2.COLOR_BGR2HSV)
		self.bins = [self.rg[1] - self.rg[0], self.rg[3] - self.rg[2]]
		self.roi_hist = cv2.calcHist([self.hsv_roi], self.chnl, None, self.bins, self.rg)
		self.roi_hist =	cv2.normalize(self.roi_hist, None, 0, 255, cv2.NORM_MINMAX)

	def update(self, img):
		# if not self.fail:
			# self.getRoi()
			# self.getHist()
		
		self.img = cv2.medianBlur(img,5)
		self.tmp = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)		
		self.tmp = cv2.calcBackProject([self.tmp], self.chnl, self.roi_hist, self.rg, 1)
		cv2.equalizeHist(self.tmp, dst = self.tmp)
		cv2.medianBlur(self.tmp, 5, dst = self.tmp)

		term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
		ret, self.bbox = cv2.CamShift(self.tmp, self.bbox, term_crit)

		self.matcher()

	def matcher(self):
		kernel = np.ones((4, 4), np.uint8)
		cv2.threshold(self.tmp, 0, 255, cv2.THRESH_OTSU, dst = self.tmp)
		if self.filt is None:
			self.filt = seqFilter(self.tmp, 3)
		
		self.tmp = np.array((self.filt.refresh(self.tmp) > 200) * 255, dtype = np.uint8)
		# cv2.morphologyEx(self.tmp, cv2.MORPH_OPEN, kernel, iterations = 2, dst = self.tmp)
		# cv2.morphologyEx(self.tmp, cv2.MORPH_CLOSE, kernel, iterations = 2, dst = self.tmp)

	def draw(self, img):
		cv2.rectangle(img, (self.bbox[0], self.bbox[1]), (self.bbox[0] + self.bbox[2], self.bbox[1] + self.bbox[3]), (0, 0, 255), 2)

	def fail(self):
		# needs further judgement
		if self.bbox[2] > 80 or self.bbox[3] > 80:
			return True
		if self.bbox[2] < 20 or self.bbox[3] < 20:
			return True
		tmp = self.bbox[2] / self.bbox[3]
		if tmp > 1.5 or tmp < 0.7:
			return True
		return False

def main():
	camItr  = cameraInit()
	roi= centerCalib(camItr)
	frame = camItr.next()
	tracker = camTracker(frame, roi)
	print("Detection Begins")
	for frame in camItr:
		tracker.update(frame)
		tracker.draw(frame)
		tmp = cv2.cvtColor(tracker.tmp, cv2.COLOR_GRAY2BGR)
		cv2.imshow('res', np.hstack([frame, tmp]))
		k = cv2.waitKey(1) & 0xff
		if k == 32:
			cv2.destroyAllWindows()
			break

main()
