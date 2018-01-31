import cv2
import numpy as np
from utility import *
from matplotlib import pyplot as plt
from math import *
from skimage import data, color
from logic import *

def nothing(x):
	pass

def cannyCalib():
	cameraItr = webcamWrap()
	cv2.namedWindow('res')
	cv2.createTrackbar('low', 'res', 80, 500, nothing)
	cv2.createTrackbar('high', 'res', 163, 500, nothing)
	for frame in cameraItr:
		low = cv2.getTrackbarPos('low', 'res')
		high = cv2.getTrackbarPos('high', 'res')
		edges = cv2.Canny(frame, low, high)
		kernel = np.ones((5,5),np.uint8)
		# edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations = 1)
		cv2.imshow('res', edges)
		k = cv2.waitKey(1) & 0xFF
		if k == ord(" "):
			break

	print(low, high)


def floor(orig, thresh, low, high):
	kernel = np.ones((5,5),np.uint8)
	try:
		img = cv2.GaussianBlur(orig, (3, 3), 0)
	except:
		print(img.shape)
		
	edges = cv2.Canny(orig, low, high)
	# edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations = 2)
	mask = cv2.copyMakeBorder(edges, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value = 0) 
	low = (thresh, thresh, thresh)
	high = (thresh, thresh, thresh)
	lab = cv2.cvtColor(img,cv2.COLOR_BGR2Lab)
	# mask = np.zeros((img.shape[0] + 2, img.shape[1] + 2), dtype = np.uint8)
	newValue = (100, 0, 0)
	fillColor = int(1)
	count = []

	for i in range(img.shape[1]):
		seedPoint = (i, img.shape[0] - 1)
		if mask[seedPoint[1], seedPoint[0]] != 0:
			continue
		
		cv2.floodFill(lab, mask, seedPoint, newValue, low, high, 4 | (fillColor << 8) | cv2.FLOODFILL_MASK_ONLY)
		count.append(np.count_nonzero(mask == fillColor))	
		fillColor += 1

	# rgb = cv2.cvtColor(lab, cv2.COLOR_Lab2BGR)
	maxIdx = np.array(count).argmax()
	# print(maxIdx, fillColor)
	mask = (mask[1 : -1, 1 : -1] == (maxIdx + 1))
	mask = np.array(mask * 255, dtype = np.uint8)
	
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations = 2)

	# cv2.imshow('res', np.hstack([mask, edges]))
	_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	cnt = contours[0]
	hull = cv2.convexHull(cnt)


	return mask, hull, edges

def avoid():
	cameraItr = webcamWrap()
	frame = cameraItr.next()
	ratio = 10
	band = int(frame.shape[0] * (ratio - 1) / ratio)
	# frame = cameraItr.next()
	for frame in cameraItr:
		region = floor(frame, 2.2, 80, 161)
		_, contours, _ = cv2.findContours(region,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(frame, contours, -1, (0,255,0), 3)
		cv2.imshow('res', frame)
		k = cv2.waitKey(1) & 0xFF
		if k == ord(" "):
			break

	cv2.destroyAllWindows()


def main():
	cameraItr = webcamWrap()
	cv2.namedWindow('res')
	# cv2.createTrackbar('thresh', 'res', 300, 500, nothing)
	thresh = 2.2
	low = 80
	high = 161

	for frame in cameraItr:
		mask, hull, edges= floor(frame, thresh, low, high)
		black = np.zeros(frame.shape[:2], dtype = np.uint8)
		cv2.drawContours(black, [hull], -1, 255, 3)
		cv2.imshow('res', black)
		cv2.imshow('res2', edges)
		k = cv2.waitKey(1) & 0xFF
		if k == ord(" "):
			thresh = input("New thresh values")
			thresh = float(thresh)
			# thresh, low, high = input("new values").split()
			# thresh, low, high = [float(thresh), int(low), int(high)]
			# print(thresh, low, high)
			print(thresh)

		if k == ord("q"):
			break

	cv2.destroyAllWindows()

# cannyCalib()
# avoid()
main()
