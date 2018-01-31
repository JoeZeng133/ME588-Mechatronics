import cv2
import numpy as np
from utility import *
from skimage import data, segmentation, color

def getGaborBank(frame):
	sigma = 2
	wavelengthMin = 4/sqrt(2);
	wavelengthMax = frame.shape[1] * sqrt(2)
	n = floor(log2(wavelengthMax/wavelengthMin));
	wavelength = 2**(np.array(range(n - 2))) * wavelengthMin;
	orientation = np.linspace(0, pi, 8, endpoint = False)
	filterBank = []
	filterWaveLen = []
	for i in range(len(wavelength)):
		for j in range(len(orientation)):
			kernel = cv2.getGaborKernel((0, 0), sigma, orientation[j], wavelength[i], 1) 
			filterBank.append(kernel)
			filterWaveLen.append(wavelength[i])
			# print(kernel.min())
			# cv2.imshow('res', kernel)
			# k = cv2.waitKey(0) & 0xFF
			# if k == ord("q"):
			# 	break

	return filterBank, filterWaveLen

if __name__ == '__main__':
	cameraItr = webcamWrap()
	print("Choose the position")
	while(1):
		frame = cameraItr.next()
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		cv2.imshow("res", frame)
		k = cv2.waitKey(1) & 0xFF
		if k == ord(" "):
			break

	roi = cv2.selectROI("res", frame)
	roi = getIntRoi(roi)


	filterBank, filterWaveLen = getGaborBank(frame)
	num  = int(len(filterBank))
	imgBank = np.zeros(frame.shape + (num + 2,), dtype = np.float32)
	x = np.array(range(frame.shape[1]))
	y = np.array(range(frame.shape[0]))
	xv, yv = np.meshgrid(x, y)
	xv = (xv - xv.mean()) / xv.std()
	yv = (yv - yv.mean()) / yv.std()
	imgBank[: ,:, num] = xv
	imgBank[:, :, num + 1] = yv
	for (i, item) in enumerate(filterBank):

		tmp = cv2.filter2D(np.array(frame, dtype = np.float32), cv2.CV_32F, item)
		tmp = (tmp - tmp.mean()) / tmp.std()
		tmp = cv2.GaussianBlur(tmp, (0, 0), 0.5 * filterWaveLen[i])
		imgBank[:, :, i] = tmp
		# print(tmp.max())
		# print(tmp.min())
		# cv2.imshow('res',tmp)
		# k = cv2.waitKey(0) & 0xFF
		# if k == ord("q"):
		# 	break
	# print(roi)
	pick = imgBank[roi[1] : roi[1] + roi[3], roi[0] : roi[0] + roi[2], :].mean(axis = (0, 1))
	flatten = imgBank.reshape((frame.shape[0] * frame.shape[1],) + (num + 2,))
	# print(flatten.shape)
	
	# print(pick.shape)
	# tol = 0.1
	dist = np.sqrt(((flatten - pick)**2).sum(axis = (1,)))
	# dist = (dist < tol)
	# print(dist.shape)
	# dist = np.transpose(dist)
	dist = dist.reshape(frame.shape)
	# dist = np.array(dist, dtype = np.uint8)
	# print(dist.shape)
	cv2.imshow('res',dist)
	# flatten = flatten.reshape((len(filterBank), frame.shape[0], -1))
	cv2.waitKey(0)
	# abs(flatten - imgBank).max()
	cv2.destroyAllWindows()