import cv2
import numpy as np 
from matplotlib import pyplot as plt
from math import *
import time

try:
	from picamera.array import PiRGBArray
	from picamera import PiCamera
	from fractions import Fraction
except:
	pass

class floorFinder:
	def __init__(self):
		pass

	def calib(self, img):
		pass

	def find(self, seedPoint):
		pass

def cameraInit():
	print("Camera is initializing")
	picSize = (512, 384)
	wantSize = (512, 384)
	camera = PiCamera(resolution = picSize, framerate = 40)


	camera.vflip = True
	rawCapture = PiRGBArray(camera, size=wantSize)
	camera.awb_mode='fluorescent'
	time.sleep(3)
#	camera.shutter_speed = 24830
	camera.shutter_speed = camera.exposure_speed
	print("exposure speed is", camera.shutter_speed)
	camera.exposure_mode = 'off'
#	g = camera.awb_gains
#	 print("awb gain is", g)
	camera.awb_mode='off'
	g = (1, 2.5)
	camera.awb_gains = g
	
	print('The awb_gains are', g)
	cameraItr = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True, resize=wantSize);
	print("Camera initialization is done")

#	print(camera.digital_gain, camera.analog_gain)

	return piCamWrap(cameraItr, rawCapture), camera

def pause():
	return cv2.waitKey(0) & 0xFF

class detector:
	maxItr = 100
	seqMask = []
	mask = None

	def __init__(self, low, high, cnt, cameraItr):
		self.cameraItr = cameraItr
		self.low = low.copy()
		self.high = high.copy()
		self.cnt = cnt.copy()

	def filt(self, frame):
		self.frame = frame
		cv2.imshow('Original', frame)

		frame = cv2.GaussianBlur(frame, (5, 5), 0)

		cv2.imshow('GaussianBlur', frame)

		hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		self.mask = cv2.inRange(hsv, self.low, self.high)

		cv2.imshow('mask', self.mask)

		# cv2.imshow('res',self.mask)
		# cv2.waitKey(1)
#		if not self.seqMask:
#			self.seqMask = seqFilter(self.mask, 1)
#		else:
#			self.mask = self.seqMask.refresh(self.mask)
			# print(self.mask.max())

	def getCnt(self):
		self.mask = np.array((self.mask > 250) * 255, dtype = np.uint8)
		kernel = np.ones((5,5),np.uint8)
		self.mask = cv2.morphologyEx(self.mask, cv2.MORPH_CLOSE, kernel, iterations = 2)

		cv2.imshow('morph', self.mask)

		_, self.contours, _ = cv2.findContours(self.mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		self.des = self.desCalc(self.contours)	

	def detect_continuous(self):
		self.filt(self.cameraItr.next())
		self.getCnt()
		return self.contours, self.des, self.frame

	def clear(self):
		try:
			self.seqMask.clear()
		except:
			pass

	def detect(self):
		self.clear()
		for i in range(6):
			self.filt(self.cameraItr.next())

		self.getCnt()
		return self.contours, self.des

	def getVal(self):
		return self.contours, self.des

	def preview(self):

		while(1):
			self.filt(self.cameraItr.next())
			self.getCnt()
			frame = self.cameraItr.next()
			self.draw(frame)
			cv2.imshow('contours',frame)
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
		res = np.zeros((num, 5), dtype = np.float32)
		for i, item in enumerate(cnt):
			M = cv2.moments(item)
			if M['m00'] != 0:
				res[i] = np.array([M['m10']/M['m00'], M['m01']/M['m00'], sqrt(M['m00']), 
					cv2.matchShapes(self.cnt, self.contours[i], 1, 0.0), M['m00']], dtype = np.float32)
			else:
				res[i] = np.array([0, 0, 0, 0, 0], dtype = np.float32)
		return res	

class piCamWrap:
	def __init__(self, cameraItr, rawCapture):
		self.cameraItr = cameraItr
		self.rawCapture = rawCapture

	def __iter__(self):
		return self

	def __next__(self):
		return self.next()
	
	def next(self):
		for res in self.cameraItr:
			break
		self.rawCapture.truncate(0)
		return res.array

class webcamWrap:
	def __init__(self):
		self.cap = cv2.VideoCapture(0)
		self.cap.set(cv2.CAP_PROP_FPS, 30)
		self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 500)
		self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)
		time.sleep(0.5)
		print("Webcam is ready")
		# print(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
		# print(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
		# print(self.cap.get(cv2.CAP_PROP_FPS))
		# print(self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE))

	def __iter__(self):
		return self

	def __next__(self):
		return self.next()

	def next(self):
		ret, frame = self.cap.read()
		return frame

class stableFilter:
	idx = 0
	threshratio = 0.9
	def __init__(self, des, N, radius):
		self.N = N
		self.radius = radius
		self.numKey = des.shape[0]
		self.des = [np.zeros((1, des.shape[1]), dtype = np.float32) for i in range(N)]
		self.bf = cv2.BFMatcher(crossCheck = True)
		self.thresh = int(N * self.threshratio)
		self.stablePoints = []
		
	def refresh(self, des):
		if(des.shape[0] != 0):
			vote = np.zeros((des.shape[0], ), dtype = np.uint32)
			for i in self.des:
				matches = self.bf.knnMatch(des, i, k = 1)
				for idx,j in enumerate(matches):
					if j:
						if j[0].distance < self.radius:
							vote[idx] += 1
			
			
# print(des.shape[0] - (vote > self.thresh).sum())
			self.idx = (self.idx + 1) % self.N
			self.des[self.idx] = np.array(des, dtype = np.float32)
			return (vote > self.thresh)
			

class seqFilter:
	indx = 0
	def __init__(self, img, Nbit):
		self.Nbit = Nbit
		# self.N = 1 << Nbit
		self.N = 1 << Nbit
		self.img = [0 for i in range(self.N)]
		self.tot = np.array(img, dtype = np.uint32)
		self.img[0] = img.copy()

	def refresh(self, img):
		self.indx = (self.indx + 1) % self.N

		# print(img.shape)
		# print(self.tot.shape)

		self.tot = self.tot - self.img[self.indx] + img
		self.img[self.indx] = img.copy()
		# print(self.tot.max())
		return np.array(self.tot >> self.Nbit, dtype = np.uint8)

	def get(self):
		return np.array(self.tot >> self.Nbit, dtype = np.uint8)

	def clear(self):
		self.img = [0 for i in range(self.N)]
		self.tot = np.zeros(self.tot.shape, dtype = np.uint32)


def hillFunc(A, a, b):
	chnl = len(a)
	res = np.zeros(A.shape[:2])
	for i in range(chnl):
		res += np.exp(-(A[:, :, i] - (a[i] + b[i] ) / 2)**2 / (b[i] - a[i])**2 * 4)
	
	return np.array(res / 3 * 255, dtype = np.uint8)

def adjustHSV(cameraItr):
	print("press space when you are ready")
	for img in cameraItr:
#		print(img)
		cv2.imshow('image', img)
		k = cv2.waitKey(1)
		if k == ord(" "):
			break


	cv2.namedWindow('image')
	hh='Hue High'
	hl='Hue Low'
	sh='Saturation High'
	sl='Saturation Low'
	vh='Value High'
	vl='Value Low'

	cv2.createTrackbar(hl, 'image',0,255,nothing)
	cv2.createTrackbar(hh, 'image',255,255,nothing)
	cv2.createTrackbar(sl, 'image',0,255,nothing)
	cv2.createTrackbar(sh, 'image',255,255,nothing)
	cv2.createTrackbar(vl, 'image',0,255,nothing)
	cv2.createTrackbar(vh, 'image',255,255,nothing)
	
	# frame = cv2.GaussianBlur(img, (5, 5), 0)
	frame = img
	hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	while(1):
		hul=cv2.getTrackbarPos(hl, 'image')
		huh=cv2.getTrackbarPos(hh, 'image')
		sal=cv2.getTrackbarPos(sl, 'image')
		sah=cv2.getTrackbarPos(sh, 'image')
		val=cv2.getTrackbarPos(vl, 'image')
		vah=cv2.getTrackbarPos(vh, 'image')
		#make array for final values
		HSVLOW=np.array([hul,sal,val])
		HSVHIGH=np.array([huh,sah,vah])

		#apply the range on a mask
		mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
		res = cv2.bitwise_and(frame,frame, mask =mask)
		
		# res2 = hillFunc(hsv[:, :, :1], HSVLOW[:1], HSVHIGH[:1])
		# res2 = cv2.cvtColor(res2, cv2.COLOR_GRAY2BGR)
		cv2.imshow('image', res)
		k = cv2.waitKey(1) & 0xFF
		if k == 32:
			break

	cv2.destroyAllWindows()
	return HSVLOW, HSVHIGH
	
def centerCalib(camItr):
	print("Calibration Stage. Put your object inside the rectangle and press space")
	frame = camItr.next()
	x = frame.shape[0] // 7 * 3
	y = frame.shape[1] // 7 * 3
	h = frame.shape[0] // 7
	w = h
#	rawCapture.truncate(0)

	for frame in camItr:
		cv2.rectangle(frame, (y, x), (y + w, x + h), (0, 255, 0), 2)
		cv2.imshow('roi', frame)
		k = cv2.waitKey(1) & 0xff
		if k == 32:
			break
	cv2.destroyWindow('roi')
	return (y, x, w, h)

def center(cnt):
	M = cv2.moments(cnt)
	if M['m00'] > 0:
		return (int(M['m10'] / M['m00']), int(M['m01']/M['m00']))
	else:
		return (int(cnt[:, 0, 0].mean()), int(cnt[:, 0, 1].mean()))
	
def circleContour():
	img = cv2.imread('circleContour.png')
	frame = np.zeros(img.shape, dtype = np.uint8)
	img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	im2, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	return contours[0]

def nothing(x):
	pass

def getIntRoi(roi):
		return (int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3]))


