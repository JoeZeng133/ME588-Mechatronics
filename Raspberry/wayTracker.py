import cv2
import numpy as np 
from matplotlib import pyplot as plt
from math import *
from utility import *
from detector import detector

if __name__ ==  "__main__" :
	# camItr = cameraInit()
	cameraItr = webcamWrap()
	low = np.array([19, 119, 107])
	high = np.array([35, 255, 255])
	dante = detector(low, high, circleContour())

	# roi = centerCalib(camItr)
	# roi = cv2.selectROI('res', frame)
	while(1):
		print("I am looking !")
		objList, objDes, lastFrame = dante.detect(cameraItr)
		if objList:
			idx = objDes.argmax(0)[2]
			if objDes[idx][2] > 10:
				print("I found something!")
				print("Length is", objDes[idx][2])

				x,y,w,h = cv2.boundingRect(objList[idx])
				bbox = (x, y, w, h)
				frame = cameraItr.next()
				tracker = cv2.Tracker_create('MEDIANFLOW')
				tracker.init(lastFrame, bbox)

				print("Detection Begins")
				for frame in cameraItr:
					trackerFlag, bbox = tracker.update(frame)
					if trackerFlag:
						p1 = (int(bbox[0]), int(bbox[1]))
						p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
						cv2.rectangle(frame, p1, p2, (0, 255, 0), 2)
					else:
						print("I lost it")
						break

					cv2.imshow('res', frame)	
					k = cv2.waitKey(1) & 0xff
					if k == ord('q'):
						break

	cv2.destroyAllWindows()

main()
