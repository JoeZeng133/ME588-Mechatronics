from utility import *
import numpy as np
import cv2

cameraItr = webcamWrap()
# low, high = adjustHSV(cameraItr)
low = np.array([23, 86, 98])
high = np.array([39, 255, 255])

print(low, high)
dec = detector(low, high, circleContour(), cameraItr)
dec.preview()

while(1):
	k = cv2.waitKey(1) & 0xFF
	if k == ord("q"):
		break

cv2.destroyAllWindows()
