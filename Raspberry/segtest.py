from skimage import data, color
from skimage.future import graph
from matplotlib import pyplot as plt
from skimage.color import rgb2gray
from skimage.filters import sobel
from skimage.feature import canny
from skimage.segmentation import felzenszwalb, slic, quickshift, watershed
from scipy import ndimage as ndi

import cv2
from utility import *

cameraItr = webcamWrap()
print("Press space if you are OK")
for img in cameraItr:
	img = cv2.resize(img, None, fx = 0.5, fy = 0.5)
	labels1 = slic(img, compactness=30, n_segments=1000)
	out1 = color.label2rgb(labels1, img, kind='avg')
	out2 = color.rgb2lab(out1)


	cv2.imshow('res', out1)
	k = cv2.waitKey(1) & 0xFF
	if k == ord(" "):
		break

roi = cv2.selectROI('res', out1)
roi = getIntRoi(roi)
val = out2[roi[1] : roi[1] + roi[3], roi[0] : roi[0] + roi[2], :].mean(axis = (0, 1))

print(val)
cv2.namedWindow('res')
cv2.createTrackbar('thresh', 'res', 500, 1000, nothing)
# cv2.createTrackbar('high', 'res', 200, 500, nothing)

for img in cameraItr:
	thresh = cv2.getTrackbarPos('thresh', 'res') / 10

	img = cv2.resize(img, None, fx = 0.5, fy = 0.5)
	labels1 = slic(img, compactness=30, n_segments=1000)
	out1 = color.label2rgb(labels1, img, kind='avg')
	out2 = color.rgb2lab(out1)
	
	out3 = np.zeros(out2.shape[:2])
	for i in range(3):
		out3 += abs(out2[:, :, i] - val[i])

	out3 = np.array(out3 / out3.max() * 255, dtype = np.uint8)
	out4 = np.array((out3 < thresh) * 255, dtype = np.uint8)
	# edges = cv2.Canny(out3, 100, 200)
	cv2.imshow('res', out4)
	k = cv2.waitKey(1) & 0xFF
	if k == ord(" "):
		break

cv2.destroyAllWindows()