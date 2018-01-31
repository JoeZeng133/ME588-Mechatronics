from raspberry_pi_comm import *
from utility import *
from logic import *
import cv2
import time


def motorTest():
	print("Go forward")
	command[:4] = [3, 25, 0, 0]
	updateArduino()
	time.sleep(2)
	
	print("Go forward")
	command[:4] = [3, -25, 0, 0]
	updateArduino()
	time.sleep(2)

#	print("Go side ways")
#	command[:4] = [3, 0, 25, 0]
#	updateArduino()
#	time.sleep(2)
#	
#	print("Go side ways")
#	command[:4] = [3, 0, -25, 0]
#	updateArduino()
#	time.sleep(2)

#	print("Rotate")
#	command[:4] = [3, 0, 0, 1]
#	updateArduino()
#	time.sleep(2)
#	
#	print("Rotate")
#	command[:4] = [3, 0, 0, -1]
#	updateArduino()
#	time.sleep(2)
	
	
	stopRobot()

def servoTest():
	command[-2] = 150
	updateArduino()
	command[:4] = [3, 0, 0, 0]
#	print("Test first servo")
#	for i in range(0, 180, 30):
#		command[-1] = i
#		updateArduino()
#		time.sleep(0.5)
	
	print("Test second servo")
#	for i in range(0, 180, 30):
	command[-2] = 50
	updateArduino()
	time.sleep(2)


		
def camTest():
	stopRobot()
#	command[:4] = [3, 0, 0, 0]
	cameraItr, _ = cameraInit()
	angle = 0
	prevtime = time.time() - 2
	for frame in cameraItr:
		cv2.imshow('res', frame)
		k = cv2.waitKey(1) & 0xFF
		now = time.time()
		if now - prevtime > 2:
			angle = (angle + 20) % 150
			print(angle)
			command[-1] = int(angle)
			updateArduino()
			prevtime = now
		if k == ord("q"):
			break
	
	cv2.destroyAllWindows()

def rollerTest():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(3, GPIO.IN, pull_up_down = GPIO.PUD_UP) #front 

	stopRobot()
	command[-3] = 255
	updateArduino()
	time.sleep(0.1)
	updateArduino()
	tstart = time.time()
	while(1):
		if time.time() - tstart > 5:
			break

		if not GPIO.input(3):
			print("Ball is in")


#	command[-3] = -255
#	updateArduino()
#	time.sleep(5)

	stopRobot()

def centerTest():
	stopRobot()
	command[0] = 3
	command[1] = 20
	command[-3] = 0
#	command[:4] = [3, 0, 0, 0]
#	command[-2] = 90
	command[-1] = 50
	updateArduino()
	
	cameraItr, _ = cameraInit()
	low = np.array([31, 56, 98])
	high = np.array([41, 131, 227])
	ballDec = detector(low, high, circleContour(), cameraItr)

	ballExist = False
	timeout = 1
	losttime = time.time()
	angle = 50
	command[-1] = 50
	updateArduino()
	
	obj = []
	p2 = (0, 0)
	while(1):
		objList, objDes, frame = ballDec.detect_continuous()
		obj = chooseTennisBall(objList, objDes, obj)
		
		cv2.drawContours(frame, objList, -1, (0,255,0), 1)
#		if obj:
#			print(obj)
#		cv2.rectangle(frame, obj[0], obj[1], (0, 255, 0), 2)
#		if obj:
#			ballExist = True
#			p1 = obj[0]
#			p2 = obj[1]
#			centerX = obj[2] / frame.shape[1]
#			centerY = obj[3] / frame.shape[0]
#			cv2.rectangle(frame, p1, p2, (0, 255, 0), 2)
#			
#			Kp_servo = 10
#			angle = floor(angle + Kp_servo * (centerY - 0.5))
#			command[-1] = angle
#			if angle > 120:
#				angle = 120
#			
#			if angle < 50:
#				angle = 50
#				
##			if angle > 30:
###				command[-3] = 200
###				updateArduino()
##				time.sleep(5)
##				return
#			
#			Kp_drive = 1
#			command[3] = -Kp_drive * (centerX - 0.5)
#			updateArduino()

#		else:
#			if ballExist:
#				losttime = time.time()
#				ballExist = False
#				
#			elif time.time() - losttime > timeout:
#				print("Object is lost")
#				break
				
		cv2.imshow('res', frame)
		k = cv2.waitKey(1) & 0xff
		if k == ord('q'):
			print("User interrupt")
			break
	
	stopRobot()
	cv2.destroyAllWindows()
	
def commTest():
	command[:4] = [1,0,0,0]
	while 1:
		updateArduino()
		print(state)
		time.sleep(0.1)

#rollerTest()
#servoTest()
#motorTest()
#stopRobot()
#camTest()
motorTest()
#calibration()
#centerTest()
#stopRobot()
#stopRobot()
#rollerTest()
#commTest()
