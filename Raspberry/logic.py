from raspberry_pi_comm import *
from multiprocessing import Process
from utility import *
import RPi.GPIO as GPIO
import time



def ballBehave():
#	global state, command
	global ballHave
	print("Roller On")
	# turn on roller, wait until the ball is collected and stop the robot
	command[:4] = [3,15,0,0]
	updateArduino()
	t = time.time()
	sw = GPIO.input(2)
	
	while(1):
		if time.time() - t > 3:
			print("I didn't get the ball")
			command[:4] = [3, 0, 0, 0]
			command[-3] = 0
			updateArduino()
			time.sleep(0.1)
			updateArduino()
#			time.sleep(2)
			break
		
		if not GPIO.input(2):
			ballHave += 1
			print("I got the ball")
			break

	command[-3] = 0
	updateArduino()
	time.sleep(0.1)
	updateArduino()

def conBehave():
	print("I am getting close to container")
	#wait until the robot reaches container and stop the robot
	command[:4] = [3,15,0,0]
	t = time.time()

	while(1):
		if time.time() - t > 3:
			print("I didn't reach the  container")
			time.sleep(0.1)
			stopRobot()
			time.sleep(0.5)
#			return True
			break
	
		if not GPIO.input(3):
			print("I reach the container")
			time.sleep(0.1)
			stopRobot()
			command[-2] = 50
#			updateArduino()
			time.sleep(0.1)
			updateArduino()
#			return False
			time.sleep(5)
			break


def follow(ballDec, closeBehave, chooseBehave):
	print("Following the object")
	global cameraAngle, t_roll, logicState
	ballExist = False
	losttime = time.time()

	if logicState == 0:
#		for finding ball
		command[-3] = 255
		safetyAngle = 100
		timeout = 1

	if logicState == 1:
#		for finding container
		command[-3] = 0
		safetyAngle = 70
		timeout = 0.2

	obj = []
	p2 = (0, 0)
	while(1):
		objList, objDes, frame = ballDec.detect_continuous()
		obj = chooseBehave(objList, objDes, obj)
		if obj:
			ballExist = True
			p1 = obj[0]
			p2 = obj[1]
			centerX = obj[2] / frame.shape[1]
			centerY = obj[3] / frame.shape[0]
			cv2.rectangle(frame, p1, p2, (0, 255, 0), 2)
			
			Kp_servo = 15
			cameraAngle = floor(cameraAngle + Kp_servo * (centerY - 0.5))
			if cameraAngle < limit[0]:
				cameraAngle = limit[0]
			
			if cameraAngle > limit[1]:
				cameraAngle = limit[1]
			
			command[-1] = cameraAngle
			centerXOffset = 0.42
			forward_speed= 20
			turn_speed =0.5
			command[0] = 3
			command[1] = 2*forward_speed* (1-abs(centerXOffset-centerX))**2
			if cameraAngle > 70:
				command[1] = command[1] / 2

			if cameraAngle < 60:
				command[1] = command[1] * 2
			
			command[2] = 0
			command[3] = -2*turn_speed * (centerXOffset-centerX)
			
#			print(command[1], command[2], command[3])
#			print(centerX)
			
#			print('Comand is ', command)
			updateArduino()
			
#			print(centerX)

			#if the object enters safety zone
			
			if (cameraAngle > safetyAngle):
				closeBehave()
				return
		
		
		else:
			now = time.time()
			if ballExist:
				losttime = time.time()
				ballExist = False
				
			elif now - losttime > timeout:
				print('The lost angle is', cameraAngle)
				print("The lost time is",  now - losttime) 
				print("Object is lost")
				return False
				
		cv2.imshow('res', frame)
		k = cv2.waitKey(1) & 0xff
		if k == ord('q'):
			print("User interrupt")
			stopRobot()
			time.sleep(2)
			exit()
			break


def chooseTennisBall(objList, objDes, lastObj):
	if not objList:
		return []

	res = []
	mask = ((objDes[:, 2] > 10) & (objDes[:, -2] < 0.1))

	if lastObj:
		pos = np.array(lastObj[2:], dtype = np.float32)
		dist = objDes[:, :2] - pos
		dist = dist[:, 0]**2 + dist[:, 1]**2
#		print(dist)
		mask = (mask & (dist < 1000))

	if mask.max():
#		print('Yes')
		idx = objDes[mask, :].argmax(0)[2]
		idx = np.nonzero(mask)[0][idx]
		x,y,w,h = cv2.boundingRect(objList[idx])
		p1 = (int(x), int(y))
		p2 = (int(x + w), int(y + h))
		centerX = (p1[0] + p2[0]) / 2
		centerY = (p1[1] + p2[1]) / 2
		res = [p1, p2, centerX, centerY]
		#print("Shape ratio is", objDes[idx, -1])

	return res

def chooseCon(objList, objDes, lastObj):
	if not objList:
		return []
		
	idx = objDes.argmax(0)[-1]
#	print('The area is', objDes[idx][2])
	if objDes[idx][2] < 40:
		return []
	
#	print('YES')
	x,y,w,h = cv2.boundingRect(objList[idx])
	p1 = (int(x), int(y))
	p2 = (int(x + w), int(y + h))
	centerX = (p1[0] + p2[0]) / 2
	centerY = (p1[1] + p2[1]) / 2
	res = [p1, p2, centerX, centerY]
	return res

def findAround(detector, chooseBehave):
	#should stop robot completely first
	global cameraAngle, t_roll
	flip = 0
	print("Close view searching")
	obj = None
	tstart = time.time()
	tstart2 = time.time()
	command[:4] = [3,0,0,0]
	command[-1] = closeAngle

#	if logicState == 1:
#		command[-1] = farAngle
#		flip = 1

	command[-3] = 0;
	updateArduino()
	time.sleep(1)
	flags = 0

	while(1):
		if time.time() - tstart > 1 and not flags:
			command[3] = 1
			flags = 1
		if time.time() - tstart2> 15:
			break
			
		updateArduino()
		now = time.time()
		objList, objDes, frame= detector.detect_continuous()
		cv2.imshow('res', frame)
		k = cv2.waitKey(1) & 0xFF
		if k == ord("q"):
			stopRobot()
			exit()
			return
		
		obj = chooseBehave(objList, objDes, obj)
		if obj:
			if flip == 0:
				cameraAngle = closeAngle
			else:
				cameraAngle = farAngle
			print("I found something")
			return True

		
		
		if now - tstart  > 6.2:
			flip = ~flip
			tstart = now
			if flip == 0:
				cameraAngle = closeAngle
				command[3] = 1
				updateArduino()
				time.sleep(0.1)
				updateArduino()
				print("Close view searching")
#				time.sleep(1)
			else:
				cameraAngle = farAngle
				command[3] = -1
				updateArduino()
				time.sleep(0.1)
				updateArduino()
				print("Far view searching")
#				time.sleep(1)

			
			command[-1] = cameraAngle

	print("Didn't find anything")
	return []


def calibration(cameraItr):
	
#	cameraItr,_ = cameraInit()
	print("calibrate ball color")
	low, high = adjustHSV(cameraItr)
	f = open('data.txt', 'w')
	
	f.write(str(low[0]) + ' ' + str(low[1]) + ' ' + str(low[2]) + ' ')
	f.write(str(high[0]) + ' ' + str(high[1]) + ' ' + str(high[2]) + ' ')
	
	print("calibrate container color")
	low, high = adjustHSV(cameraItr)
	f.write(str(low[0]) + ' ' + str(low[1]) + ' ' + str(low[2]) + ' ')
	f.write(str(high[0]) + ' ' + str(high[1]) + ' ' + str(high[2]))
	f.close()
#	for i in range(3):
#		print(high[i], ',', )
#	return low, high
def getVal():
	f = open('data.txt', 'r')
	s = f.read().split(' ')
	lowball = np.array([int(s[0]), int(s[1]), int(s[2])])
	highball = np.array([int(s[3]), int(s[4]), int(s[5])])
	lowcon = np.array([int(s[6]), int(s[7]), int(s[8])])
	highcon = np.array([int(s[9]), int(s[10]), int(s[11])])
	f.close()
	return lowball, highball, lowcon, highcon

def main():
	cameraItr, _ = cameraInit()
	calibration(cameraItr)
	GPIO.setup(2, GPIO.IN, pull_up_down = GPIO.PUD_UP) #ball 
	GPIO.setup(3, GPIO.IN, pull_up_down = GPIO.PUD_UP) #front 
	stopRobot()

	print('Press front bar when you are ready')
	while GPIO.input(3):
		pass

	command[:4] = [3, 20, 0, 0]
	updateArduino()
	time.sleep(0.1)
	updateArduino()
	time.sleep(4)
	reallyStop()
	global logicState, starttime, ballHave

	lowBall, highBall, lowCon, highCon = getVal()

#	lowBall = np.array([27 , 58 , 137])
#	highBall = np.array([43 , 166 , 250])
	ballDec = detector(lowBall, highBall, circleContour(), cameraItr)

#	lowCon = np.array([99 , 136 , 61])
#	highCon = np.array([115 , 255 , 255])
	conDec = detector(lowCon, highCon, circleContour(), cameraItr)

	logicState = 0
	
	print("Find balls")
	t = time.time()
	starttime = time.time()
	while(ballHave < 3 and time.time() - starttime < 45):
		if findAround(ballDec, chooseTennisBall):
			follow(ballDec, ballBehave, chooseTennisBall)
		
	logicState = 1
	print("Find container _____________________________________")
	if findAround(conDec, chooseCon):
		follow(conDec, conBehave, chooseCon)

	reallyStop()

	cv2.destroyAllWindows()

def reallyStop():
	stopRobot()
	time.sleep(0.1)
	stopRobot()

def terminate(x):
	reallyStop()
	command[-2] = 50
	updateArduino()
	exit()

if __name__ ==  "__main__" :
	
	reallyStop()
	closeAngle = 80
	farAngle = 50

	ballHave = 0
	starttime = time.time()

	
#	cameraItr, _ = cameraInit()
	

	limit = [farAngle - 5, 120]
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(4, GPIO.IN, pull_up_down = GPIO.PUD_UP) #front 
	GPIO.add_event_detect(4, GPIO.RISING)
	GPIO.add_event_callback(4, terminate)
	main()
			
			
#	calibration()
#	print(getVal())
