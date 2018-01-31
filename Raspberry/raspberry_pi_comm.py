import serial
import struct
import time
import binascii

#arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
arduino = serial.Serial('/dev/ttyACM0', 115200)

command = [0 for i in range(8)] # {drive mode, (float) drive_data1, (float) drive_data2, (float) drive_data3, enable ultrasound, secondary motor command, servo command 1, servo command 2)

state = [0 for i in range(13)] # {drive_mode, rpm1, rpm2, rpm3, yaw, pitch, roll, ultrasonic_range, limit_switch1, limit_switch2, limit_switch3, limit_switch4, limit_switch5}

def stopRobot():
	command[:4] = [3, 0, 0, 0]
	command[-3] = 0
	command[-2] = 145
	command[-1] = 50
	print("Stop robot")
	updateArduino()
	time.sleep(0.1)
	updateArduino()

def updateArduino():
#	global state
#	print("The current angle is ", command[-1])
	b = struct.pack('=hfff?hhh', *command)
	arduino.write('\r'.encode() + b)
	arduino.flush()
	time.sleep(0.01)
	a = arduino.readline(40)
#	if arduino.inWaiting() >= 40:
#		a = arduino.readline(40)
#	else:
#		a = [];

		
	if arduino.inWaiting() > 1000:
		arduino.reset_input_buffer()
	if len(a) == 40:
		state = struct.unpack('=hfffffffhhhhh', a)
#		print('The current state is', state)

	
#	else:
#		state = []


#if __name__ == "__main__":
#	j = 0
#	a = 0
#	while 1:
#		if j < 500:
#			a = 0
#		elif j < 1000:
#			a = 90
#		else:
#			j = 0
#		command = [3,0,0,0,0,0, 180, a]
#		state = updateArduino()
#		if state != {}:
#			print(state)
#		
#		time.sleep(0.01)
#		j = j+1
