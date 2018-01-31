import serial
import struct
import time
import binascii

arduino = serial.Serial('COM5', 115200)

command = [0 for i in range(8)] # {drive mode, (float) drive_data1, (float) drive_data2, (float) drive_data3, enable ultrasound, secondary motor command, servo command 1, servo command 2)

state = [] # {drive_mode, rpm1, rpm2, rpm3, yaw, pitch, roll, ultrasonic_range, limit_switch1, limit_switch2, limit_switch3, limit_switch4, limit_switch5}

def updateArduino():
    b = struct.pack('=hfff?hhh', *command)
    arduino.write("\r".encode() + b)
    arduino.flush()
    time.sleep(0.01)
    a = arduino.readline(40)
    if arduino.inWaiting() > 1000:
        arduino.reset_input_buffer()
    if len(a) == 40:
        return struct.unpack('=hfffffffhhhhh', a)
    else:
        return {}

#if __name__ == "__main__":
j = 0
while 1:
    command = [0,1.2,1.3,9.2,0,0, 180, 90]
    state = updateArduino()
    if state != {}:
        print(state)
    time.sleep(0.01)
    j = j+1