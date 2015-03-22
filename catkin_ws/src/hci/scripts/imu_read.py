#!/usr/bin/python
##This program will eventually need to be ported to cpp for speed!

#program will read in values from imu, calibrate, and then
#send to ros the adjusted accelerometer values
#the first 5 seconds are used 

import serial, time

print("Initializing serial connection")
#initialize arduino
arduino = serial.Serial('/dev/ttyACM0',115200)

print("Connection established")

print("Calibrating IMU")
startRead = time.time()
while time.time() - startRead < 3:
	a = arduino.readline() 
	print(a)

arduino.close()