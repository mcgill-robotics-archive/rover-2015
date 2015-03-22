#!/usr/bin/python
##This program will eventually need to be ported to cpp for speed!

#program will read in values from imu, calibrate, and then
#send to ros the adjusted accelerometer values
#the first 5 seconds are used 

#serial for serial connection, time for function timing, 
#numpy for median, regex (re) for number scraping
import serial, time, numpy, re

renum = '-?[0-9]+\.?[0-9]+'

print("Initializing serial connection")
#initialize arduino
arduino = serial.Serial('/dev/ttyACM0',115200)

print("Connection established")

print("Calculating Bias")
startRead = time.time()

serialCalibData = []
gyroCalibValues = []
#read from IMU for 5 seconds to calibrate
while time.time() - startRead < 1:
	#split up into list
	a = arduino.readline()
	serialCalibData.append(a)


for x in serialCalibData:
	a = x.split(',')
	#the following will fail if there is no number in the string
	try:
		#pull out numeric part of string
		a = [re.search(renum,x).group(0) for x in a]
		if len(a) == 6:
			#first 3 values are gyroscope values
			#make them float after removing non numericals
			gyroValues = [float(x) for x in a[0:3]]
			gyroCalibValues.append(gyroValues)
	except: continue

#calculate median bias
gyroBiasX = numpy.median(gyroCalibValues[:][0])
gyroBiasY = numpy.median(gyroCalibValues[:][1])
gyroBiasZ = numpy.median(gyroCalibValues[:][2])

print ("gyroscope x bias = ", gyroBiasX)
print ("gyroscope y bias = ", gyroBiasY)
print ("gyroscope z bias = ", gyroBiasZ)

arduino.close()