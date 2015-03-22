#!/usr/bin/python
##This program will eventually need to be ported to cpp for speed!

#program will read in values from imu, calibrate, and then
#send to ros the adjusted accelerometer values
#the first 5 seconds are used 

#serial for serial connection, time for function timing, 
#numpy for median, regex (re) for number scraping
import serial, time, numpy, re

#regular expression for numbers
renum = '-?[0-9]+\.?[0-9]+'


print("Initializing serial connection")

try:
	#initialize arduino at port and specified baud rate
	arduino = serial.Serial('/dev/ttyACM0',115200)
except serial.serialutil.SerialException:
	print("Connection could not be established")
	quit()

print("Connection established")

print("Calculating Gyro Bias")
startRead = time.time()

serialCalibData = []
calibValues = []
#read from IMU for specified seconds to calibrate
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
			#make them float after removing non numericals
			values = [float(x) for x in a]
			calibValues.append(values)
	except: continue
#calculate median bias for gyro and accelerometer
gyroBiasX = numpy.median([x[0] for x in calibValues])
gyroBiasY = numpy.median([x[1] for x in calibValues])
gyroBiasZ = numpy.median([x[2] for x in calibValues])
acelBiasX = numpy.median([x[3] for x in calibValues])
acelBiasY = numpy.median([x[4] for x in calibValues])
acelBiasZ = numpy.median([x[5] for x in calibValues])

print ("gyroscope x bias = ", gyroBiasX)
print ("gyroscope y bias = ", gyroBiasY)
print ("gyroscope z bias = ", gyroBiasZ)
print ("accelerometer x bias = ", acelBiasX)
print ("accelerometer y bias = ", acelBiasY)
print ("accelerometer z bias = ", acelBiasZ)

#Current bias seems to be around -200 for each axis (flat 
#IMU with leads facing up)
#http://www.i2cdevlib.com/forums/topic/91-how-to-decide-gyro-and-accelerometer-offsett/#entry257

arduino.close()