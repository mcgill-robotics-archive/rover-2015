#!/usr/bin/python
##This program will eventually need to be ported to cpp for speed!

#program will read in values from imu, calibrate, and then
#send to ros the adjusted accelerometer values
#the first 5 seconds are used 

#serial for serial connection, time for function timing, 
#numpy for median, regex (re) for number scraping
import serial, time, numpy, re, rospy

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
		a = [re.search(renum,y).group(0) for y in a]
		if len(a) == 6:
			#make them float after removing non numericals
			values = [float(y) for y in a]
			calibValues.append(values)
	except: continue
#calculate median bias for gyro and accelerometer
gyroBiasX = numpy.median([x[0] for x in calibValues])
gyroBiasY = numpy.median([x[1] for x in calibValues])
gyroBiasZ = numpy.median([x[2] for x in calibValues])
acelBiasX = numpy.median([x[3] for x in calibValues])
acelBiasY = numpy.median([x[4] for x in calibValues])
#MPU 6050 gives +16384 for right side up. This program interprets
#which way the accelerometer is pointed, and interprets from there
#get reading, then modify for correct value
acelBiasZ = numpy.median([x[5] for x in calibValues])

#points correct way
if acelBiasZ > 0:
	acelBiasZ -= 16384
else: #upside down!
	acelBiasZ += 16384

print ("gyroscope x bias = ", gyroBiasX)
print ("gyroscope y bias = ", gyroBiasY)
print ("gyroscope z bias = ", gyroBiasZ)
print ("accelerometer x bias = ", acelBiasX)
print ("accelerometer y bias = ", acelBiasY)
print ("accelerometer z bias = ", acelBiasZ)

#http://www.i2cdevlib.com/forums/topic/91-how-to-decide-gyro-and-accelerometer-offsett/#entry257

###Current test values are:
#('gyroscope x bias = ', 20.0)
#('gyroscope y bias = ', -487.0)
#('gyroscope z bias = ', -215.0)
#('accelerometer x bias = ', 2512.0)
#('accelerometer y bias = ', -1220.0)
#('accelerometer z bias = ', -480.0)

startRead = time.time()
#now, for readings, need to subtract bias from each. 
#continue for specified number of seconds
while time.time() - startRead < 3:
	a = arduino.readline().split(',')
	#the following will fail if there is no number in the string
	try:
		#pull out numeric part of string
		a = [re.search(renum,y).group(0) for y in a]
		if len(a) == 6:
			#make them float after removing non numericals
			values = [float(y) for y in a[3:]]
			#correct all according to biases
			values[0] -= acelBiasX
			values[1] -= acelBiasY
			values[2] -= acelBiasZ
			print "The accelerations are :"
			print "x =", 9.81*values[0]/16384, "m/s^2"
			print "y =", 9.81*values[1]/16384, "m/s^2"
			print "z =", 9.81*values[2]/16384, "m/s^2"
	except: continue


arduino.close()