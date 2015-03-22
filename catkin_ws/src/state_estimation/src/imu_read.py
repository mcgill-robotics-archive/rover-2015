#!/usr/bin/python
##This program will eventually need to be ported to cpp for speed!

#program will read in values from imu, calibrate, and then
#send to ros the adjusted accelerometer values
#the first 5 seconds are used 

#serial for serial connection, time for function timing, 
#numpy for median, regex (re) for number scraping
import serial, time, numpy, re, rospy
from state_estimation.msg import Imu

#regular expression for numbers
renum = '-?[0-9]+\.?[0-9]+'

class IMUReader(object):
    def __init__ (self):
        #initiate node
        rospy.init_node('imu_reader')
        #publish imu settings to this topic
        self.pubImu = rospy.Publisher('/imu',Imu,queue_size=10,latch=10)
        #settings to be read in
        self.settings = Imu()
        #set values to 0
        self.settings.gyroX = 0
        self.settings.gyroY = 0
        self.settings.gyroZ = 0
        self.settings.acelX = 0
        self.settings.acelY = 0
        self.settings.acelZ = 0
        self.arduino = 0
        try:
            #initialize arduino at port and specified baud rate
            self.arduino = serial.Serial('/dev/ttyACM0',115200)
        except serial.serialutil.SerialException:
            print("Connection could not be established")
            quit()

        ##Calibration Stage
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
        self.gyroBiasX = numpy.median([x[0] for x in calibValues])
        self.gyroBiasY = numpy.median([x[1] for x in calibValues])
        self.gyroBiasZ = numpy.median([x[2] for x in calibValues])
        self.acelBiasX = numpy.median([x[3] for x in calibValues])
        self.acelBiasY = numpy.median([x[4] for x in calibValues])
        #MPU 6050 gives +16384 for right side up. This program interprets
        #which way the accelerometer is pointed, and interprets from there
        #get reading, then modify for correct value
        self.acelBiasZ = numpy.median([x[5] for x in calibValues])
        #points correct way
        if self.acelBiasZ > 0:
            self.acelBiasZ -= 16384
        else: #upside down!
            self.acelBiasZ += 16384

    def update_settings(self):
        a = self.arduino.readline().split(',')
		#the following will fail if there is no number in the string
		try:
			#pull out numeric part of string
			a = [re.search(renum,y).group(0) for y in a]
			if len(a) == 6:
				#make them float after removing non numericals
				values = [float(y) for y in a]
				#correct all according to biases
				values[0] -= gyroBiasX
				values[1] -= gyroBiasY
				values[2] -= gyroBiasZ
				values[3] -= acelBiasX
				values[4] -= acelBiasY
				values[5] -= acelBiasZ
				self.settings.gyroX = values[0]
				self.settings.gyroY = values[1]
				self.settings.gyroZ = values[2]
				#convert the following to standard units (m/s^2)
				self.settings.acelX = 9.81*values[3]/16384
				self.settings.acelY = 9.81*values[4]/16384
				self.settings.acelZ = 9.81*values[5]/16384

    #function will publish at 60Hz
    def run(self):
        r = rospy.Rate(60)
        #continue until quit
        while not rospy.is_shutdown():
        	#get new values
        	self.update_settings()
            #publish to topic
            self.pubImu.publish(self.settings)
            #next iteration
            r.sleep()

    #exit
    def close(self):
    	self.arduino.close()

