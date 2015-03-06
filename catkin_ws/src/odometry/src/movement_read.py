#!/usr/bin/python

from math import sin, cos, tan, pi
import rospy
from control_systems.msg import SetPoints,Moving,MotionType 
from odometry.msg import RoverSpeed


# distance between wheels of: front and middle/middle and rear[m]
D = rospy.get_param('control/wh_distance_fr',0.5)
# distance between longitudinal axis and port/startboard wheels[m]
B = rospy.get_param('control/wh_base',0.4)
R = rospy.get_param('control/wh_radius',0.165) # wheel radius [m]
W = rospy.get_param('control/wh_width',0.15) # wheel width [m]



class MovementReader(object):
	def __init__(self):

		#wheel settings
		self.settings = SetPoints()
		#speeds of the rover
		self.speeds = RoverSpeed()

		#download all of the wheel settings from this topic
		#(((((eventually this will read from the encoders??)))))
		rospy.Subscriber('/wheels',SetPoints,self.update_wheels,
			queue_size=10)

	def update_wheels(self, msg):
		#update all settings from wheels
		self.settings.thetaFL = msg.thetaFL
		self.settings.thetaFR = msg.thetaFR
		self.settings.thetaRL = msg.thetaRL
		self.settings.thetaRR = msg.thetaRR
		self.settings.speedFL = msg.speedFL
		self.settings.speedFR = msg.speedFR
		self.settings.speedML = msg.speedML
		self.settings.speedMR = msg.speedMR
		self.settings.speedRL = msg.speedRL
		self.settings.speedRR = msg.speedRR

		(self.speeds.linear,
			self.speeds.angular) = findRoverSpeeds(settings)

	def run(self):
		#calculate required wheel angles, speeds
		r = rospy.Rate(60)
        #continue endlessly
		while not rospy.is_shutdown():
			#log rospy speed
			rospy.loginfo(self.speeds)
			r.sleep()


def findRoverSpeeds (settings):
	totalVelocity = 0
	totalVelocity += cos(settings.thetaFL)*settings.speedFL
	totalVelocity += cos(settings.thetaFR)*settings.speedFR
	totalVelocity += cos(settings.thetaRL)*settings.speedRL
	totalVelocity += cos(settings.thetaRR)*settings.speedRR
	averageVelocity = totalVelocity/4

	return (averageVelocity, 0)