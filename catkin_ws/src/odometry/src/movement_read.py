#!/usr/bin/python

from math import sin, cos, tan, pi, sqrt
import rospy
from control_systems.msg import SetPoints,Moving,MotionType 
from odometry.msg import RoverSpeed


# distance between wheels of: front and middle/middle and rear[m]
D = rospy.get_param('control/wh_distance_fr',0.5)
# distance between longitudinal axis and port/starboard wheels[m]
B = rospy.get_param('control/wh_base',0.4)
R = rospy.get_param('control/wh_radius',0.165) # wheel radius [m]
W = rospy.get_param('control/wh_width',0.15) # wheel width [m]

#distance from center of rover to corner wheel
cornerRadius = sqrt(D**2 + B**2)



class MovementReader(object):
	def __init__(self):
		rospy.init_node('movement_reader') #Name of this node
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
			self.speeds.angular) = findRoverSpeeds(self.settings)

	def run(self):
		#calculate required wheel angles, speeds
		r = rospy.Rate(60)
        #continue endlessly
		while not rospy.is_shutdown():
			#log rospy speed
			rospy.loginfo(self.speeds)
			r.sleep()


def findRoverSpeeds (settings):
	cumulativeLinVelocity = 0
	#linear velocity is predicted from average linear velocity of all
	#four corner wheels
	cumulativeLinVelocity += cos(settings.thetaFL)*settings.speedFL
	cumulativeLinVelocity += cos(settings.thetaFR)*settings.speedFR
	cumulativeLinVelocity += cos(settings.thetaRL)*settings.speedRL
	cumulativeLinVelocity += cos(settings.thetaRR)*settings.speedRR
	cumulativeLinVelocity += settings.speedML
	cumulativeLinVelocity += settings.speedMR
	#also factor in size of the wheel
	averageLinVelocity = R*cumulativeLinVelocity/6

	#Next predict average position of rotational axis (on cross-section line)

	#go through corner wheels, and draw line from each; find intersection point
	#evaluate each and add to total evaluated (if one wheel perpendicular, 
	#does not count)
	evaluated = 0
	cumulativeAngVelocity = 0
	#test if angle is reasonable
	if abs(settings.thetaFL) > 1e-5:
		#get intersection point (treat cross-section like number line)
		evaluated += 1
		#distance to the wheel's rotational axis is D/sin(settings.thetaFL)
		cumulativeAngVelocity += settings.speedFL*sin(settings.thetaFL)/D
	if abs(settings.thetaFR) > 1e-5:
		evaluated += 1
		cumulativeAngVelocity += settings.speedFL*sin(settings.thetaFR)/D
	if abs(settings.thetaRL) > 1e-5:
		evaluated += 1
		cumulativeAngVelocity -= settings.speedFL*sin(settings.thetaRL)/D
	if abs(settings.thetaRR) > 1e-5:
		evaluated += 1
		cumulativeAngVelocity -= settings.speedFL*sin(settings.thetaRR)/D
	#if any angle was slightly non-perpendicular
	if evaluated > 0:
		averageAngVelocity = R*cumulativeAngVelocity/evaluated
	else:
		averageAngVelocity = 0
	
	#also test for difference of two middle wheel speeds - use to get another


	return (averageLinVelocity, averageAngVelocity)

if __name__ == '__main__':
	print "Initializing Node"
	speedReader = MovementReader()
	print "Running Node"
	speedReader.run()
	rospy.spin()