#!/usr/bin/env python
#This program simulates control of a robotic arm, and publishes to 
#the topic "cmd_arm"

import rospy, math, time
from control_systems.msg import ArmMotion

def publish_arm_motion_continuous(simulation):
	#declare type of node
	rospy.init_node('arm_sim',anonymous=False)
	#object which publishes arm control data from joystick
	armPublisher = rospy.Publisher('/cmd_arm', ArmMotion, queue_size=10)

	#variable to be published
	armSettings = ArmMotion()
	#coordinates in x-y plane (0 to 1 (max))
	armSettings.x = 0.
	armSettings.y = 0.
	#angle of x-y plane
	armSettings.theta = 0.
	armSettings.on = True
	armSettings.cartesian = False
	motionFloat = 0.
	#step to be repeated
	#first simulation just moves the arms in one giant loop through
	#all axes of freedom, using non-cartesian coordinates
	if simulation == 0:
		while not rospy.is_shutdown():
			#cycle through
			if motionFloat > 2*math.pi:
				motionFloat = 0.
			else:
				motionFloat += 0.01
			#x and y move slowly in a circle	
			armSettings.x = 0.3*math.cos(motionFloat)+1.6
			armSettings.y = 0.5*math.sin(motionFloat)-0.25
			#theta just moves back and forth
			if motionFloat < math.pi:
				armSettings.theta = (motionFloat-math.pi/2.)/4.
			else:
				armSettings.theta = (3*math.pi/2-motionFloat)/4.

			rospy.loginfo(armSettings)

			armPublisher.publish(armSettings)
			#60 Hz processing cycle
			r = rospy.Rate(60)
			r.sleep()
	elif simulation == 1:
		armSettings.cartesian = True
		#constant x,y settings
		armSettings.x = 0.6
		armSettings.y = 0.5
		while not rospy.is_shutdown():
			#move the arm along a line in front of the robot,
			#cycling over time
			armSettings.theta = math.sin(time.clock())

			rospy.loginfo(armSettings)

			armPublisher.publish(armSettings)
			#60 Hz processing cycle
			r = rospy.Rate(60)
			r.sleep()


if __name__ == '__main__':
	try:
		#decide which simulation to run
		publish_arm_motion_continuous(0)
	except KeyboardInterrupt:
		print "Exit"

