#!/usr/bin/env python
#This program simulates control of a robotic arm, and publishes to 
#the topic "cmd_arm"

import rospy, math
from control_systems.msg import ArmMotion

def publish_arm_motion_continuous():
	#declare type of node
	rospy.init_node('arm_sim',anonymous=False)
	#object which publishes arm control data from joystick
	armPublisher = rospy.Publisher('/cmd_arm', ArmMotion, queue_size=10)


	armSettings = ArmMotion()
	armSettings.x = 0.
	armSettings.y = 0.
	armSettings.theta = 0.
	armSettings.on = True
	motionFloat = 0.

	while not rospy.is_shutdown():
		if motionFloat > 2*math.pi:
			motionFloat = 0.
		else:
			motionFloat += 0.01

		armSettings.x = 0.5*math.cos(motionFloat)+1
		armSettings.y = 0.5*math.sin(motionFloat)+1
		armSettings.theta = (motionFloat-math.pi)/4.

		rospy.loginfo(armSettings)

		armPublisher.publish(armSettings)

		r = rospy.Rate(60)
		r.sleep()

if __name__ == '__main__':
	try:
		publish_arm_motion_continuous()
	except KeyboardInterrupt:
		print "Exit"

