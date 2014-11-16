#!/usr/bin/env python
from mappingsteer import steer #convert linear,angular speed to wheel settings
import rospy #for reading and publishing to topics
from control_systems.msg import SetPoints #type of wheel setting output

def talker(x,w):
	input = steer(x,w)
	pub = rospy.Publisher('wheels',SetPoints,queue_size=10)
	rospy.init_node('joystick_reader',anonymous=True)
	r = rospy.Rate(10) #UPDATE FFINAL!
	while not rospy.is_shutdown():
		settings = SetPoints()
		settings.thetaFL = input[0]
		settings.thetaFR = input[1]
		settings.thetaRL = input[2]
		settings.thetaRR = input[3]
		settings.speedMW = input[4]
		rospy.loginfo(settings)
		pub.publish(settings)
		r.sleep()
	return 0 


if __name__ == '__main__':
	try:
		talker(1,1)
	except rospy.ROSInterruptException: pass