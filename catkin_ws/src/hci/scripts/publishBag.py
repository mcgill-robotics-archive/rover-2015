#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker1():
	pub1=rospy.Publisher('chatter1',String, queue_size=10)
	pub2=rospy.Publisher('chatter2',String, queue_size=10)
	pub3=rospy.Publisher('chatter3',String, queue_size=10)
	rospy.init_node('talker',anonymous=True)
	r=rospy.Rate(10)
	a=1;
	while not rospy.is_shutdown():
		str1="first message: %d\n"%a
		str2="second message:%d\n"%a
		str3="third message: %d\n"%a
		a=a+1
		if a>10:
			a=0
		pub1.publish(str1)
		pub2.publish(str2)
		pub3.publish(str3)
		r.sleep()

if __name__ == '__main__':
	try:
		talker1()
	except rospy.ROSInterruptException: pass

