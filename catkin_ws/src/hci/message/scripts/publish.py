#!/usr/bin/env python
import rospy
import thread
import time
from std_msgs.msg import String

a=1

def publish():
	pub=rospy.Publisher('chatter',String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	while not rospy.is_shutdown():
		str="the value is %d"%a
		#rospy.loginfo(str)
		print str
		pub.publish(str)
		time.sleep(0.1)
def main(no,interval):
	global a
	while  not rospy.is_shutdown():
		a=a+1;
		time.sleep(interval)
def task():
	thread.start_new_thread(main,(1,0.1))

if __name__ == '__main__':
	try:
		task()
		publish()
	except rospy.ROSInterruptException: pass