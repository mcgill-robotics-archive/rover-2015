#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo("%s is received"%data.data[13:len(data.data)])

def listener():
	rospy.init_node('listener',anonymous=True)
	rospy.Subscriber("chatter",String, callback)
	rospy.spin()

if __name__ == '__main__' :
	listener()