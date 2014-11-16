#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def publish_integer():
    rospy.init_node('int_publisher', anonymous=True)
    pub = rospy.Publisher('integers', Int16, queue_size=10)
 
    while not rospy.is_shutdown():

        for x in range(10):
    
            str=x

            rospy.loginfo(str)
            pub.publish(str)

            r = rospy.Rate(1) # 1hz
            r.sleep()

    
if __name__ == '__main__':
    publish_integer()

