#!/usr/bin/python

import rospy
from geometry_msgs.msg import Pose



def publish_integer():
    rospy.init_node('pose_publisher', anonymous=True)
    pub = rospy.Publisher('pose', Pose, queue_size=10)
    x=0
    while not rospy.is_shutdown():
            msg=Pose()
            msg.position.x=x
            msg.position.y=x

            x=x+1

            rospy.loginfo(msg)
            pub.publish(msg)

            r = rospy.Rate(1) # 1hz
            r.sleep()

    
if __name__ == '__main__':
    publish_integer()
