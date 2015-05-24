#!/usr/bin/python

import rospy
from rover_msgs.msg import GPS
from geometry_msgs.msg import Pose


def handle_pose(msg):
    pose = Pose()
    pose.position.x = msg.latitude
    pose.position.y = msg.longitude

    pub.publish(pose)

rospy.init_node('gpsToPose', anonymous=True)
pub = rospy.Publisher('pose', Pose, queue_size=10)
rospy.Subscriber('/raw_gps', GPS, handle_pose)
rospy.spin()
