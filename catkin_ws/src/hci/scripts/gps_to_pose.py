#!/usr/bin/python

import rospy
from rover_msgs.msg import AhrsStatusMessage
from geometry_msgs.msg import Pose


def handle_pose(msg):
    pose = Pose()
    pose.position.x = msg.gpsLatitude
    pose.position.y = msg.gpsLongitude
    pose.orientation.z = msg.heading
    pub.publish(pose)

rospy.init_node('gpsToPose', anonymous=True)
pub = rospy.Publisher('pose', Pose, queue_size=10)
rospy.Subscriber('/ahrs_status', AhrsStatusMessage, handle_pose)
rospy.spin()
