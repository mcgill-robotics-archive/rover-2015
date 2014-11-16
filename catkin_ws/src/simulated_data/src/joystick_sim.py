#!/usr/bin/env python

from geometry_msgs.msg import Twist
import rospy
import math


def publish_twist_continuous():
    rospy.init_node('joy_sim', anonymous=False)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    MAX_LIN_VEL = rospy.get_param('/controls/max_lin_vel', 0.1)
    joy_lin = -math.pi
    while not rospy.is_shutdown():
        if joy_lin > math.pi:
            joy = -math.pi

        v_body = math.sin(joy_lin)*MAX_LIN_VEL
        w_body = 0

        joy_lin = joy_lin + math.radians(2)
        
        twist = Twist()

        twist.linear.x = v_body
        twist.angular.z = 0
        
        rospy.loginfo(twist)

        publisher.publish(twist)

        r = rospy.Rate(10) # 10hz
        r.sleep()

if __name__ == '__main__':
    try:
        publish_twist_continuous()
    except KeyboardInterrupt:
        print "Exit"
