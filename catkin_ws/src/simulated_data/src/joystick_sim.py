#!/usr/bin/env python

from geometry_msgs.msg import Twist
import rospy


def publish_twist_continuous():
    rospy.init_node('joy_sim', anonymous=False)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        twist = Twist()

        rospy.loginfo('[[1,0,0],[0,0,0]')
        for x in range(20):
            twist.linear.x = 1

            publisher.publish(twist)

            r = rospy.Rate(10) # 10hz
            r.sleep()

        rospy.loginfo('[[0,0,0],[0,0,0]')
        for x in range(20):
            twist.linear.x = 0

            publisher.publish(twist)

            r = rospy.Rate(10) # 10hz
            r.sleep()

        rospy.loginfo('[[0,0,0],[0,0,1]')
        for x in range(20):
            twist.angular.z = 1

            publisher.publish(twist)

            r = rospy.Rate(10) # 10hz
            r.sleep()

        rospy.loginfo('[[0,0,0],[0,0,0]')
        for x in range(20):
            twist.angular.z = 0

            publisher.publish(twist)

            r = rospy.Rate(10) # 10hz
            r.sleep()

if __name__ == '__main__':
    try:
        publish_twist_continuous()
    except KeyboardInterrupt:
        print "Exit"