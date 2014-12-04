#!/usr/bin/env python
#Program simulates readings from a joystick and publishes them to "cmd_vel" 
from geometry_msgs.msg import Twist
import rospy
import math
from std_msgs.msg import Int8
from control_systems.msg import MotionType


def publish_twist_continuous():
    rospy.init_node('joy_sim', anonymous=False)
    publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    typePublisher = rospy.Publisher('/cmd_motion',MotionType,queue_size=10)
    motionFloat = 0.
    motion = MotionType()
    motion.ACKERMANN = 1
    motion.POINT = 0
    motion.TRANSLATORY = 0

    MAX_LIN_VEL = rospy.get_param('/controls/max_lin_vel', 0.1)
    joy_lin = -math.pi
    joy_ang = math.pi
    while not rospy.is_shutdown():
        if joy_lin > math.pi:
            joy_lin = -math.pi
        
        v_body = math.sin(joy_lin)*MAX_LIN_VEL
        w_body = math.cos(joy_lin)*MAX_LIN_VEL

        joy_lin = joy_lin + math.radians(2)
        motionFloat += 0.05
        if motionFloat < 3:
            motion.ACKERMANN = 1
            motion.POINT = 0
            motion.TRANSLATORY = 0 #ackermann
        elif motionFloat < 4+2:
            motion.ACKERMANN = 0
            motion.POINT = 1 #eventually five different numbers will work here
            motion.TRANSLATORY = 0 #point
        elif motionFloat < 5+4:
            motion.ACKERMANN = 0
            motion.POINT = 0
            motion.TRANSLATORY = 1
        else:
            motionFloat = 0

        twist = Twist()

        twist.linear.x = v_body
        twist.angular.z = w_body
        
        rospy.loginfo(twist)
        rospy.loginfo(motion)

        publisher.publish(twist)
        typePublisher.publish(motion)

        r = rospy.Rate(10) # 10hz
        r.sleep()

if __name__ == '__main__':
    try:
        publish_twist_continuous()
    except KeyboardInterrupt:
        print "Exit"
