#!/usr/bin/env python
#Program simulates readings from a joystick and publishes them to "cmd_vel" 
from geometry_msgs.msg import Twist
import rospy
import math
from std_msgs.msg import Int8
from control_systems.msg import MotionType


def publish_twist_continuous():
    rospy.init_node('joy_sim', anonymous=False)
    joyPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    typePublisher = rospy.Publisher('/cmd_motion',MotionType,queue_size=10)
    #alternative joystick publisher
    altJoyPublisher = rospy.Publisher('/cmd_alt_vel', Twist, queue_size=10)
    motionFloat = 0.
    motion = MotionType()
    motion.ACKERMANN = 1
    motion.POINT = 0
    motion.TRANSLATORY = 0
    motion.SWERVE = 0

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
            motion.SWERVE = 0
        elif motionFloat < 6:
            motion.ACKERMANN = 0
            motion.POINT = 1 
            motion.TRANSLATORY = 0 #point
            motion.SWERVE = 0
        elif motionFloat < 9:
            motion.ACKERMANN = 0
            motion.POINT = 0
            motion.TRANSLATORY = 1
            motion.SWERVE = 0
        elif motionFloat < 12:
            #swerve drive
            motion.ACKERMANN = 0
            motion.POINT = 0
            motion.TRANSLATORY = 0
            motion.SWERVE = 1
        else:
            motionFloat = 0

        twist = Twist()
        altTwist = Twist()

        twist.linear.x = v_body
        twist.angular.z = w_body

        #for now, make the second joystick move inverse to the first
        altTwist.linear.x = -v_body
        altTwist.angular.z = -w_body
        
        #publish all to ros
        rospy.loginfo(twist)
        rospy.loginfo(motion)
        rospy.loginfo(altTwist)

        joyPublisher.publish(twist)
        typePublisher.publish(motion)
        altJoyPublisher.publish(altTwist)

        r = rospy.Rate(10) # 10hz
        r.sleep()

if __name__ == '__main__':
    try:
        publish_twist_continuous()
    except KeyboardInterrupt:
        print "Exit"
