#!/usr/bin/env python
#Program simulates readings from two joysticks and publishes
from geometry_msgs.msg import Twist
import rospy
import math
from std_msgs.msg import Int8, String
from control_systems.msg import MotionType


#mode refers to whether lin velocity or angular velocity should be
#kept constant
def publish_twist_continuous(mode):
    rospy.init_node('joy_sim', anonymous=False)
    joyPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    typePublisher = rospy.Publisher('/cmd_motion',MotionType,queue_size=10)
    #alternative joystick publisher
    altJoyPublisher = rospy.Publisher('/cmd_alt_vel', Twist, queue_size=10)
    motionFloat = 0.
    motion = MotionType()
    motion.ACKERMANN = 0
    motion.POINT = 0
    motion.TRANSLATORY = 0
    motion.SWERVE = 1

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
