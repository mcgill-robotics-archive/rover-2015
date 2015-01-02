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
    motion = MotionType()
    #this is the permanent setting for this
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

        twist = Twist()
        altTwist = Twist()
        
        if not mode:
                
            #keep the following constant so angular motion can be seen
            twist.linear.x = 1
            twist.angular.z = 0
            altTwist.linear.x = 0
            altTwist.angular.z = 1
            
        else:
    
            twist.linear.x = v_body
            twist.angular.z = w_body
            #keep the following constant so angular motion can be seen
            altTwist.linear.x = 0
            altTwist.angular.z = 1 
            
        #publish all to ros
        #initiate message
        logMessage = String()
        logMessage.data = "\n________________________________________"
        logMessage.data += "\n\n\nINITIATE MESSAGE\n\n\n"
        rospy.loginfo(logMessage)
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
        #if 0 is the argument, the rover should just go straight
        #while spinning
        publish_twist_continuous(0)
    except KeyboardInterrupt:
        print "Exit"                       
