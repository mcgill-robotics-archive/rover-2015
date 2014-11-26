#!/usr/bin/env python
#Program simulates readings from a joystick and publishes them to "cmd_vel" 
from geometry_msgs.msg import Twist
import rospy
import math
from std_msgs.msg import Int8


def publish_twist_continuous():
    rospy.init_node('joy_sim', anonymous=False)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    typePublisher = rospy.Publisher('cmd_motion',Int8,queue_size=10)
    motionFloat = 0.
    motion = Int8()
    motion.data = 0
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
            motion.data = 0
            typePublisher.publish(motion) #ackermann
        elif motionFloat < 4+2:
            motion.data = 1
            typePublisher.publish(motion) #point
        elif motionFloat < 5+4:
            motion.data = 2
            typePublisher.publish(motion)
        else:
            motionFloat = 0

        twist = Twist()

        twist.linear.x = v_body
        twist.angular.z = w_body
        
        rospy.loginfo(twist)
        rospy.loginfo(motion)

        publisher.publish(twist)

        r = rospy.Rate(10) # 10hz
        r.sleep()

if __name__ == '__main__':
    try:
        publish_twist_continuous()
    except KeyboardInterrupt:
        print "Exit"
