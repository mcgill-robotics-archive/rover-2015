import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from control_systems.msg import MotionType, PanTiltZoom, ArmMotion, EndEffector

class Publisher(object):
    def __init__(self):
        #TODO change names once control systems has a defined topic name and variable names, copied from AUV as of now
        self.cam_pub = rospy.Publisher(rospy.get_param("electrical_interface/cameraPos_topic","electrical_interface/camera"),PanTiltZoom, queue_size=10)
        self.vel_pub = rospy.Publisher(rospy.get_param("cmd_vel_topic","cmd_vel"),Twist, queue_size=10)
        self.arm_movement_pub = rospy.Publisher(rospy.get_param("electrical_interface/arm_topic","/cmd_arm"),ArmMotion, queue_size=10)
        self.arm_endEffector_pub = rospy.Publisher(rospy.get_param("cmd_endEffector_topic","cmd_endEffector"),EndEffector,queue_size=10)
        self.motionTypePublisher = rospy.Publisher(rospy.get_param("cmd_motion_topic","cmd_motion"),MotionType, queue_size=10)



    #publisher for velocity
    def publish_velocity(self, a1, a2):
        """
        Publish linear and angular command velocity in twist for control systems
        """
        msg = Twist()
        msg.linear.x = a2
        msg.angular.z = a1

        self.vel_pub.publish(msg)

    #publish 2 main joystick axes for arm base movement (mode must be arm)
    def publish_arm_base_movement(self, armLength, armHeight, angle):
        """
        Publish base arm position
        """
        msg = ArmMotion()
        msg.x = armLength # y axis on joystick moves the target point forward and back
        msg.y = armHeight # rotation of the joystick moves the target point up and down
        msg.theta = angle
        msg.on = True
        self.arm_movement_pub.publish(msg)
    
    def publish_endEffector(self, x, y, rotate, grip):
        msg = EndEffector()
        msg.x = x
        msg.y = y
        msg.theta = rotate
        msg.grip = grip # grip is either 1,0, or -1.
        self.arm_endEffector_pub.publish(msg)
    

    #publish camera zoom from axis 4
    def publish_camera(self, a3, a4):
        msg = PanTiltZoom()
        msg.pan=a3
        msg.tilt=a4
        self.cam_pub.publish(msg)


    def setSteerMode(self, boolean):
        motionType = MotionType()
        
        motionType.TRANSLATORY=0
        motionType.SWERVE=0
        if boolean:
            motionType.ACKERMANN=0
            motionType.POINT=1
        else:
            motionType.ACKERMANN=1
            motionType.POINT=0

        self.motionTypePublisher.publish(motionType)
