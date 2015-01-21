import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from control_systems.msg import MotionType

class Publisher(object):
    def __init__(self):
        #TODO change names once control systems has a defined topic name and variable names, copied from AUV as of now
        self.vel_pub = rospy.Publisher("cmd_vel",Twist, queue_size=10)
        self.arm_movement_pub = rospy.Publisher("electrical_interface/arm",Twist, queue_size=10)
        self.arm_rotate_pub = rospy.Publisher("electrical_interface/arm",Int16, queue_size=10)
        self.zoom_pub = rospy.Publisher("electrical_interface/cameraZoom",Int16, queue_size=10)
        self.pan_pub = rospy.Publisher("electrical_interface/cameraPan",Int16, queue_size=10)
        self.pan_pub = rospy.Publisher("electrical_interface/cameraPan",Int16, queue_size=10)
        self.motionTypePublisher = rospy.Publisher("cmd_motion",MotionType, queue_size=10)



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
    def publish_arm_base_movement(self, armLength, armHeight):
        """
        Publish base arm position
        """
        msg = Twist()
        msg.linear.x = armLength # y axis on joystick moves the target point forward and back
        msg.linear.y = armHeight # rotation of the joystick moves the target point up and down
        self.arm_movement_pub.publish(msg)

    #publish joystick3 for rotating hand (mode must be arm)
    def publish_arm_rotation(self, angle):
        msg = angle # x axis on joystick rotates the angle of the armbase.
        self.arm_rotate_pub(msg)

    #publish camera zoom from axis 4
    def publish_zoom(self, a4):
        msg = a4
        self.zoom_pub.publish(msg)

    #publish camera pan from axis 3 (mode must be drive)
    def publish_pan(self, a3):
        msg = a3
        self.pan_pub.publish(msg)

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
