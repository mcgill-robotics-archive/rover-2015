import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

class Publisher(object):
    def __init__(self):
        #TODO change names once control systems has a defined topic name and variable names, copied from AUV as of now
        self.vel_pub = rospy.Publisher("electrical_interface/motor",Twist, queue_size=10)
        self.arm_movement_pub = rospy.Publisher("electrical_interface/arm",Twist, queue_size=10)
        self.arm_rotate_pub = rospy.Publisher("electrical_interface/arm",Int16, queue_size=10)
        self.zoom_pub = rospy.Publisher("electrical_interface/cameraZoom",Int16, queue_size=10)
        self.pan_pub = rospy.Publisher("electrical_interface/cameraPan",Int16, queue_size=10)
        print "publisher initialized"


    #publisher for velocity
    def publish_velocity(self, a1, a2):
        print "Hello World"
        """
        Publish linear and angular command velocity in twist for control systems
        """
        msg = Twist()
        msg.linear.x = a1
        msg.angular.z = a2

        self.vel_pub.publish(msg)

    #publish 2 main joystick axes for arm base movement (mode must be arm)
    def publish_arm_base_movement(self, a1, a2):
        """
        Publish base arm position
        """
        msg = Twist()
        msg.linear.x = a1
        msg.linear.y = a2
        self.arm_movement_pub.publish(msg)

    #publish joystick3 for rotating hand (mode must be arm)
    def publish_arm_rotation(self, a3):
        msg = a3
        self.arm_rotate_pub(msg)

    #publish camera zoom from axis 4
    def publish_zoom(self, a4):
        msg = a4
        self.zoom_pub.publish(msg)

    #publish camera pan from axis 3 (mode must be drive)
    def publish_pan(self, a3):
        msg = a3
        self.pan_pub.publish(msg)
