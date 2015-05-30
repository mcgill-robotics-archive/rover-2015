import rospy

from geometry_msgs.msg import Twist
from control_systems.msg import MotionType, PanTiltZoom, ArmMotion, EndEffector, ArmAngles
from std_msgs.msg import Bool, Int16


class Publisher(object):
    def __init__(self):
        # TODO change names once control systems has a defined topic name and variable names, copied from AUV as of now
        self.cam_pub = rospy.Publisher("/camera_orientation",
                                       PanTiltZoom, queue_size=10)
        self.vel_pub = rospy.Publisher("wheels", Int16, queue_size=10)
        self.arm_movement_pub = rospy.Publisher(rospy.get_param("electrical_interface/arm_topic", "/cmd_arm"),
                                                ArmMotion, queue_size=10)
        self.arm_endEffector_pub = rospy.Publisher(rospy.get_param("cmd_endEffector_topic", "cmd_endEffector"),
                                                   EndEffector, queue_size=10)
        self.motionTypePublisher = rospy.Publisher(rospy.get_param("cmd_motion_topic", "cmd_motion"),
                                                   MotionType, queue_size=10)
        self.moving_bool_pub = rospy.Publisher("is_moving", Bool, queue_size=10)
        self.yolo_arm = rospy.Publisher("arm", ArmAngles, queue_size=10)
        self.yolo_claw = rospy.Publisher("claw", Int16, queue_size=10)

    def yolo_claw_pub(self, op)
        msg = Int16(op)
        self.yolo_claw.publish(msg)

    # publisher for velocity
    def publish_velocity(self, a1, a2, on):
        """
        Publish linear and angular command velocity in twist for control systems
        """
        msg = Int16(4000 * a2 + 5000)
        self.vel_pub.publish(msg)

    # publish 2 main joystick axes for arm base movement (mode must be arm)
    def publish_arm_base_movement(self, armLength, armHeight, angle, cartesian=False, velocity=False):
        """
        Publish base arm position
        """
        msg = ArmMotion()
        msg.theta = angle * 0.01
        msg.cartesian = cartesian
        msg.on = True
        msg.velocity = velocity
        if velocity:
            msg.y = armLength*0.01  # y axis on joystick moves the target point forward and back
            msg.x = armHeight*0.01  # rotation of the joystick moves the target point up and down
        else:
            msg.y = armLength*1.5  # y axis on joystick moves the target point forward and back
            msg.x = armHeight*1.5  # rotation of the joystick moves the target point up and down
        self.arm_movement_pub.publish(msg)

    def publish_yolo_arm(self, joint_index, value):
        msg = ArmAngles()
        if joint_index is 0:
            msg.shoulderOrientation = value
        elif joint_index is 1:
            msg.shoulderElevation = value
        elif joint_index is 2:
            msg.elbow = value
        elif joint_index is 3:
            msg.wristElevation = value
        self.yolo_arm.publish(msg)


    # publish end effector position
    def publish_endEffector(self, x, y, rotate, grip):
        msg = EndEffector()
        msg.x = x
        msg.y = y
        msg.theta = rotate
        msg.grip = grip  # grip is either 1,0, or -1.
        self.arm_endEffector_pub.publish(msg)

    # publish camera zoom from axis 4
    def publish_camera(self, a3, a4):
        msg = PanTiltZoom()
        msg.pan = a3
        msg.tilt = a4
        self.cam_pub.publish(msg)

    # publish steering mode
    def setSteerMode(self, type):
        motionType = MotionType()

        motionType.SWERVE = 0
        if type == 0:
            motionType.ACKERMANN = 0
            motionType.TRANSLATORY = 0
            motionType.POINT = 1
            motionType.SKID = 0

        elif type == 1:
            motionType.ACKERMANN = 1
            motionType.TRANSLATORY = 0
            motionType.POINT = 0
            motionType.SKID = 0

        elif type == 2:
            motionType.ACKERMANN = 0
            motionType.TRANSLATORY = 1
            motionType.POINT = 0
            motionType.SKID = 0

        elif type == 3:
            motionType.ACKERMANN = 0
            motionType.TRANSLATORY = 0
            motionType.POINT = 0
            motionType.SKID = 1

        self.motionTypePublisher.publish(motionType)
