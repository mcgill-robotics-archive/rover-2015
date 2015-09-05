import rospy

from geometry_msgs.msg import Twist
from control_systems.msg import MotionType, ArmMotion, EndEffector
from std_msgs.msg import Bool, Int16
from rover_msgs.msg import PanTiltZoom, MotorControllerMode, JointSpeedArm, ValueControl, ArmModeControl

max_speed = 2


class Publisher(object):
    def __init__(self):
        # TODO change names once control systems has a defined topic name and variable names, copied from AUV as of now
        self.cam_pub = rospy.Publisher("/camera_motion",
                                       Twist, queue_size=10)
        self.vel_pub = rospy.Publisher(rospy.get_param("cmd_vel_topic", "cmd_vel"), Twist, queue_size=10)
        self.arm_movement_pub = rospy.Publisher(rospy.get_param("electrical_interface/arm_topic", "/cmd_arm"),
                                                ArmMotion, queue_size=10)
        self.arm_endEffector_pub = rospy.Publisher(rospy.get_param("cmd_endEffector_topic", "cmd_endEffector"),
                                                   EndEffector, queue_size=10)
        self.motionTypePublisher = rospy.Publisher(rospy.get_param("cmd_motion_topic", "cmd_motion"),
                                                   MotionType, queue_size=10)
        self.moving_bool_pub = rospy.Publisher("is_moving", Bool, queue_size=10)
        self.mode_publisher = rospy.Publisher("mc_mode", MotorControllerMode, queue_size=10)
        self.joint_vel_publisher = rospy.Publisher("arm_joint_speed", JointSpeedArm, queue_size=10)
        self.arm_mode_pub = rospy.Publisher("arm_mode", ArmModeControl, queue_size=10)
        self.science_angle = rospy.Publisher("science_angle", Int16, queue_size=10)

    def publish_science(self, angle):
        message = Int16()
        message.data = angle
        self.science_angle.publish(angle)

    def publish_arm_mode(self, mode):
        message = ArmModeControl()
        if mode == 0:
            message.PositionControl = True
            message.VelocityControl = False
        elif mode == 1:
            message.PositionControl = False
            message.VelocityControl = True

        self.arm_mode_pub.publish(message)

    # publisher for velocity
    def publish_velocity(self, angular, linear, on):
        """
        Publish linear and angular command velocity in twist for control systems
        Input should be between -1 and 1 where 1  is max speed forward, -1 is max back and 0 is immobile
        """
        msg = Twist()
        msg.linear.x = linear * max_speed
        msg.angular.z = angular
        moving_bool = Bool()
        moving_bool.data = on

        self.vel_pub.publish(msg)
        self.moving_bool_pub.publish(moving_bool)

    def publish_mode(self, msg):
        self.mode_publisher.publish(msg)

    # publish 2 main joystick axes for arm base movement (mode must be arm)
    def publish_arm_base_movement(self, armLength, armHeight, angle, cartesian=False, velocity=False):
        """
        Publish base arm position
        """
        msg = ArmMotion()
        msg.theta = angle
        msg.cartesian = cartesian
        msg.on = True
        msg.velocity = velocity

        msg.y = armLength*0.01  # y axis on joystick moves the target point forward and back
        msg.x = armHeight*0.01  # rotation of the joystick moves the target point up and down

        self.arm_movement_pub.publish(msg)

    def publish_arm_joint_velocity(self, joystick, motor_dict):
        message = JointSpeedArm()
        value_active = ValueControl()
        value_active.Enable = True
        value_active.Value = joystick

        value_disable = ValueControl()
        value_active.Enable = False

        message.shoulder = value_disable
        message.elbow = value_disable
        message.wrist = value_disable
        message.roll = value_disable
        message.grip = value_disable
        message.base = value_disable

        try:
            if motor_dict["shoulder"].isChecked():
                message.shoulder = value_active
                message.shoulder.Enable = True
            elif motor_dict["elbow"].isChecked():
                message.elbow = value_active
                message.elbow.Enable = True
            elif motor_dict["wrist"].isChecked():
                message.wrist = value_active
                message.wrist.Enable = True
            elif motor_dict["roll"].isChecked():
                message.roll = value_active
                message.roll.Enable = True
            elif motor_dict["grip"].isChecked():
                message.grip = value_active
                message.grip.Enable = True
            elif motor_dict["base"].isChecked():
                message.base = value_active
                message.base.Enable = True

        except KeyError:
            rospy.logwarn("Invalid dictionary received")
            return

        self.joint_vel_publisher.publish(message)
        pass
    
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
        msg = Twist()
        msg.angular.z = a3  # pan
        msg.angular.y = a4  # tilt
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

