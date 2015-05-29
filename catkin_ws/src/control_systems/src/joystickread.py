#!/usr/bin/env python
#  convert linear,angular speed to wheel settings
import rospy  # for reading and publishing to topics
from mappingsteer import steer, pointTurn, translationalMotion, swerve, max_mag, skid_steer
from geometry_msgs.msg import Twist  # type of joystick input
# type of wheel setting output
from control_systems.msg import SetPoints, Moving, MotionType
from std_msgs.msg import Int8, Float32, Bool, String
from time import clock

zero = 1e-10


class DualJoystickReader(object):
    def __init__(self):
        self.value = [0, 0]  # Default settings so vehicle should not move
        self.altValue = [0, 0]  # Values from the second joystick
        self.settings = SetPoints()  # Type of output
        self.motion = MotionType()
        self.moving = Moving()
        self.rotation = 0
        #Dependent on is_moving topic
        self.isMoving = False

        # Serve motion will consist of:
        # A variable to record current orientation of the rover with respect to
        self.swerving = Bool()
        self.swerving.data = False

        # this variable will allow us to calculate the orientation of the
        # rover, as we can measure the time taken from the last changed
        # settings
        self.clock = Float32()
        self.clock.data = clock()
        # the locked one
        self.swerve = Float32()
        self.swerve.data = 0  # start at a 0 rad heading from start
        # note that this is only used for swerve - it resets upon
        # resetting swerve

        # self.motion.data = 0
        rospy.init_node('joystick_reader')  # Name of this node

        # Open publisher - whih publishes to wheel
        self.pubwheels = rospy.Publisher('/wheels', SetPoints, queue_size=10,
                                         latch=True)

        # publisher for tests - gives rotation
        self.pubrotation = rospy.Publisher('/rotation', Float32, queue_size=10,
                                           latch=True)

        # whether or not wheels should be moving
        self.pubmovement = rospy.Publisher('/movement', Moving, queue_size=10,
                                           latch=True)

        # Subscribe to the topic "/cmd_vel", and print out output to function
        rospy.Subscriber('/cmd_vel', Twist, self.update_value_settings,
                         queue_size=10)
        # type of motion
        rospy.Subscriber('/cmd_motion', MotionType, self.update_value_motion,
                         queue_size=10)

        # Secondary joystick
        rospy.Subscriber('/cmd_alt_vel', Twist, self.update_second_joystick,
                         queue_size=10)

        #Boolean speed control
        rospy.Subscriber('/is_moving', Bool, self.update_is_moving,
                         queue_size=10)

    #Whether or not rover is moving
    def update_is_moving(self,msg):
        if msg.data:
            self.isMoving = True
        else:
            self.isMoving = False

    # update_settings depending on reading from topic
    def update_value_settings(self, msg):
        # read in values from twist
        self.value[0] = msg.linear.x
        self.value[1] = msg.angular.z

        # print self.value,self.motion.SWERVE
        # if not swerving, turn off swerving bool
        if self.swerving.data and not self.motion.SWERVE:
            self.swerving.data = False

        # translational motion
        if self.motion.TRANSLATORY:
            output = translationalMotion(self.value[0], self.value[1])
        # point steering (around middle)
        elif self.motion.POINT:
            output = pointTurn(self.value[1])
            time_passed = clock() - self.clock.data
            self.clock.data = clock()
            self.rotation += self.value[1] * time_passed
        # swerve drive :)
        elif self.motion.SWERVE:
            # value from other joystick is the spin
            spin = self.altValue[1]
            # if starting to swerve
            if self.swerving.data == False:
                self.swerve.data = 0
                self.swerving.data = True
                self.rotation = 0
                # do point steering to start off, which will trigger
                # wheels to turn in the right direction
                output = pointTurn(spin)

                # ###############################################
                # This may need to be moved in future versions
                self.clock.data = clock()
            else:
                # find the time passed since the last cycle -
                # we still have these settings stored so we can predict
                # the current position, etc.
                # use this with the self.settings to find the change
                # in theta - find tangential velocity at each wheel
                # (I suppose this could later be done with encoder readings)
                # once the change in angle is discovered, it would be added to
                # swerve, and then this could be used with the desired direction
                # to add to the swerve
                heading = 0

                # straight or back
                if abs(self.value[1]) < zero:
                    if self.value[0] < 0:
                        heading = math.pi
                else:
                    heading = self.value[0] / self.value[1]
                # get output settings, and new rotation of the
                # rover
                # testing code:
                # print "\n\nSettings (for below):\n\n",\
                # 	self.moving.move,self.settings.thetaFR\
                # 	,self.settings.thetaFL,\
                # 	self.settings.thetaRR,\
                # 	self.settings.thetaRL,\
                # 	self.settings.speedFL,\
                # 	self.settings.speedFR,\
                # 	self.settings.speedML,\
                # 	self.settings.speedMR,\
                # 	self.settings.speedRL,\
                # 	self.settings.speedRR,\
                # 	"time,spin,vbody,heading,rotation",\
                # 	time_passed,spin,\
                # 	maxMag(self.altValue),\
                # 	heading,self.rotation
                time_passed = clock() - self.clock.data
                self.clock.data = clock()
                (output, self.rotation) = swerve(self.settings, time_passed, spin,
                                                 max_mag(self.altValue), heading, self.rotation)

        elif self.motion.SKID:
            output = skid_steer(self.value[0],self.value[1])
            #[a-z]ckermann steering
        else:
            output = steer(self.value[0], self.value[1])
            # Find new rotation of the rover
            time_passed = clock() - self.clock.data
            self.clock.data = clock()
            self.rotation += self.value[1] * time_passed

        # Convert output of function to setpoint variable type
        self.moving.move = output['movement']
        self.settings.thetaFL = output['pfsa']
        self.settings.thetaFR = output['sfsa']
        self.settings.thetaRL = output['prsa']
        self.settings.thetaRR = output['srsa']
        self.settings.speedFL = output['pfrv']
        self.settings.speedFR = output['sfrv']
        self.settings.speedML = output['pmrv']
        self.settings.speedMR = output['smrv']
        self.settings.speedRL = output['prrv']
        self.settings.speedRR = output['srrv']

    def update_value_motion(self, msg):
        # read in values from Int8
        self.motion = msg

    def update_second_joystick(self, msg):
        # read in values from twist
        self.altValue[0] = msg.linear.x
        self.altValue[1] = msg.angular.z

    # function publishes
    def run(self):
        # calculate required wheel angles, speeds
        r = rospy.Rate(10)
        # continue endlessly
        while not rospy.is_shutdown():
            # log wheel settings
            rotOut = Float32()
            rotOut.data = self.rotation

            #Set all speeds to zero if speed set off
            if self.isMoving:
                self.settings.speedFL = 0
                self.settings.speedFR = 0
                self.settings.speedML = 0
                self.settings.speedMR = 0
                self.settings.speedRL = 0
                self.settings.speedRR = 0

            verbose = rospy.get_param("~verbose", False)
            if verbose:
                rospy.loginfo(rotOut)
                rospy.loginfo(self.moving)
                rospy.loginfo(self.settings)

            # publish it
            # tests:
            self.pubrotation.publish(rotOut)
            # non-tests:
            self.pubwheels.publish(self.settings)
            self.pubmovement.publish(self.moving)
            # 10 Hz rate regardless of joystick rate
            r.sleep()


if __name__ == '__main__':
    print "Initializing Node"
    joystickreader1 = DualJoystickReader()
    print "Running Node"
    joystickreader1.run()
    rospy.spin()
    # try:
    # joystickTranslator = Node()
    # also try to read from joystick
    # except rospy.ROSInterruptException: pass
