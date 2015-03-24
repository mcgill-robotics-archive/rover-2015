#!/usr/bin/python
#For point steering, set up wheels even if zero speed
#####################################################
#####################################################
#####################################################
#####################################################
#####################################################
#####################################################
#####################################################
#####################################################
#####################################################
#####################################################
#####################################################
#####################################################
#####################################################
#####################################################

import math, rospy
from control_systems.msg import ArmMotion, ArmAngles
from std_msgs.msg import String

#length of first part of arm
a1 = rospy.get_param('control/ln_upperarm',1)
#length of second part of arm
a2 = rospy.get_param('control/ln_forearm',1)

############################################
#if lengths are same, gives div by zero 
############################################
############################################
############################################
############################################
############################################
############################################
############################################

#max is not truely the max, but forms a box for ease of comprehension
maxExtension = math.sqrt(a1**2+a2**2)

class ArmControlReader(object):
    def __init__ (self):
        #initiate node
        rospy.init_node('arm_reader')
        #publish arm settings to this topic
        self.pubArm = rospy.Publisher('/arm',ArmAngles,queue_size=10,latch=10)
        #settings to be read in
        self.settings = ArmMotion()
        #set values to off
        self.settings.x = 0
        self.settings.y = 0
        self.settings.theta = 0
        self.settings.on = False
        #angles:
        #angle at base
        self.angles = ArmAngles()
        # set all angles to zero
        self.angles.shoulderOrientation = 0
        self.angles.shoulderElevation = 0
        self.angles.elbow = 0
        self.angles.wristOrientation = 0
        self.angles.wristElevation = 0

        rospy.Subscriber('/cmd_arm', ArmMotion, self.update_settings,
            queue_size=10)

    def update_settings(self,msg):
        #import readings into object
        self.settings.x = msg.x
        self.settings.y = msg.y
        self.settings.theta = msg.theta
        self.settings.on = msg.on

        #calculate new angles for robotic arm
        #these are the angles for movement in the x-y plane
        (self.angles.shoulderElevation,
            self.angles.elbow) = getArmExtension(
            self.settings.x,self.settings.y,
            self.angles.shoulderElevation,
            self.angles.elbow)

        #this is the angle of the x-y plane relative to the forward direction
        #of the robot
        self.angles.shoulderOrientation = self.settings.theta

    #function will publish at 60Hz
    def run(self):
        r = rospy.Rate(60)
        #continue until quit
        while not rospy.is_shutdown():
            #publish to topic
            self.pubArm.publish(self.angles)
            #next iteration
            r.sleep()

#xi and yi are always between 0 and 1 (they are the fraction)
def getArmExtension (xi, yi, tht1_0, tht2_0):
    #since xi and yi are fractions, x and y will give the actual extension
    #in metres (so greater than the max extension is never entered
    x = xi*maxExtension
    y = yi*maxExtension
    if x == 0 and y == 0:
        return tht1_0, tht2_0

    c1_1 = (a1**2*x - y*((- a1**2 + 2*a1*a2 - a2**2 + x**2 + y**2)*(a1**2 + 2*a1*a2 + a2**2 - x**2 - y**2))**(1/2) - a2**2*x + x*y**2 + x**3)/(2*a1*(x**2 + y**2))
    c1_2 = (y*((- a1**2 + 2*a1*a2 - a2**2 + x**2 + y**2)*(a1**2 + 2*a1*a2 + a2**2 - x**2 - y**2))**(1/2) + a1**2*x - a2**2*x + x*y**2 + x**3)/(2*a1*(x**2 + y**2))

    s1_1 = (x*((- a1**2 + 2*a1*a2 - a2**2 + x**2 + y**2)*(a1**2 + 2*a1*a2 + a2**2 - x**2 - y**2))**(1/2) + a1**2*y - a2**2*y + x**2*y + y**3)/(2*a1*(x**2 + y**2))
    s1_2 = (a1**2*y - x*((- a1*2 + 2*a1*a2 - a2**2 + x**2 + y**2)*(a1**2 + 2*a1*a2 + a2**2 - x**2 - y**2))**(1/2) - a2**2*y + x**2*y + y**3)/(2*a1*(x**2 + y**2))

    #currently, division by zero is possible
    tht1_1 = math.atan(s1_1/c1_1)
    tht1_2 = math.atan(s1_2/c1_2)

    tht2_1 = math.atan((y-a1*math.sin(tht1_1))/(x-a1*math.cos(tht1_1)))-tht1_1
    tht2_2 = math.atan((y-a1*math.sin(tht1_2))/(x-a1*math.cos(tht1_2)))-tht1_2

    dev_1 = (tht1_1-tht1_0)**2+(tht2_1-tht2_0)**2
    dev_2 = (tht1_2-tht1_0)**2+(tht2_2-tht2_0)**2

    if dev_1 < dev_2:
        tht1 = tht1_1
        tht2 = tht2_1
    else:
        tht1 = tht1_2
        tht2 = tht2_2

    return tht1,tht2

if __name__ == '__main__':
    print "Initializing Node"
    reader1 = ArmControlReader()
    print "Running Node"
    reader1.run()
    rospy.spin()
