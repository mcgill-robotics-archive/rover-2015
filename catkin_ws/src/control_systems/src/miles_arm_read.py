#!/usr/bin/python

from math import sqrt, atan, pi
import rospy
from control_systems.msg import ArmMotion, ArmAngles
from std_msgs.msg import String

#a1 is the length of the upper arm (touches base)
a1 = rospy.get_param('control/ln_upperarm',1)
#a2 is the length of the forearm (attached to hand)
a2 = rospy.get_param('control/ln_forearm',1)

forearmLowerBound = pi/6
forearmUpperBound = 5*pi/4
upperarmLowerBound = -pi/2
upperarmUpperBound = pi/2


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
        #ADJUST BOUNDS HERE!!!
        self.settings.on = msg.on

        #calculate new angles for robotic arm
        #these are the angles for movement in the x-y plane
        newSetting = nextAngle(
            (self.angles.shoulderElevation,self.angles.elbow),
            self.settings.x,self.settings.y)

        #if new angle is good, update
        if newSetting[1]:
        	(self.angles.shoulderElevation,
        		self.angles.elbow) = newSetting[0]

        #this is the angle of the x-y plane relative to the forward direction
        #of the robot
        newOrientation = msg.theta
        if newOrientation <= pi/2 and newOrientation >= -pi/2:
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




def sgn(x):
	if x == 0:
		return 1
	return x/abs(x)

#gets the euclidian distance to a point from the origin
def distance(x,y,z=0):
	return sqrt(x**2+y**2+z**2)

#arctangent function that adjusts based on
#what quadrant it should be in

#function will not return an angle 
#greater in magnitude than pi
def ArcTan(x, y):
	if x == 0:
		if y > 0:
			return pi/2
		return 3*pi/2
	initial = atan(y/x)
	if initial >= 0:
		if y >= 0: #correct quadrant (1)
			return initial
		#incorrect (3)
		return initial - pi
	else:
		if y < 0: #correct quadrant (4)
			return initial
		#incorrect (2) 
		return initial + pi


#Some basic vector geometry functions:
def dot(u,v):
	total = 0
	for x,y in zip(u,v):
		total+=x*y
	return total

def project(u,v):
	mag = 0
	for x in v:
		mag += x**2
	scalar = dot(u,v)/mag
	result = []
	for x in v:
		result.append(x*scalar)
	return result

def reflect(u,v):
	projection = project(u,v)
	result = []
	for x,y in zip(u,projection):
		result.append(2*y - x)
	return result

#function will give two sets of angles for the robotic arm (both valid)
#, when told which point in the xy plane needs to be reached
def possibleAngles (x, y):
	#out of reach
	angles = [[0.,0.],[0.,0.]]
	if distance(x,y) > a1+a2:
		return angles
	#else, compute normally...


	#the following are the equations for each angle, computed by mathematica

	#variables to reduce reduntant calculation
	preCalculated1 = a1**2-a2**2+x**2+y**2
	preCalculated2 = sqrt(-a1**4-(-a2**2+x**2+y**2)**2+2*a1**2*(a2**2+x**2+y**2))
	#real nasty equations to get angles
	angles[0][0] = 180/pi * ArcTan(
		x*preCalculated1 - y*preCalculated2/sgn(y),
		y*preCalculated1 + x*preCalculated2/sgn(y))

	angles[0][1] = 180/pi * -ArcTan(-2*a1**2+preCalculated1,-preCalculated2/sgn(y))

	#other set of angles are a reflection in the direction vector to the point
	angles[1][0] = 180/pi * ArcTan(
		x*(preCalculated1)+(y*preCalculated2)/sgn(y),
		y*(preCalculated1)-(x*preCalculated2)/sgn(y))
	#simple calculation for opposing angle on forearm :)
	angles[1][1] = -angles[0][1]
	return angles

#get next reasonable angles for the arm
#will return angles, and boolean to say
#whether arm should stop or not
def nextAngle(initialAngles, x, y, psi):
	#quick check for safety
	if a1+a2 < distance(x,y):
		#point is unreachable;
		#don't calculate anything!
		return [[0,0],False]


	#get possible angles to move to
	(angleset1,angleset2) = possibleAngles(x,y)
	#both sets are not good until tested
	set1good,set2good = False,False

	#test if angles are in fact out of bounds
	for x in range(0,2):
		if angleset1[x][0] >= upperarmLowerBound and\
			angleset1[x][0] <= upperarmUpperBound and\
			angleset1[x][1] >= forearmLowerBound and\
			angleset1[x][1] <= forearmUpperBound:

			#if all angles in set are fine, declare the set okay
			if x = 0:
				set1good = True
			else:
				set2good = True

	#if both sets are out of bounds,
	if not set1good and not set2good:
		#return false for movement
		return [[0,0],False]
	#if only one is good, return that one
	elif set1good and not set2good:
		return [angleset1,True]
	elif set2good and not set1good:
		return [angleset2,True]

	#if both are good, then find best angleset for current position

	#get total angle deviation for each angle set
	deviation1 = abs(initialAngles[0][0]-angleset1[0])
	deviation1 += abs(initialAngles[0][1]-angleset1[1])
	deviation2 = abs(initialAngles[0][0]-angleset2[0])
	deviation2 += abs(initialAngles[1][1]-angleset2[1])

	#if first set is closer to current position, return it
	if deviation1 <= deviation2:
		return [angleset1,True]
	return [angleset2,True]

#example code
print nextAngle([[0,0],[0,0]],0,2)