#!/usr/bin/python

from math import sqrt, atan, pi, cos, sin
import rospy
from control_systems.msg import ArmMotion, ArmAngles
from std_msgs.msg import String

#a1 is the length of the upper arm (touches base)
a1 = rospy.get_param('control/ln_upperarm',0.6)
#a2 is the length of the forearm (attached to hand)
a2 = rospy.get_param('control/ln_forearm',0.6)

#bounds on forearm and upperarm angles
forearmLowerBound = rospy.get_param('control/bound_lower_forearm',-30*pi/36)
forearmUpperBound = rospy.get_param('control/bound_upper_forearm',31*pi/36)
upperarmLowerBound = rospy.get_param('control/bound_lower_upperarm',pi/18)
upperarmUpperBound = rospy.get_param('control/bound_upper_upperarm',8*pi/18)
orientationLowerBound = rospy.get_param('control/bound_lower_orientation',-7*pi/8)
orientationUpperBound = rospy.get_param('control/bound_upper_orientation',7*pi/8)

#max is not truely the max, but forms a box for ease of comprehension
maxExtension = sqrt(a1**2+a2**2)


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
		#self.settings.x = msg.x
		#self.settings.y = msg.y

		#bounds for x and y are not necessarily a rectangle,
		#so are hardcoded as functions of eachother
		#test if in bounds
		if msg.y >= 0 and msg.x >= 0 and distance(msg.x,msg.y)<=a1+a2:
			self.settings.x = msg.x
			self.settings.y = msg.y
		elif msg.y < 0 and msg.x >= 0 and msg.x <= a1+a2:
			self.settings.x = msg.x
			self.settings.y = 0
		elif msg.x < 0 and msg.y >= 0 and msg.y <= a1+a2:
			self.settings.x = 0
			self.settings.y = msg.y
		#both below bounds
		elif msg.x < 0 and msg.y < 0:
			(self.settings.x,self.settings.y)=(0,0)
		#out of bounds lengthwise
		elif distance(msg.x,msg.y) > a1+a2:
			#adjust for correct angle, but max boundary
			angle = ArcTan(msg.x,msg.y)
			#new settings are at boundary
			self.settings.x = (a1+a2)*cos(angle)
			self.settings.y = (a1+a2)*sin(angle)



		if msg.theta >= orientationLowerBound and\
			msg.theta <= orientationUpperBound:

			self.settings.theta = msg.theta
		self.settings.on = msg.on

		#calculate new angles for robotic arm
		#these are the angles for movement in the x-y plane
		newSetting = nextAngle(
			(self.angles.shoulderElevation,self.angles.elbow),
			self.settings.x,self.settings.y)


		##if new angle is good, update
		if newSetting[1]:
			self.angles.shoulderElevation = newSetting[0][0]
			self.angles.elbow = newSetting[0][1]
        
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
			rospy.loginfo(self.angles)
			#next iteration
			r.sleep()

#makes angle between -pi and pi
def modAngle(x):
	#if already good
	if x >= -pi and x <= pi:
		return x
	#if not, modulate x value
	return x - 2*pi * round(x/2./pi)


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
	angles[0][0] = ArcTan(
		x*preCalculated1 - y*preCalculated2/sgn(y),
		y*preCalculated1 + x*preCalculated2/sgn(y))

	angles[0][1] = -ArcTan(-2*a1**2+preCalculated1,-preCalculated2/sgn(y))

	#other set of angles are a reflection in the direction vector to the point
	angles[1][0] = ArcTan(
		x*(preCalculated1)+(y*preCalculated2)/sgn(y),
		y*(preCalculated1)-(x*preCalculated2)/sgn(y))
	#simple calculation for opposing angle on forearm :)
	angles[1][1] = -angles[0][1]
	return angles

#get next reasonable angles for the arm
#will return angles, and boolean to say
#whether arm should stop or not
def nextAngle(initialAngles, x, y):
	#quick check for safety
	if a1+a2 < distance(x,y):
		#point is unreachable;
		#don't calculate anything!
		return [[0,0],False]


	#get possible angles to move to
	(angleset1,angleset2) = possibleAngles(x,y)

	angleset1[0] = modAngle(angleset1[0])
	angleset1[1] = modAngle(angleset1[1])
	angleset2[0] = modAngle(angleset2[0])
	angleset2[1] = modAngle(angleset2[1])

	#both sets are not good until tested
	set1good,set2good = False,False

	#test if angles are in fact out of bounds
	if angleset1[0] >= upperarmLowerBound and\
		angleset1[0] <= upperarmUpperBound and\
		angleset1[1] >= forearmLowerBound and\
		angleset1[1] <= forearmUpperBound:

		#if all angles in set are fine, declare the set okay
		set1good = True

	if angleset2[0] >= upperarmLowerBound and\
		angleset2[0] <= upperarmUpperBound and\
		angleset2[1] >= forearmLowerBound and\
		angleset2[1] <= forearmUpperBound:

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
	deviation1 = abs(initialAngles[0]-angleset1[0])
	deviation1 += abs(initialAngles[1]-angleset1[1])
	deviation2 = abs(initialAngles[0]-angleset2[0])
	deviation2 += abs(initialAngles[1]-angleset2[1])

	#if first set is closer to current position, return it
	if deviation1 <= deviation2:
		return [angleset1,True]
	return [angleset2,True]

if __name__ == '__main__':
    print "Initializing Node"
    reader1 = ArmControlReader()
    print "Running Node"
    reader1.run()
    rospy.spin()