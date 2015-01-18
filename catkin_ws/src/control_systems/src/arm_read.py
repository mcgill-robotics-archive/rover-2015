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

#length of first part of arm
a1 = rospy.get_param('control/ln_upperarm',1)
#length of second part of arm
a2 = rospy.get_param('control/ln_forearm',1.01)

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
	def __init__ (sefl):

		rospy.init_node('arm_reader')
		self.pubArm = rospy.Publisher('/arm',)

		self.x = 0
		self.y = 0
		self.theta = 0
		#angles:
		#angle at base
		self.shoulder = 0
		#angle in middle
		self.elbow = 0
		#angle for wrist of it
		self.wrist = 0

#xi and yi are always between 0 and 1 (they are the fraction)
def getArmExtension (xi, yi, tht1_0, tht2_0):
	x = xi*maxExtension
	y = yi*maxExtension
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

	return (tht1,tht2)

print getArmExtension(0.5,0.5,0,0)