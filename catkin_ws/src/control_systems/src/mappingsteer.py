#!/usr/bin/env python
import math #for trig functions
# amrv: middle wheel rotation velocity.
# afsa: actural front wheel steering angle.
# rho:  radius of the rover around ICR
# sfsa: starboard front wheel steering angle
# pfsa: port front wheel steering angle
# pmsa: starboard middle wheel steering angle 
# smsa: port middle wheel steering angle
# srsa: starboard rear wheel steering angle
# prsa: port rear wheel steering angle
# w_s_a: = [pfsa,sfsa,pmsa,smsa,prsa,srsa,afsa]; 

D = 50e-2  # distance between wheels of: front and middle/middle and rear[cm]   
B = 40e-2  # distance between longitudinal axis and port/startboard wheels[cm]
R = 16.5e-2 # wheel radius [cm]
W = 15e-2 # wheel width [cm]
T = math.pi/4 # max angle of front and rear wheels [rad]

zero = 1e-10 # Offers protection against numbers very close to zero

#the minimum distance of the ICR that the wheels can accomodate
rhoMin = D*math.tan(math.pi/2-T)+B

#also incorporate point steering!

def sign(n): return float(n)/abs(float(n))

#this function maps joystick input to steering angle of 6 wheels
#wBody is only always true in magnitude 
#(except for when wheels cannot accomodate)
def steer(vBody, wBody):
	#To be used in the final calculation of the velocity
	sgnv = sign(vBody)
	#angle of the wheels also depend on which way the rover is moving
	sgnw = sgnv*sign(wBody)
	#absolute values used in the calculations
	vBody,wBody = abs(float(vBody)),abs(float(wBody))
	#I could make an adjustment at start to force wbody positive, then change later
	
	#if robot not moving
	if abs(wBody) < zero and abs(vBody) < zero:
		#indicates that no settings should change, robot
		#should just stop movement
		#in the future we might want to put a button to
		movement = False
		pfsa = 0
		sfsa = 0
		pmsa = 0
		smsa = 0
		prsa = 0
		srsa = 0
		pfrv = 0
		sfrv = 0
		pmrv = 0
		smrv = 0
		prrv = 0
		srrv = 0
	elif abs(wBody) < zero: 
		#straight velocity
		#all wheels should be the same
		movement = True
		#angles are zero -> wheels point forward
		pfsa = 0
		sfsa = 0
		pmsa = 0
		smsa = 0
		prsa = 0
		srsa = 0
		#translate linear velocity to rotational velocity of wheel
		#along with the correct direction
		pfrv = vBody/R*sgnv
		sfrv = pfrv
		pmrv = pfrv
		smrv = pfrv
		prrv = pfrv
		srrv = pfrv

	###############
	#Might be better to implement a button to do this (or at least a delay), 
	#so the rover is not constantly trying to change to this state
	################
	
	#elif vBody/wBody <= B: #angle that the wheels cannot do, rover should do
	##a rotation on point
	#	movement = True
	#	pfsa = math.pi/2 - math.atan(B/D) #forms circle
	#	sfsa = -pfsa 
	#	pmsa = 0
	#	smsa = 0
	#	prsa = -pfsa
	#	srsa = pfsa
	#	r = math.sqrt(D**2+B**2)
	#	v = sgnw*wBody*r #linear velocity of each wheel
	#	pfrv = v/R #match angular velocity to rotation of wheel
	#	sfrv = -pfrv #should all move in circle
	#	pmrv = 0
	#	smrv = 0
	#	prrv = pfrv
	#	srrv = -pfrv
	

	else: #moving forward at an angle
		#impose a limit on this
		movement = True
		#get radius for circular motion
		rho = vBody/wBody

		#Make sure wheels can accomodate angle
		if rho < rhoMin:
			#if they cannot accomdate, do maximum angle
			rho = rhoMin
			#the angular velocity must be changed
			wBody = vBody/rho

		#Simple trig to get angle to each wheel
		pfsa = math.atan(D/(rho-B)) 
		
		sfsa = math.atan(D/(rho+B))

		#incorporate the correct direction of the angular
		#displacement of the wheels
		#multiplying this by the sign of the velocity makes the angular
		#velocity of the rover different than the input, but is of a more
		#natural movement 
		pfsa *= sgnw*sgnv
		sfsa *= sgnw*sgnv
		pmsa = 0
		smsa = 0
		prsa = -pfsa
		srsa = -sfsa

		#distance to front/rear wheels on each side of rover from ICR
		rp = math.sqrt((rho-B)**2+D**2)#distance to starboard side
		rs = math.sqrt((rho+B)**2+D**2)#radius a bit larger
		#the linear velocity of the front/rear wheels on each side
		vpLin = abs(wBody*rp)*sgnv
		vsLin = abs(wBody*rs)*sgnv

		#the individual velocities of each of the wheels
		pfrv = vpLin/R
		sfrv = vsLin/R
		#notice the middle wheels have different distance to ICR
		pmrv = sgnv*abs((rho-B))*wBody/R
		smrv = sgnv*abs((rho+B))*wBody/R
		prrv = pfrv
		srrv = sfrv

	#I split them up to stay within 80 columns 
	out={'movement':movement,'pfsa': pfsa,'sfsa': sfsa,'pmsa': pmsa}
	#add more values
	out.update({'smsa': smsa,'prsa': prsa,'srsa': srsa,'pfrv': pfrv})
	out.update({'sfrv': sfrv,'pmrv': pmrv,'smrv': smrv,'prrv': prrv})
	out.update({'srrv': srrv})
	return out

def pointTurnSteer(wBody):
	wBody = float(wBody)
	movement = True
	pfsa = math.pi/2 - math.atan(B/D) #forms circle
	sfsa = -pfsa 
	pmsa = 0
	smsa = 0
	prsa = -pfsa
	srsa = pfsa

	r = math.sqrt(D**2+B**2)
	v = wBody*r #linear velocity of each wheel

	pfrv = v/R #match angular velocity to rotation of wheel
	sfrv = -pfrv #should all move in circle
	pmrv = 0
	smrv = 0
	prrv = pfrv
	srrv = -pfrv	
	#I split them up to stay within 80 columns 
	out={'movement':movement,'pfsa': pfsa,'sfsa': sfsa,'pmsa': pmsa}
	#add more values
	out.update({'smsa': smsa,'prsa': prsa,'srsa': srsa,'pfrv': pfrv})
	out.update({'sfrv': sfrv,'pmrv': pmrv,'smrv': smrv,'prrv': prrv})
	out.update({'srrv': srrv})
	return out


#testing code and sample usage
"""
a= steer(-1,-1)
print a['pfrv']
print a['pfsa']
print a['prsa']
"""