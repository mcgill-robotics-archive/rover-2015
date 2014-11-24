#!/usr/bin/env python
import math #for trig functions
# rho:  radius of the rover around ICR
# sfsa: starboard front wheel steering angle
# pfsa: port front wheel steering angle
# pmsa: starboard middle wheel steering angle 
# smsa: port middle wheel steering angle
# srsa: starboard rear wheel steering angle
# prsa: port rear wheel steering angle
# pfrv: port front wheel rotation velocity
# sfrv: starboard front wheel rotation velocity
# pmrv: port middle wheel rotation velocity
# smrv: starboard middle wheel rotation velocity
# prrv: port rear wheel rotation velocity
# srrv: starboard rear wheel rotation velocity
 

D = 50e-2  # distance between wheels of: front and middle/middle and rear[m]   
B = 40e-2  # distance between longitudinal axis and port/startboard wheels[m]
R = 16.5e-2 # wheel radius [m]
W = 15e-2 # wheel width [m]
T = math.pi/4 # max angle of front and rear wheels [rad]

zero = 1e-10 # Offers protection against numbers very close to zero

#the minimum distance of the IPCR that the wheels can accomodate
rhoMin = D*math.tan(math.pi/2-T)+B

#function returns the sign of a variable (1,0 or -1)
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

	#higher value of "zero" is so that when the user aims straight,
	#even if the joystick is a little bit off, the robot will go straight
	elif abs(wBody) < zero*1e7: 
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

#Function will do a point turn, so there is no velocity
#note that the wheels be turned at a different angle than above
########################
#also note this does not respect the max angle of the wheel
#######################
def pointTurn(wBody):
	wBody = float(wBody)
	movement = True
	#wheels have specific angle - all of them should form a circle together
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
#a= pointTurn(-1)
#print a['pfrv']
#print a['pfsa']
#print a['prsa']
