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
#This value needs to be tested for
T = 3*math.pi/8 # max angle of front and rear wheels [rad] (technically no max)

#angle on the wheels for point steering
pointSteeringAngle = math.pi/2 - math.atan(B/D)
#radius from the wheels to the middle of the rover for point steering
pointSteeringRadius = math.sqrt(D**2+B**2)
zero = 1e-10 # Offers protection against numbers very close to zero

#the minimum distance of the IPCR that the wheels can accomodate
rhoMin = D*math.tan(math.pi/2-T)+B

#function returns the sign of a variable (1,0 or -1)
def sign(n): 
	if n == 0:
		return 0.  
	else: 
		return float(n)/abs(float(n))

def stop():
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
	#I split them up to stay within 80 columns 
	out={'movement':movement,'pfsa': pfsa,'sfsa': sfsa,'pmsa': pmsa}
	#add more values
	out.update({'smsa': smsa,'prsa': prsa,'srsa': srsa,'pfrv': pfrv})
	out.update({'sfrv': sfrv,'pmrv': pmrv,'smrv': smrv,'prrv': prrv})
	out.update({'srrv': srrv})
	return out


#this function maps joystick input to steering angle of 6 wheels
#wBody is only always true in magnitude 
#(except for when wheels cannot accomodate)
def steer(vBody, wBody):
	#To be used in the final calculation of the velocity
	sgnv = sign(vBody)
	#angle of the wheels also depend on which way the rover is moving
	sgnw = sign(wBody)
	#absolute values used in the calculations
	vBody,wBody = abs(float(vBody)),abs(float(wBody))
	#I could make an adjustment at start to force wbody positive, then change later
	
	#if robot not moving
	if abs(vBody) < zero:
		#indicates that no settings should change, robot
		#should just stop movement
		#in the future we might want to put a button to
		return stop()

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

		#distance from ICR to side changes depending on side of ICR
		rp = rho+sgnw*B
		rs = rho-sgnw*B

		#Simple trig to get angle to each wheel
		pfsa = math.atan(D/(rp)) 
		sfsa = math.atan(D/(rs))

		#incorporate the correct direction of the angular
		#displacement of the wheels
		#multiplying this by the sign of the velocity makes the angular
		#velocity of the rover different than the input, but is of a more
		#natural movement 
		pfsa *= sgnw
		sfsa *= sgnw
		pmsa = 0
		smsa = 0
		prsa = -pfsa
		srsa = -sfsa

		#distance to front wheels on each side of rover from ICR
		rpf = math.sqrt(rp**2+D**2)#distance to port side front wheels
		rsf = math.sqrt(rs**2+D**2)#starboard side
		#the linear velocity of the front/rear wheels on each side
		vpLin = sgnv*wBody*rpf
		vsLin = sgnv*wBody*rsf

		#the individual velocities of each of the wheels
		pfrv = vpLin/R
		sfrv = vsLin/R
		#notice the middle wheels have different distance to ICR
		pmrv = sgnv*rp*wBody/R
		smrv = sgnv*rs*wBody/R
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
	if abs(wBody) < zero:
		#if no velocity, stop motion and keep angles
		return stop()
	wBody = float(wBody)
	movement = True
	#wheels have specific angle - all of them should form a circle together
	pfsa = pointSteeringAngle #forms circle
	sfsa = -pfsa 
	pmsa = 0
	smsa = 0
	prsa = -pfsa
	srsa = pfsa

	r = pointSteeringRadius
	v = wBody*r #linear velocity of each wheel

	pfrv = v/R #match angular velocity to rotation of wheel
	sfrv = -pfrv #should all move in circle
	pmrv = wBody*B/R #same angular velocity
	smrv = -pmrv
	prrv = pfrv
	srrv = -pfrv	
	#I split them up to stay within 80 columns 
	out={'movement':movement,'pfsa': pfsa,'sfsa': sfsa,'pmsa': pmsa}
	#add more values
	out.update({'smsa': smsa,'prsa': prsa,'srsa': srsa,'pfrv': pfrv})
	out.update({'sfrv': sfrv,'pmrv': pmrv,'smrv': smrv,'prrv': prrv})
	out.update({'srrv': srrv})
	return out

#translational motion will always make wheel move in forward direction
def translationalMotion(y,x):
	###########################
	#eventually this function should pick the angle based on the 
	#current and accumulated angles of the wheels
	###########################
	if abs(x) < zero and abs(y) < zero:
		#no velocity
		return stop()
	elif abs(y) < zero:
		#just forward/zero motion, so can use middle wheels as well
		return steer(y,0)
		#determines which side should get the diagonal

	sgnx = sign(x)
	#equivalent direction of wheels
	theta = math.pi/2 - math.atan(y/x)
	#now find actual direction of wheels, such that
	#the wheels will maintain a similar direction on each side, 
	#so that when moving sideways the wheels don't do a full
	#rotation whenever the joystick goes past 90 from forward
	if sgnx > 0 and theta < 0:#make theta positive
		theta = math.pi - theta
	elif sgnx <0 and theta > 0:#make theta negative
		theta = theta - math.pi
	movement = True
	pfsa = theta
	sfsa = theta
	pmsa = 0 #wheels cannot turn
	smsa = 0
	prsa = theta
	srsa = theta
	#translate linear velocity to rotational velocity of wheel
	#along with the correct direction

	#because ps3 joystick is being mapped to a square,
	#this maps it back to a circle (same speed around
	#edges of joystick)
	pfrv = max([abs(x),abs(y)])/R
	sfrv = pfrv
	pmrv = 0
	smrv = 0
	prrv = pfrv
	srrv = pfrv
	#I split them up to stay within 80 columns
	out={'movement':movement,'pfsa': pfsa,'sfsa': sfsa,'pmsa': pmsa}
	#add more values
	out.update({'smsa': smsa,'prsa': prsa,'srsa': srsa,'pfrv': pfrv})
	out.update({'sfrv': sfrv,'pmrv': pmrv,'smrv': smrv,'prrv': prrv})
	out.update({'srrv': srrv})
	return out

def swerve

#testing code and sample usage
#a= pointTurn(-1)
#print a['pfrv']
#print a['pfsa']
#print a['prsa']