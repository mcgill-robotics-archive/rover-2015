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

zero = 1e-10 # Offers protection against numbers very close to zero

#also incorporate point steering!

def sign(n): return float(n)/abs(float(n))

#this function maps joystick input to steering angle of 6 wheels
def steer(vBody, wBody):
	
	vBody,wBody = float(vBody),float(wBody)
	#I could make an adjustment at start to force wbody positive, then change later
	
	#if robot not moving
	if abs(wBody) < zero and abs(vBody) < zero:
		return [False,0,0,0,0,0,0,0,0,0,0,0,0]#first bool indicates no movement

	elif abs(wBody) < zero: #straight velocity
		#all wheels should be the same
		pfrv = vBody/R
		sfrv = pfrv
		pmrv = pfrv
		smrv = pfrv
		prrv = pfrv
		srrv = pfrv

		#no angle on wheels, #True for change movement
		return [True,0,0,0,0,0,0,pfrv,sfrv,pmrv,smrv,prrv,srrv]

	###############
	#Might be better to implement a button to do this (or at least a delay), 
	#so the rover is not constantly trying to change to this state
	################
	elif abs(vBody) < zero: #nonzero angle, rover should do 
	#a rotation on point
		pfsa = math.pi/2 - math.atan(B/D) #forms circle
		sfsa = -pfsa #45 deg to left
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
		return [True,pfsa,sfsa,pmsa,smsa,prsa,srsa,pfrv,sfrv,pmrv,smrv,prrv,srrv]

	else: #moving forward at an angle
		#impose a limit on this
		#get radius for circular motion
		rho = vBody/wBody
		#need to modulate this
		pfsa = (math.pi/2-math.atan(D/(rho-B)))%(math.pi)
		sfsa = (math.pi/2-math.atan(D/(rho+B)))%(math.pi)
		if pfsa > math.pi/2:
			pfsa = math.pi/2 - pfsa
		if sfsa > math.pi/2:
			sfsa = math.pi/2 - sfsa
		pmsa = 0
		smsa = 0
		prsa = -pfsa
		srsa = -sfsa
		rp = math.sqrt((rho-B)**2+D**2)#distance to starboard side
		rs = math.sqrt((rho+B)**2+D**2)#radius a bit larger

		vpLin = abs(wBody*rp) * sign(vBody)
		vsLin = abs(wBody*rs) * sign(vBody)
		print vpLin,vsLin, pfsa, prsa
		#print [True,pfsa,sfsa,pmsa,smsa,prsa,srsa]#,pfrv,sfrv,pmrv,smrv,prrv,srrv]



	"""

	
	smrv = (1/R)*vBody #starboard middle wheel rotation velocity
	
	#if not moving or no rotation, rho is zero
	if abs(smrv) < zero or abs(wBody) < zero:
		afsa = math.pi/2 #rho is zero, so steering angle is 90 degrees to
		rho = 0
		rotation = False
	else:
		afsa = D*math.atan(wBody/smrv)
		rho = D/math.tan(afsa)
		rotation = True
	if rotation:
		pmrv = smrv/(rho-B)*(rho+B) #port is a bit faster as it is on the far side
	else:
		pmrv = smrv
	if abs(rho+B) < zero:
		if (rho+B) > 0:#atan will tend to positive infinity
			sfsa = math.pi/2
		else: #negative infinity
 			sfsa = -math.pi/2
	else:
		sfsa = math.atan(D/(rho+B))
	if abs(rho-B) < zero:
		if (rho-B) > 0: #positive infinity
			pfsa = math.pi/2
		else:
			pfsa = -math.pi/2
	else:
 		pfsa = math.atan(D/(rho-B))
	srsa = -sfsa
	prsa = -pfsa
	#return theta(FL), FR, RL, RR, speedMW

	#rho-b is for port
	if not rotation:
		sfrv = smrv
		srrv = smrv
	else:
		sr = math.sqrt((rho+B)**2+D**2)
		sfrv = wBody*sr/R
		srrv = sfrv #for this type of motion, speed of wheel is same
	
	if not rotation:
		pfrv = pmrv
		prrv = pmrv
	else:

		#unsure if this would be right or left side
		#starboard, port radius about angular motion
		
		pr = math.sqrt((rho-B)**2+D**2)

		#starboard front and back rotation velocity 
		#CURRENTLY INNER WHEEL

		pfrv = wBody*pr/R
		prrv = pfrv

	#print first angles, then velocities
	#first front, then middle, then rear
	#first port, then starboard
	return [pfsa,sfsa,0,0,prsa,srsa,pfrv,sfrv,pmrv,smrv,prrv,srrv]
	"""

print steer(-1,-1)