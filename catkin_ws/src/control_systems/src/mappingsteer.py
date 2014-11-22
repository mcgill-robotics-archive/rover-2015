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
	elif abs(wBody) < zero: #straight velocity
		#all wheels should be the same
		movement = True
		pfsa = 0
		sfsa = 0
		pmsa = 0
		smsa = 0
		prsa = 0
		srsa = 0
		pfrv = vBody/R
		sfrv = pfrv
		pmrv = pfrv
		smrv = pfrv
		prrv = pfrv
		srrv = pfrv

	###############
	#Might be better to implement a button to do this (or at least a delay), 
	#so the rover is not constantly trying to change to this state
	################
	elif abs(vBody) < zero: #nonzero angle, rover should do 
	#a rotation on point
		movement = True
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
		

	else: #moving forward at an angle
		#impose a limit on this
		movement = True
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
		pfrv = vpLin/R
		sfrv = vsLin/R
		pmrv = (rho-B)*wBody/R
		smrv = (rho+B)*wBody/R
		prrv = pfrv
		srrv = sfrv

	#I split them up to stay within 80 columns
	out={'movement':movement,'pfsa': pfsa,'sfsa': sfsa,'pmsa': pmsa}
	out.update({'smsa': smsa,'prsa': prsa,'srsa': srsa,'pfrv': pfrv})
	out.update({'sfrv': sfrv,'pmrv': pmrv,'smrv': smrv,'prrv': prrv})
	out.update({'srrv': srrv})
	return out

a= steer(1,-1)
print a['pfrv']
print a['pfsa']
print a['prsa']