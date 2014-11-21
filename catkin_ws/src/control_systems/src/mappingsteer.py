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

#this function maps joystick input to steering angle of 6 wheels
def steer(vBody, wBody):
	vBody,wBody = float(vBody),float(wBody)

	amrv = (1/R)*vBody #middle wheel rotation velocity
	if abs(amrv) < zero or abs(wBody) < zero:
		afsa = math.pi/2 #rho is zero, so steering angle is 90 degrees to
		rho = 0
	else:
		afsa = D*math.atan(wBody/amrv)
		rho = D/math.tan(afsa)
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

	#unsure if this would be right or left side
	radiusright = sqrt((rho-B)^2+D^2)
	radiusleft = sqrt((rho+B)^2+D^2)
	
	return [pfsa,sfsa,prsa,srsa,amrv]

steer(1,1)