import math
# amrv: middle wheel rotation velocity.
# afsa: actural front wheel steering angle.
# rho:  radius of the rover around ICR
# sfsa: starboard front wheel steering angle
# pfsa: port front wheel steering angle
# pmsa: starboard middle wheel steering angle 
# smsa: port middle wheel steering angle
# srsa: starboard front wheel steering angle
# prsa: port front wheel steering angle
# w_s_a: = [pfsa,sfsa,pmsa,smsa,prsa,srsa,afsa]; 

D = 50e-2;  # distance between wheels of: front and middle/middle and rear[cm]   
B = 40e-2;  # distance between longitudinal axis and port/startboard wheels[cm]
R = 16.5e-2; # wheel radius [cm]
W = 15e-2; # wheel width [cm]

zero = 1e-10

#this function maps joystick input to steering angle of 6 wheels
def steer(vBody,wBody):
	amrv = (1/R)*vBody
	if abs(amrv) < zero:
		afsa = math.pi/2 #rho is zero, so steering angle is 90 degrees to
		rho = 0
	else:
		afsa = D*math.atan(wBody/amrv)
		rho = D*math.cot(afsa)

	sfsa = math.atan(D/(rho+B))
	pfsa = math.atan(D/(rho-B))
	pmsa = zeros
	smsa = zeros


	return rho

print steer(0,1)