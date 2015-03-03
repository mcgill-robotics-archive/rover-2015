from math import sqrt, atan, pi

#a1 is the length of the upper arm (touches base)
a1 = 1
#a2 is the length of the forearm (attached to hand)
a2 = 1

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
	if initial > 0:
		if y > 0: #correct quadrant (1)
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
	#else, compute normally
	#the following are the equations for each angle, computed by mathematica
	"""
	{{ArcTan(x*(a1^2 - a2^2 + x^2 + y^2) - (y*sqrt(-a1^4 - (-a2^2 + x^2 + y^2)^2 + 2*a1^2*(a2^2 + x^2 + y^2)))/Sign(y), 
	y*(a1^2 - a2^2 + x^2 + y^2) + (x*sqrt(-a1^4 - (-a2^2 + x^2 + y^2)^2 + 2*a1^2*(a2^2 + x^2 + y^2)))/Sign(y)), 
	-ArcTan(-a1^2 - a2^2 + x^2 + y^2, -(sqrt(-a1^4 - (-a2^2 + x^2 + y^2)^2 + 2*a1^2*(a2^2 + x^2 + y^2))/Sign(y)))}, 
	{ArcTan(x*(a1^2 - a2^2 + x^2 + y^2) + (y*sqrt(-a1^4 - (-a2^2 + x^2 + y^2)^2 + 2*a1^2*(a2^2 + x^2 + y^2)))/Sign(y), 
	y*(a1^2 - a2^2 + x^2 + y^2) - (x*sqrt(-a1^4 - (-a2^2 + x^2 + y^2)^2 + 2*a1^2*(a2^2 + x^2 + y^2)))/Sign(y)), 
	-ArcTan(-a1^2 - a2^2 + x^2 + y^2, sqrt(-a1^4 - (-a2^2 + x^2 + y^2)^2 + 2*a1^2*(a2^2 + x^2 + y^2))/Sign(y))}}
	"""
	#reducing reduntant calculation
	preCalculated1 = a1**2-a2**2+x**2+y**2
	preCalculated2 = sqrt(-a1**4-(-a2**2+x**2+y**2)**2+2*a1**2*(a2**2+x**2+y**2))
	#real nasty equations to get angles
	angles[0][0] = 180/pi * ArcTan(
		x*preCalculated1 - y*preCalculated2/sgn(y),
		y*preCalculated1 + x*preCalculated2/sgn(y))

	angles[0][1] = 180/pi * -ArcTan(-2*a1**2+preCalculated1,-preCalculated2/sgn(y))

	#remaining angles are a reflection in the direction vector to the point
	angles[1][0] = 2*ArcTan(x,y)-angles[0][0]
	angles[1][1] = -angles[0][1]
	return angles

print possibleAngles(sqrt(2),sqrt(2)-0.1)
