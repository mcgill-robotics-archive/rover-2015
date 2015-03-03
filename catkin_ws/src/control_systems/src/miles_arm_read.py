import math

#a1 is the length of the upper arm (touches base)
a1 = 1
#a2 is the length of the forearm (attached to hand)
a2 = 1

#gets the euclidian distance to a point from the origin
def distance(x,y,z=0):
	return math.sqrt(x**2+y**2+z**2)

#arctangent function that adjusts based on
#what quadrant it should be in
def ArcTan(y, x):
	initial = math.atan(y/x)
	if initial > 0:
		if y > 0: #correct quadrant (1)
			return initial
		#incorrect (3)
		return initial + math.pi
	else:
		if y < 0: #correct quadrant (4) (adjusted for +ve angle)
			return initial + 2*math.pi
		#incorrect (2) 
		return initial + math.pi

#function will give two sets of angles for the robotic arm (both valid)
#, when told which point in the xy plane needs to be reached
def possibleAngles (x, y):
	if x+y 
