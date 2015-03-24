import numpy as np
import matplotlib.pyplot as plt 
from matplotlib import animation
import matplotlib as mpl
import re

D = 50e-2  # distance between wheels of: front and middle/middle and rear[m]   
B = 40e-2  # distance between longitudinal axis and port/startboard wheels[m]
R = 16.5e-2 # wheel radius [m]
W = 15e-2 # wheel width [m]

fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7,6.5)

ax = plt.axes(xlim=(-2,2),ylim=(-2,2))
#The main body of the rover
body = plt.Rectangle((-B,-D),width=2*B,height=2*D,angle=0.0,fc='y')
#The wheels
FL   = plt.Rectangle((-B-W/2, D-R),width=W,height=2*R,angle=0.0,fc='b')
FR   = plt.Rectangle(( B-W/2, D-R),width=W,height=2*R,angle=0.0,fc='b')
RL   = plt.Rectangle((-B-W/2,-D-R),width=W,height=2*R,angle=0.0,fc='b')
RR   = plt.Rectangle(( B-W/2,-D-R),width=W,height=2*R,angle=0.0,fc='b')

topright = plt.Rectangle((2,2),width=0,height=0,angle=0.0,fc='w')

def midPoint(window_extent):
	text = str(window_extent)
	values = ''.join([c for c in text if c in '0123456789.,']).split(',')
	numeric = [float(x) for x in values]
	return ((numeric[0]+numeric[2])/2,(numeric[1]+numeric[3])/2)


def init():
	ax.add_patch(body)
	ax.add_patch(FL)
	ax.add_patch(FR)
	ax.add_patch(RL)
	ax.add_patch(RR)
	ax.add_patch(topright)
	body.set_visible(False)
	FL.set_visible(False)
	FR.set_visible(False)
	RL.set_visible(False)
	RR.set_visible(False)
	topright.set_visible(True)
	#ax.add_patch(clearPatch)
	#t = mpl.transforms.Affine2D().
	return body,FL,FR,RL,RR,

def animate(i):
	#First frame always freezes, this fixes
	if i > 1:
		body.set_visible(True)
		FL.set_visible(True)
		FR.set_visible(True)
		RL.set_visible(True)
		RR.set_visible(True)
	t = mpl.transforms.Affine2D().rotate(theta=np.radians(i))+ax.transData
	#t2 = mpl.transforms.Affine2D().rotate(theta=np.radians(i))+ax.transData+mpl.transforms.Affine2D().rotate_around(x=FL.get_x()+W/2,y=FL.get_y()+R,theta=np.radians(i))
	#body.set_transform(t)
	#+\
		#mpl.transforms.Affine2D().rotate_around(x=FL.get_x()+W/2,y=FL.get_y()+R,theta=np.radians(i)))
	FL.set_transform(mpl.transforms.Affine2D().rotate(theta=np.radians(i)) + \
		mpl.transforms.Affine2D().rotate_around(x=FL.get_x()+W/2,y=FL.get_y()+R,theta=np.radians(i))+ax.transData)
	#FR.set_transform(t)
	#RL.set_transform(t)
	#RR.set_transform(t)

	#topCorner = midPoint(topright.properties()['verts'])
	#FLpoint = midPoint(FL.properties()['verts'])
	#FLx = FLpoint[0]/topCorner[0]
	#FLy = FLpoint[1]/topCorner[1]
	#print (FLx,FLy)
	#FL.set_transform(mpl.transforms.Affine2D().rotate_around(x=FL.get_x(),y=FL.get_y(),theta=np.radians(i))+ax.transData)
	#FL.set_transform(mpl.transforms.Affine2D().rotate_around(x=FLx,y=FLy,theta=np.radians(i))+ax.transData)
	if i%10==0:
		mid= midPoint(FL.properties()['window_extent'])
		topScreen= midPoint(topright.properties()['window_extent'])
		print (4*mid[0]/topScreen[0]-2, 4*mid[0]/topScreen[0]-2), topScreen


	#FL.set_transform(mpl.transforms.Affine2D().rotate_around(x=FL.get_x()+W/2,y=FL.get_y()+R,theta=np.radians(i))+ax.transData)

	return body,FL,FR,RL,RR,

anim = animation.FuncAnimation(fig,animate,
	init_func=init,
	frames=360,
	interval=20,
	blit=True)

plt.show()

