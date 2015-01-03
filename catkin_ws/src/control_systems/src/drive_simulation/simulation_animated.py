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

def midPoint(wheelProperties):
	positions = [float(a[0]) for a in [re.findall("\d+.\d+", z) for z in \
				sum([y for y in [x.split(' ') for x in \
				str(wheelProperties).split('[')[2:]]],[])\
				if len(z) ] if len(a)]
	corners = [(positions[x],positions[x+1]) for x in range(0,10,2)]
	midX = sum([x[0] for x in corners])/float(len(corners))
	midY = sum([y[1] for y in corners])/float(len(corners))
	return (midX,midY)


def init():
	ax.add_patch(body)
	ax.add_patch(FL)
	ax.add_patch(FR)
	ax.add_patch(RL)
	ax.add_patch(RR)
	body.set_visible(False)
	FL.set_visible(False)
	FR.set_visible(False)
	RL.set_visible(False)
	RR.set_visible(False)
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
	body.set_transform(t)
	FL.set_transform(mpl.transforms.Affine2D().rotate_around(x=0,y=0,theta=np.radians(i))+ax.transData)#+\
		#mpl.transforms.Affine2D().rotate_around(x=FL.get_x()+W/2,y=FL.get_y()+R,theta=np.radians(i)))
	FR.set_transform(t)
	RL.set_transform(t)
	RR.set_transform(t)


	print midPoint(FL.properties()['verts'])

	#FL.set_transform(mpl.transforms.Affine2D().rotate_around(x=FL.get_x()+W/2,y=FL.get_y()+R,theta=np.radians(i))+ax.transData)

	return body,FL,FR,RL,RR,

anim = animation.FuncAnimation(fig,animate,
	init_func=init,
	frames=360,
	interval=20,
	blit=True)

plt.show()

