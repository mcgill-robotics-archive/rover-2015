import numpy as np
import matplotlib.pyplot as plt 
from matplotlib import animation
import matplotlib as mpl

D = 50e-2  # distance between wheels of: front and middle/middle and rear[m]   
B = 40e-2  # distance between longitudinal axis and port/startboard wheels[m]

fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7,6.5)

ax = plt.axes(xlim=(-2,2),ylim=(-2,2))
#The main body of the rover
patch=plt.Rectangle((-B,-D),width=2*B,height=2*D,angle=0.0,fc='y')

def init():
	ax.add_patch(patch)
	patch.set_visible(False)
	#ax.add_patch(clearPatch)
	#t = mpl.transforms.Affine2D().
	return patch,

def animate(i):
	#First frame always freezes, this fixes
	if i > 1:
		patch.set_visible(True)
	t = mpl.transforms.Affine2D().rotate(np.radians(i))+ax.transData
	patch.set_transform(t)
	return patch,

anim = animation.FuncAnimation(fig,animate,
	init_func=init,
	frames=360,
	interval=20,
	blit=True)

plt.show()

