#!/usr/bin/python

import pygame
from pygame.locals import *

# a1 is the length of the upper arm (touches base)
a1 = rospy.get_param('control/ln_upperarm', 0.5)
# a2 is the length of the forearm (attached to hand)
a2 = rospy.get_param('control/ln_forearm', 0.5)

# bounds on forearm and upperarm angles
forMin = pi/18  # rospy.get_param('control/bound_lower_forearm',-30*pi/36)
forMax = 7*pi/18  # rospy.get_param('control/bound_upper_forearm',31*pi/36)
uppMin = pi/18  # rospy.get_param('control/bound_lower_upperarm',pi/18)
uppMax = 7*pi/18  # rospy.get_param('control/bound_upper_upperarm',8*pi/18)
rotMin = -pi  # rospy.get_param('control/bound_lower_orientation',-7*pi/8)
rotMax = pi # rospy.get_param('control/bound_upper_orientation',7*pi/8)
#wrist!
wriMin = -pi/2 
wriMax = pi/2

def main():
	maxX,maxY = 500,500
	msgx,msgy = 0,0
	x,y = 0,0
	#init screen
	pygame.init()
	screen = pygame.display.set_mode((maxX,maxY))
	pygame.display.set_caption('Arm Control')

	#fill background
	background = pygame.Surface(screen.get_size())
	background = background.convert()
	background.fill((250, 250, 250))

	# Blit everything to the screen
	screen.blit(background,(0, 0))
	pygame.display.flip()

	#Draw bounds by applying same check to pixels as with

	# Event loop
	while 1:
		for event in pygame.event.get():
			if event.type == QUIT:
				return
		screen.blit(background, (0, 0))
		pygame.display.flip()
		#update position of box
		if pygame.mouse.get_pressed()[0]:
			(msgx,msgy) = pygame.mouse.get_pos()
			(x,y) = (2*(float(msgx)-20.)/maxX,2.-4*msgy/float(maxY))
			print (x,y)
		background.fill((255,255,255,0))
		pygame.draw.rect(background,(0,0,0),
			pygame.Rect(msgx-5,msgy-5,10,10))
		#screen.blit(background,(0,0),special_flags=(pygame.BLEND_RGBA_ADD))

if __name__ == '__main__': 
	main()