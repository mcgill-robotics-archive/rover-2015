#!/usr/bin/python

import pygame
from pygame.locals import *


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
		screen.fill((250,250,250))
		pygame.draw.rect(background,(0,0,0),
			pygame.Rect(msgx-5,msgy-5,10,10))

if __name__ == '__main__': 
	main()