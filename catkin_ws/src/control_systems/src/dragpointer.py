#!/usr/bin/python

import pygame
from pygame.locals import *

def main():
	#init screen
	pygame.init()
	screen = pygame.display.set_mode((500,500))
	pygame.display.set_caption('Arm Control')

	#fill background
	background = pygame.Surface(screen.get_size())
	background = background.convert()
	background.fill((250, 250, 250))

	# Blit everything to the screen
	screen.blit(background, (0, 0))
	pygame.display.flip()

	# Event loop
	while 1:
		for event in pygame.event.get():
			if event.type == QUIT:
				return
		screen.blit(background, (0, 0))
		pygame.display.flip()
		if pygame.mouse.get_pressed()[0]:
			print pygame.mouse.get_pos()


if __name__ == '__main__': 
	main()