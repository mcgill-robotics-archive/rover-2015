## @package PS3Controller
#
#  Main file for PS3 controller
#
#  @author David Lavoie-Boutin

import os
import sys
from VARIABLES import vel_vars

try:
    from pygame import locals
    import pygame
except ImportError:
    print "Cannot import the PyGame library \nNow terminating the process..."
    sys.exit()

os.environ["SDL_VIDEODRIVER"] = "dummy"

def test_left():
    vel_vars.tester = "left arrow"

def test_up():
    vel_vars.tester = "up arrow"

def test_right():
    vel_vars.tester = "right arrow"

def test_down():
    vel_vars.tester = "down arrow"

def test_joystick():
    vel_vars.tester = "(x1,y1) = (%s,%s)\n(x2,y2) = (%s,%s)" % (vel_vars.joystick_1_x,vel_vars.joystick_1_y,vel_vars.joystick_2_x,vel_vars.joystick_2_y)


    



##
#  Class containing all relevant commands to fetch data from a ps3 controller
class PS3Controller(object):
    ##
    #Constructor of the ps3Controller. 
    #It basically initialize the joystick so we can fetch data form it.
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        ##Default config as ps3 controller not existant
        #will be changed to true if the controller is initialized. 
        self.controller_isPresent=False
        self.thruster_stop = False
        self.fiel_thruste_1 = 0
        self.fiel_thruster_2 = 0
        self.fiel_thruster_3 = 0
        self.fiel_thruster_4 = 0
        self.fiel_thruster_5 = 0
        self.fiel_thruster_6 = 0
        
        if pygame.joystick.get_count() == 0:
            self.controller_isPresent = False
        elif pygame.joystick.get_count() == 1:
            self.controller_isPresent = True

            ##The initialized controller
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
        else:
            self.controller_isPresent = False

    ##
    #It reads the data from the controller
    # (the queue of event more precisely) and updates the Global data of the instance.
    #@param self the object pointer
    def updateController_for_controls_systems(self):
        """This method fetches the information form the ps3 controller parses the data and returns values ready to be
        published for the implementation with the controls systems, i.e. it should be used with the 'Sensor Information'
        tab. """
        # If a changed occurred, the value will be updated; else the value will be the last one fetched.

        if self.controller_isPresent:
            for anEvent in pygame.event.get():
                if anEvent.type == pygame.locals.JOYBUTTONDOWN:

                    if self.controller.get_button(7):  # left arrow
                        test_left()
                    elif self.controller.get_button(4):  # up arrow
                        test_up()
                    elif self.controller.get_button(5):  # right arrow
                        test_right()            
                    elif self.controller.get_button(6):  # down arrow
                        test_down()

                elif anEvent.type == pygame.locals.JOYAXISMOTION:
                    vel_vars.joystick_1_x = self.controller.get_axis(0)
                    vel_vars.joystick_2_x = self.controller.get_axis(2)
                    vel_vars.joystick_1_y = self.controller.get_axis(1)
                    vel_vars.joystick_2_y = self.controller.get_axis(3)
                    test_joystick()
                

