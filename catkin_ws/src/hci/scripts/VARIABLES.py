from PyQt4 import QtCore

class vel_vars(object):
    joystick_1_x = 0
    joystick_2_x = 0
    joystick_1_y = 0
    joystick_2_y = 0
    tester = ""

class KeyMapping(object):
    test = QtCore.Qt.Key_J
##     = QtCore.Qt.Key_L
##     = QtCore.Qt.Key_R
##     = QtCore.Qt.Key_W
##     = QtCore.Qt.Key_F
##     = QtCore.Qt.Key_S
##     = QtCore.Qt.Key_E
##     = QtCore.Qt.Key_D
##     = QtCore.Qt.Key_H

    
class misc_vars(object):
    GUI_UPDATE_PERIOD = 10
    controller_updateFrequency = 100
