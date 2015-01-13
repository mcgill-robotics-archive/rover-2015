#!/usr/bin/python

from PyQt4 import QtCore, QtGui

from RoverWindow import * # main window declaration

#import Joystick class
from JoystickController import JoystickController

from VARIABLES import *

import sys
import rospy

class CentralUi(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(CentralUi,self).__init__(parent)

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.controller = JoystickController()


        self.controller_timer = QtCore.QTimer()
        self.thrust_pub_timer = QtCore.QTimer()
        
        self.set_controller_timer()
        print("Starting...")

        ##button connects

        # controller timer connect
        QtCore.QObject.connect(self.controller_timer, QtCore.SIGNAL("timeout()"), self.update_test)
        QtCore.QObject.connect(self.ui.DriveMode, QtCore.SIGNAL("clicked()"), self.setMode0)
        QtCore.QObject.connect(self.ui.ArmBaseMode, QtCore.SIGNAL("clicked()"), self.setMode1)
        QtCore.QObject.connect(self.ui.EndEffectorMode, QtCore.SIGNAL("clicked()"), self.setMode2)
        QtCore.QObject.connect(self.ui.function4, QtCore.SIGNAL("clicked()"), self.setMode3)


    def set_controller_timer(self):
        if self.controller is not None:
            self.controller_timer.start(misc_vars.controller_updateFrequency)
            rospy.loginfo("Started controller timer")
        else:
            rospy.loginfo("Missing controller, timer aborted")

    def update_test(self):
        self.controller.update()
        self.ui.MainX.setValue(self.controller.a1*1000)
        self.ui.MainY.setValue(-self.controller.a2*1000)
        self.ui.StickRotation.setValue(self.controller.a3*1000)
        self.ui.SecondaryY.setValue(-self.controller.a4*1000)

        if self.controller.b3:
            self.ui.Camera1Feed.setCurrentIndex(5)
        elif self.controller.b4:
            self.ui.Camera1Feed.setCurrentIndex(4)
        elif self.controller.b7:
            self.setMode(0)
        elif self.controller.b8:
            self.setMode(1)
        elif self.controller.b9:
            self.setMode(2)
        elif self.controller.b10:
            self.setMode(3)
        elif self.controller.hat ==(-1,0):
            self.ui.Camera1Feed.setCurrentIndex(1)
        elif self.controller.hat ==(0,1):
            self.ui.Camera1Feed.setCurrentIndex(0)
        elif self.controller.hat ==(0,-1):
            self.ui.Camera1Feed.setCurrentIndex(3)
        elif self.controller.hat ==(1,0):
            self.ui.Camera1Feed.setCurrentIndex(2)
        self.controller.hat = (0,0)
        self.controller.b3 = False
        self.controller.b4 = False
        self.controller.b7 = False
        self.controller.b8 = False
        self.controller.b9 = False
        self.controller.b10 = False

    def setMode0(self):
        self.setMode(0)


    def setMode1(self):
        self.setMode(1)


    def setMode2(self):
        self.setMode(2)


    def setMode3(self):
        self.setMode(3)


    def setMode(self, mode_id):
        if mode_id == 0:
            self.ui.DriveMode.setChecked(True)
            self.ui.ArmBaseMode.setChecked(False)
            self.ui.EndEffectorMode.setChecked(False)
            self.ui.function4.setChecked(False)
        if mode_id == 1:
            self.ui.DriveMode.setChecked(False)
            self.ui.ArmBaseMode.setChecked(True)
            self.ui.EndEffectorMode.setChecked(False)
            self.ui.function4.setChecked(False)
        if mode_id == 2:
            self.ui.DriveMode.setChecked(False)
            self.ui.ArmBaseMode.setChecked(False)
            self.ui.EndEffectorMode.setChecked(True)
            self.ui.function4.setChecked(False)
        if mode_id == 3:
            self.ui.DriveMode.setChecked(False)
            self.ui.ArmBaseMode.setChecked(False)
            self.ui.EndEffectorMode.setChecked(False)
            self.ui.function4.setChecked(True)

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())
