from mainWindow import *
##from no_imu import *
from PyQt4 import QtCore, QtGui

##import publisher
import PS3Controller_central

from VARIABLES import *
from std_msgs.msg import String

import sys
import signal
import rospy

##import rospy
import pygame

class CentralUi(QtGui.QMainWindow):

    def __init__(self, parent=None):
        self.message=""
        super(CentralUi,self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        ## creates the timers to enable or disable the ps3 controller for the controls systems
        self.ps3_timer_with_controls = QtCore.QTimer()
        ## creates the timers to enable or disable the keyboard controllers
        self.key_timer = QtCore.QTimer()
        self.ps3_timer_thrusters = QtCore.QTimer()
        self.thrust_pub_timer = QtCore.QTimer()
        ## create the ps3 controller object
        self.ps3 = PS3Controller_central.PS3Controller()

        ##button connects

        # controller timer connect
        QtCore.QObject.connect(self.ps3_timer_with_controls, QtCore.SIGNAL("timeout()"), self.update_test)

    	self.listener()

    
    def callback(self, data):
        self.message="%s is received"%data.data[13:len(data.data)]
        self.ui.label_7.setText(self.message)

    def listener(self):
    	rospy.init_node('listener',anonymous=True)
	rospy.Subscriber("chatter",String, self.callback)
	#rospy.spin()

    def set_controller_timer(self):
        if self.ps3.controller_isPresent:
            self.ps3_timer_with_controls.start(misc_vars.controller_updateFrequency)
        else:
            self.log_info("PS3 Controller not found")


    def update_test(self):
        self.ps3.updateController_for_control_systems()
        self.ui.main_menu_label.setText()

    def log_info(self, string_data):
        ##self.ui.logObject.append("[INFO] "+string_data)
        pass

    def update_message(self):
        self.ui.label_7.setText("hello");

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())
    
