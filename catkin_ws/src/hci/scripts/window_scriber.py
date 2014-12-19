from RoverWindow import *
##from no_imu import *
from PyQt4 import QtCore, QtGui
from sensor_msgs.msg import Image
##import publisher

from VARIABLES import *
from std_msgs.msg import String

import sys
import signal
import rospy

##import rospy
import pygame

from std_msgs.msg import String  # ros message types
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from computer_vision.msg import VisibleObjectData
from controls.msg import motorCommands
from geometry_msgs.msg import PoseStamped
from status.msg import temp, usb
from blinky.msg import RGB
from blinky.srv import *

class CentralUi(QtGui.QMainWindow):

    def __init__(self, parent=None):
        super(CentralUi,self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.signal=QtCore.QTimer()
        QtCore.QObject.connect(self.signal, QtCore.SIGNAL("timeout()"),self.checksignal)
    	
        self.input3=self.ui.Camera3Feed.currentText();

        self.listener()
        self.signal.start(100)

    
    def callback3(self, data):
        try:
            image = QtGui.QPixmap.fromImage(QtGui.QImage(data.data, data.width, data.height, QtGui.QImage.Format_RGB888))
	finally:
	    pass
	if image is not None:
            self.ui.camera3.setPixmap(image)
        else:
            self.ui.camera3.setText("No video feed")
	

    def checksignal(self):
        if self.input3!=self.ui.Camera3Feed.currentText():
            self.input3=self.ui.Camera3Feed.currentText()
            self.listen.unregister()
            if self.ui.Camera3Feed.currentText()=="feed 1":
                self.listen=rospy.Subscriber('camera_front_right/camera/image_raw',Image, self.callback)
            if self.ui.Camera3Feed.currentText()=="feed 2":
                self.listen=rospy.Subscriber('chatter2',Image, self.callback)
            if self.ui.Camera3Feed.currentText()=="feed n":
                self.listen=rospy.Subscriber('chatter3',Image, self.callback)

    def listener3(self):
        rospy.init_node('listener3',anonymous=False)
        self.listen=rospy.Subscriber("camera_front_right/camera/image_raw",Image, self.callback3)

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())
    

