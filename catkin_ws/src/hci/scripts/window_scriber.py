#!/usr/bin/python

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

class CentralUi(QtGui.QMainWindow):

    def __init__(self, parent=None):
        super(CentralUi,self).__init__(parent)
        self.image1=None
        self.image2=None
        self.image3=None
        self.image4=None
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.signal=QtCore.QTimer(self)
        self.signal.timeout.connect(self.repaint_image)
        self.signal.timeout.connect(self.check_signal)
        self.listener()
        self.signal.start(100)
    
    def callback3(self, data):
        try:
        	self.image3 = data
        finally:
        	pass
    def callback2(self, data):
        try:
            self.image2=data
        finally:
            pass
    def callback1(self, data):
        try:
            self.image1=data
        finally:
            pass
    def callback4(self,data):
        try:
            self.image4=data
        finally:
            pass
	
    def check_signal(self):
        if self.ui.Camera1Feed.currentIndex() != 0:
            self.listen1.unregister()
            self.listen1=rospy.Subscriber("/hello",Image,self.callback4)
            self.image1=None
        else:
            self.listen1.unregister()
            self.listen1=rospy.Subscriber("/camera_front_right/camera/image_raw",Image, self.callback1)
        if self.ui.Camera2Feed.currentIndex() != 0:
            self.listen2.unregister()
            self.listen2=rospy.Subscriber("/hello",Image,self.callback4)
            self.image2=None
        else:
            self.listen2.unregister()
            self.listen2=rospy.Subscriber("/camera_front_right/camera/image_raw",Image, self.callback2)    
        if self.ui.Camera3Feed.currentIndex() != 0:
            self.listen3.unregister()
            self.listen3=rospy.Subscriber("/hello",Image,self.callback4)
            self.image3=None
        else:
            self.listen3.unregister()
            self.listen3=rospy.Subscriber("/camera_front_right/camera/image_raw",Image, self.callback3)

    def repaint_image(self):
        if self.image2 is not None:
            try:
                qimage2 = QtGui.QImage(self.image2.data, self.image2.width/3, self.image2.height/3,QtGui.QImage.Format_RGB888)
                #TODO: FIGURE OUT THIS DIVIDED BY 3 !!! 
                # wiskey-tango-foxtrot
                image2 = QtGui.QPixmap.fromImage(qimage2)     
            finally:
                pass
            self.ui.camera2.setPixmap(image2)
        else:
            self.ui.camera2.setText("no video feed")
        if self.image3 is not None:
        	try: 
        		qimage3 = QtGui.QImage(self.image3.data, self.image3.width/3, self.image3.height/3,QtGui.QImage.Format_RGB888)
        		#TODO: FIGURE OUT THIS DIVIDED BY 3 !!! 
        		# wiskey-tango-foxtrot
        		image3 = QtGui.QPixmap.fromImage(qimage3)		
        	finally:
        		pass
        	self.ui.camera3.setPixmap(image3)
        else:
        	self.ui.camera3.setText("no video feed")
        if self.image1 is not None:
            try: 
                qimage1 = QtGui.QImage(self.image1.data, self.image1.width/3, self.image1.height/3,QtGui.QImage.Format_RGB888)
                #TODO: FIGURE OUT THIS DIVIDED BY 3 !!! 
                # wiskey-tango-foxtro
                image1 = QtGui.QPixmap.fromImage(qimage1)     
            finally:
                pass
            self.ui.camera1.setPixmap(image1)
        else:
            self.ui.camera1.setText("no video feed")



	

    def listener(self):
        rospy.init_node('listener',anonymous=False)
        self.listen3=rospy.Subscriber("/camera_front_right/camera/image_raw",Image, self.callback3)
        self.listen2=rospy.Subscriber("/camera_front_right/camera/image_raw",Image, self.callback2)
        self.listen1=rospy.Subscriber("/camera_front_right/camera/image_raw",Image, self.callback1)


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())
    

