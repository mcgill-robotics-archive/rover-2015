#!/usr/bin/python

from VideoWindow import *
##from no_imu import *
from PyQt4 import QtCore, QtGui
from sensor_msgs.msg import Image
##import publisher

import sys
import signal
import rospy

from sensor_msgs.msg import Image

class CentralUi(QtGui.QMainWindow):

    def __init__(self, parent=None):
        super(CentralUi,self).__init__(parent)
       
        self.image1=None
        self.image2=None
        self.image3=None
      
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.signal=QtCore.QTimer(self)
        self.signal.timeout.connect(self.repaint_image)
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
    

