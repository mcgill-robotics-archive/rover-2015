#!/usr/bin/python

from VideoWindow import *
from PyQt4 import QtCore, QtGui

import sys
import signal

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

class CentralUi(QtGui.QMainWindow):

    def __init__(self, parent=None):
        super(CentralUi,self).__init__(parent)
        
        self.feedTopics=[]
        self.imageMain = None
        self.imageTop = None
        self.imageBottom = None     
     
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        self.signal=QtCore.QTimer(self)
        self.signal.timeout.connect(self.repaint_image)
        
        self.ros_init()

        self.signal.start(1000/60)
    
 
 
    def receiveimageMain(self, data):
        try:
            self.imageMain = data
        finally:
            pass


    def receiveimageTop(self, data):
        try:
            self.imageTop = data
        finally:
            pass


    def receiveimageBottom(self, data):
        try:
            self.imageBottom = data
        finally:
            pass
   

    def repaint_image(self):
        if self.imageMain is not None:
            try:
                qimageMain = QtGui.QImage(self.imageMain.data, self.imageMain.width, self.imageMain.height,QtGui.QImage.Format_RGB888)
                #TODO: FIGURE OUT THIS DIVIDED BY 3 !!! 
                # wiskey-tango-foxtrot
                imageMain = QtGui.QPixmap.fromImage(qimageMain)     
            finally:
                pass
            self.ui.camera1.setPixmap(imageMain)
        else:
            self.ui.camera1.setText("no video feed")
        
        if self.imageTop is not None:
        	try: 
        		qimageTop = QtGui.QImage(self.imageTop.data, self.imageTop.width, self.imageTop.height,QtGui.QImage.Format_RGB888)
        		#TODO: FIGURE OUT THIS DIVIDED BY 3 !!! 
        		# wiskey-tango-foxtrot
        		imageTop = QtGui.QPixmap.fromImage(qimageTop)		
        	finally:
        		pass
        	self.ui.camera2.setPixmap(imageTop)
        else:
        	self.ui.camera2.setText("no video feed")

        if self.imageBottom is not None:
            try: 
                qimageBottom = QtGui.QImage(self.imageBottom.data, self.imageBottom.width/3, self.imageBottom.height/3,QtGui.QImage.Format_RGB888)
                #TODO: FIGURE OUT THIS DIVIDED BY 3 !!! 
                # wiskey-tango-foxtro
                imageBottom = QtGui.QPixmap.fromImage(qimageBottom)     
            finally:
                pass
            self.ui.camera3.setPixmap(imageBottom)
        else:
            self.ui.camera3.setText("no video feed")


    def ros_init(self):
        rospy.init_node('camera_viewer',anonymous=True)  
        self.getimageTopic() 
        rospy.Subscriber("/image_raw",Image, self.receiveimageMain)
        rospy.Subscriber(self.feedTopics[1], Image, self.receiveimageTop)
        rospy.Subscriber(self.feedTopics[2], Image, self.receiveimageBottom)


    def getimageTopic(self):
        self.feedTopics.append(rospy.get_param("feed/topicMain", "/feed1/image_raw"))
        self.feedTopics.append(rospy.get_param("feed/topicTop", "/feed2/image_raw"))
        self.feedTopics.append(rospy.get_param("feed/topicBottom", "/feed3/image_raw"))
        rospy.loginfo(self.feedTopics)


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())
    
