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
		self.image = None
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
		self.signal=QtCore.QTimer(self)
		self.signal.timeout.connect(self.checksignal)
		self.listener3()
		self.signal.start(100)
    
    def callback3(self, data):
		try:
			self.image = data
		finally:
			pass
		

    def checksignal(self):
		if self.image is not None:
			try: 
				qimage = QtGui.QImage(self.image.data, self.image.width/3, self.image.height/3, QtGui.QImage.Format_RGB888)
				#TODO: FIGURE OUT THIS DIVIDED BY 3 !!! 
				# wiskey-tango-foxtrot
				image = QtGui.QPixmap.fromImage(qimage)		
			finally:
				pass
			self.ui.camera3.setPixmap(image)
		else:
			self.ui.camera3.setText("no video feed")
	

    def listener3(self):
        rospy.init_node('listener3',anonymous=False)
        self.listen=rospy.Subscriber("/camera_front_right/camera/image_raw",Image, self.callback3)

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())
    

