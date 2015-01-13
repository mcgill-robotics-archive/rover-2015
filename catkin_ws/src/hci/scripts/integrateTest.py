#!/usr/bin/python

from RoverWindow import *
##from no_imu import *
from PyQt4 import QtCore, QtGui
##import publisher
from JoystickController import JoystickController
from VARIABLES import *

import sys
import signal
import rospy

##import rospy

from std_msgs.msg import String  # ros message types
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class CentralUi(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(CentralUi,self).__init__(parent)

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.controller = JoystickController()

        self.image1=None
        self.image2=None
        self.image3=None
        self.image4=None
       

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

        self.signal=QtCore.QTimer(self)
        self.signal.timeout.connect(self.repaint_image)
        self.signal.timeout.connect(self.check_signal)
        self.listener()
        self.signal.start(100)


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
        self.controller.b2 = False
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
        
        
        
    #publisher for velocity
    #TODO change names once control systems has a defined topic name and variable names, copied from AUV as of now
    def publish_velocity(self):
        vel_pub = rospy.Publisher("electrical_interface/motor",Twist)
        
        msg = Twist()
        
        msg.linear.x = self.controller.a1
        msg.linear.y = self.controller.a2
        vel_pub.publish(msg)
    
    #publish 2 main joystick axes for arm base movement (mode must be arm)
    def publish_arm_base_movement(self):
        arm_movement_pub = rospy.Publisher("electrical_interface/arm",Twist)
        msg = Twist()
        
        msg.linear.x = self.controller.a1
        msg.linear.y = self.controller.a2
        vel_pub.publish(msg)
        
    #publish joystick3 for rotating hand (mode must be arm)
    def publish_arm_rotation(self):
        arm_rotate_pub = rospy.Publisher("electrical_interface/arm",int)
        msg = self.controller.a3
        arm_rotate_pub(msg)
            
    #publish camera zoom from axis 4
    def publish_zoom(self):
        zoom_pub = rospy.Publisher("electrical_interface/cameraZoom",int)
        
        msg = self.controller.a4
        zoom_pub.publish(msg)
        
    #publish camera pan from axis 3 (mode must be drive)
    def publish_pan(self):
        pan_pub = rospy.Publisher("electrical_interface/cameraPan",int)
        
        msg = self.controller.a3
        pan_pub.publish(msg)
        








if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())
