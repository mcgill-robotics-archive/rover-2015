#!/usr/bin/python

from RoverWindow import *
##from no_imu import *
from PyQt4 import QtCore, QtGui
##import publisher
from JoystickController import JoystickController
from VARIABLES import *
from publisher import Publisher
import pyqtgraph as pg

import sys
import rospy
import Queue

from std_msgs.msg import String  # ros message types
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose

#TODO: create service
# from hci import Switch_Feeds

class CentralUi(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(CentralUi,self).__init__(parent)

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.controller = JoystickController()
        self.publisher = Publisher()
        self.modeId = 0
        self.grip = 0

        ## List of topic names for the camera feeds, compiled from parameter server
        self.camera_topic_list = []

        # map place holders
        self.tempPose = Queue.Queue()
        self.newx = []
        self.newy = []
        self.w1 = None
        self.s1 = None
        self.x_waypoints = []
        self.y_waypoints = []

        rospy.loginfo("Starting joystick acquisition")

        ##button connects
        # controller timer connect
        self.controller_timer = QtCore.QTimer()
        self.addPointTimer = QtCore.QTimer()
        QtCore.QObject.connect(self.controller_timer, QtCore.SIGNAL("timeout()"), self.readController)
        QtCore.QObject.connect(self.addPointTimer, QtCore.SIGNAL("timeout()"), self.addPointTimeout)
        self.setControllerTimer()

        self.addPointTimer.start(100)
        # joystick mode buttons signal connect
        QtCore.QObject.connect(self.ui.DriveMode, QtCore.SIGNAL("clicked()"), self.setMode0)
        QtCore.QObject.connect(self.ui.ArmBaseMode, QtCore.SIGNAL("clicked()"), self.setMode1)
        QtCore.QObject.connect(self.ui.EndEffectorMode, QtCore.SIGNAL("clicked()"), self.setMode2)
        QtCore.QObject.connect(self.ui.function4, QtCore.SIGNAL("clicked()"), self.setMode3)
        # connect for point steer mode
        QtCore.QObject.connect(self.ui.pointSteer, QtCore.SIGNAL("toggled(bool)"), self.setPointSteer)
        # camera feed selection signal connects
        #QtCore.QObject.connect(self.ui.Camera1Feed, QtCore.SIGNAL("currentIndexChanged(int)"), self.setFeed1Index)
        #QtCore.QObject.connect(self.ui.Camera2Feed, QtCore.SIGNAL("currentIndexChanged(int)"), self.setFeed2Index)
        #QtCore.QObject.connect(self.ui.Camera3Feed, QtCore.SIGNAL("currentIndexChanged(int)"), self.setFeed3Index)
        #TODO: Remove comments when done with switcher implementation


        self.ui.pushButton.clicked.connect(self.addWayPoint)
        self.ui.pushButton_2.clicked.connect(self.clearMap)

        self.setupMinimap()

        rospy.init_node('listener',anonymous=False)
                
        rospy.Subscriber('pose', Pose, self.handlePose)

        #self.switch_feed = rospy.ServiceProxy('switch_feeds', Switch_Feeds) #TODO: FIX NAME

        self.feed1index = 0
        self.feed2index = 0
        self.feed3index = 0

        rospy.loginfo("HCI initialization completed")


    def setupMinimap(self):

        self.w1 = self.ui.graphicsView.addViewBox()
        self.w1.setAspectLocked(False)
        self.w1.enableAutoRange('xy',True)
        self.ui.graphicsView.nextRow()

        self.x_waypoints.append(0)
        self.y_waypoints.append(0)

        self.s1 = pg.ScatterPlotItem(size=10, pen=pg.mkPen('w'), pxMode=True)
        self.s1.addPoints(self.x_waypoints, self.y_waypoints, size=10,symbol='t',brush='b')
        self.w1.addItem(self.s1)

    def handlePose(self,data):
        #add (x,y) to tempPose queue
        self.tempPose.put(data)

    def addPointTimeout(self):
        while not self.tempPose.empty():
            pose=self.tempPose.get()
            self.newx=[pose.position.x]
            self.newy=[pose.position.y]
            self.s1.addPoints(self.newx,self.newy, size=3, symbol='o', brush='w')
            self.w1.autoRange()


    def addWayPoint(self):
        self.x_waypoints.append(self.newx[0])
        self.y_waypoints.append(self.newx[0])
        self.s1.addPoints([self.newx[0]],[self.newy[0]],size=10,symbol='t',brush='b')
        self.w1.autoRange()

    def clearMap(self):
        self.s1.setData([],[],size=10,symbol='o',brush='r')
        self.s1.addPoints(self.x_waypoints,self.y_waypoints,size=10,symbol='t',brush='b')

    def setPointSteer(self, boolean):
        self.publisher.setSteerMode(boolean)

    def setControllerTimer(self):
        if self.controller.controller is not None:
            self.controller_timer.start(100)
            rospy.loginfo("Started controller timer")
        else:
            rospy.loginfo("Missing controller, timer aborted")

    def readController(self):
        self.controller.update()
        self.ui.MainX.setValue(self.controller.a1*1000)
        self.ui.MainY.setValue(-self.controller.a2*1000)
        self.ui.StickRotation.setValue(self.controller.a3*1000)
        self.ui.SecondaryY.setValue(-self.controller.a4*1000)

        if self.controller.b3:
            self.ui.Camera1Feed.setCurrentIndex(5)
        elif self.controller.b4:
            self.ui.Camera1Feed.setCurrentIndex(4)
        elif self.controller.b2:
            self.ui.pointSteer.setChecked(not self.ui.pointSteer.isChecked())
        elif self.controller.b7:
            self.setControllerMode(0)
        elif self.controller.b8:
            self.setControllerMode(1)
        elif self.controller.b9:
            self.setControllerMode(2)
        elif self.controller.b10:
            self.setControllerMode(3)
        elif self.controller.hat ==(-1,0):
            self.ui.Camera1Feed.setCurrentIndex(1)
        elif self.controller.hat ==(0,1):
            self.ui.Camera1Feed.setCurrentIndex(0)
        elif self.controller.hat ==(0,-1):
            self.ui.Camera1Feed.setCurrentIndex(3)
        elif self.controller.hat ==(1,0):
            self.ui.Camera1Feed.setCurrentIndex(2)
        self.controller.clear_buttons()
        #minus sign to compensate for joystick inherent positive and negative mappings
        self.publishControls()

    def publishControls(self):
        if self.modeId == 0:
            self.publisher.publish_velocity(self.controller.a1, -self.controller.a2)
        elif self.modeId == 1:
            length = -self.controller.a2
            height = self.controller.a1
            angle = self.controller.a3
            self.publisher.publish_arm_base_movement(length,height,angle)
        elif self.modeId == 2:
            x = -self.controller.a2
            y = self.controller.a3
            rotate = self.controller.a1
            if self.grip == 0 :
                if self.controller.b3 :
                    self.grip = 1
                elif self.controller.b4 :
                    self.grip = -1
            else :
                if self.controller.b3 or self.controller.b4 :
                    self.grip = 0
            grip = self.grip

            self.publisher.publish_endEffector(x,y,rotate,grip)

            #end effector mode
            #use joystick to controll a1,a2, a3 for rotating motion and someother button for grip motion

    def setMode0(self):
        self.setControllerMode(0)

    def setMode1(self):
        self.setControllerMode(1)

    def setMode2(self):
        self.setControllerMode(2)

    def setMode3(self):
        self.setControllerMode(3)

    def setControllerMode(self, mode_id):
        self.modeId = mode_id
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

    """ #TODO:Demonstration of desired operation MAKE FUNCTIONAL
    def setFeed1Index(self, newIndex):
        rospy.wait_for_service('switch_feeds')
        try:
            topic = self.camera_topic_list[newIndex]
            resp = self.switch_feed(0,topic)
        except IndexError :
            rospy.logwarn("New camera feed topic name not found")
            self.ui.Camera1Feed.setCurrentIndex(self.feed1index)
            return
        except rospy.ServiceException :
            rospy.logwarn("Service call failed")
            self.ui.Camera1Feed.setCurrentIndex(self.feed1index)
            return
        if resp.success is True:
            self.feed1index = newIndex
        else:
            self.ui.Camera1Feed.setCurrentIndex(self.feed1index)
            rospy.loginfo("Switch in camera1 failed")
        #TODO: Service call to switcher application for proper topic to proper screen

    def setFeed2Index(self, newIndex):
        try:
            topic = self.camera_topic_list[newIndex]
        except IndexError :
            rospy.logwarn("New camera feed topic name not found")
            self.ui.Camera2Feed.setCurrentIndex(self.feed2index)
            return
        #TODO: Service call to switcher application for proper topic to proper screen

    def setFeed3Index(self, newIndex):
        try:
            topic = self.camera_topic_list[newIndex]
        except IndexError :
            rospy.logwarn("New camera feed topic name not found")
            self.ui.Camera3Feed.setCurrentIndex(self.feed3index)
            return
        #TODO: Service call to switcher application for proper topic to proper screen
    """

    def getImageTopic(self):
        self.camera_topic_list.append(rospy.get_param("camera/topic_haz_front", "/hazcam_front/camera/image_raw"))
        self.camera_topic_list.append(rospy.get_param("camera/topic_haz_left", "/hazcam_left/camera/image_raw"))
        self.camera_topic_list.append(rospy.get_param("camera/topic_haz_right", "/hazcam_right/camera/image_raw"))
        self.camera_topic_list.append(rospy.get_param("camera/topic_haz_back", "/hazcam_back/camera/image_raw"))
        self.camera_topic_list.append(rospy.get_param("camera/topic_arm", "/camera_arm/camera/image_raw"))
        self.camera_topic_list.append(rospy.get_param("camera/topic_pan_tilt", "/camera_pan_tilt/camera/image_raw"))
        rospy.loginfo(self.camera_topic_list)


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())
