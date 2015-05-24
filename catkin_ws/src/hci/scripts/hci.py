#!/usr/bin/python

from RoverWindow import *
from PyQt4 import QtCore, QtGui
from JoystickController import JoystickController
from publisher import Publisher
import pyqtgraph as pg

import sys
import rospy
import Queue
import os

from std_msgs.msg import *
from geometry_msgs.msg import Pose
from joystick_profile import ProfileParser
from rover_camera.srv import ChangeFeed
from sensor_msgs.msg import Image


class CentralUi(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(CentralUi, self).__init__(parent)

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.DriveMode.setChecked(True)
        self.controller = JoystickController()
        self.profile = ProfileParser(self.controller)
        self.publisher = Publisher()
        self.modeId = 0
        self.grip = 0

        # List of topic names for the camera feeds, compiled from parameter server
        self.camera_topic_list = []

        #signal quality timer
        self.quality_timer = QtCore.QTimer()
        QtCore.QObject.connect(self.quality_timer, QtCore.SIGNAL("timeout()"),self.get_signal_quality)
        self.quality_timer.start(1000)
        
        # map place holders
        self.tempPose = Queue.Queue()
        self.new_x = []
        self.new_y = []
        self.w1 = None
        self.s1 = None
        self.x_waypoints = []
        self.y_waypoints = []

        rospy.loginfo("Starting joystick acquisition")

        # button connects
        # controller timer connect
        self.controller_timer = QtCore.QTimer()
        self.addPointTimer = QtCore.QTimer()
        QtCore.QObject.connect(self.controller_timer, QtCore.SIGNAL("timeout()"), self.read_controller)
        QtCore.QObject.connect(self.addPointTimer, QtCore.SIGNAL("timeout()"), self.add_point_timeout)
        self.set_controller_timer()
        self.addPointTimer.start(100)

        # joystick mode buttons signal connect
        QtCore.QObject.connect(self.ui.DriveMode, QtCore.SIGNAL("clicked()"), self.set_mode0)
        QtCore.QObject.connect(self.ui.ArmBaseMode, QtCore.SIGNAL("clicked()"), self.set_mode1)
        QtCore.QObject.connect(self.ui.EndEffectorMode, QtCore.SIGNAL("clicked()"), self.set_mode2)
        QtCore.QObject.connect(self.ui.function4, QtCore.SIGNAL("clicked()"), self.set_mode3)
        QtCore.QObject.connect(self.ui.screenshot, QtCore.SIGNAL("clicked()"), self.take_screenshot)
        QtCore.QObject.connect(self.ui.pointSteer, QtCore.SIGNAL("toggled(bool)"), self.set_point_steer)
        
        # camera feed selection signal connects
        QtCore.QObject.connect(self.ui.Camera1Feed, QtCore.SIGNAL("currentIndexChanged(int)"), self.setFeed1Index)
        QtCore.QObject.connect(self.ui.Camera2Feed, QtCore.SIGNAL("currentIndexChanged(int)"), self.setFeed2Index)
        QtCore.QObject.connect(self.ui.Camera3Feed, QtCore.SIGNAL("currentIndexChanged(int)"), self.setFeed3Index)

        self.ui.pushButton.clicked.connect(self.add_way_point)
        self.ui.pushButton_2.clicked.connect(self.clear_map)

        self.setup_minimap()

        rospy.init_node('listener', anonymous=False)

        rospy.Subscriber('pose', Pose, self.handle_pose, queue_size=10)

        self.switch_feed = rospy.ServiceProxy('/changeFeed', ChangeFeed)

        self.feed1index = 0
        self.feed2index = 0
        self.feed3index = 0
        self.first_point = False
        self.dx = 0
        self.dy = 0

        self.sub = None
        rospy.loginfo("HCI initialization completed")

    def take_screenshot(self):
        self.sub = rospy.Subscriber("/image_raw", Image, self.screenshot_callback, queue_size=1)
        rospy.loginfo("created screenshot subscriber")

    def screenshot_callback(self, msg):
        rospy.loginfo("shot callback")
        self.sub.unregister()
        image = QtGui.QImage(msg.data, msg.width, msg.height, QtGui.QImage.Format_RGB888)
        save = image.save("screen.jpeg")  # TODO: give right topic for arm camera
        if save:
            rospy.loginfo("save successfull")
        else:
            rospy.logwarn("fail save")

    def setup_minimap(self):

        self.w1 = self.ui.graphicsView.addViewBox()
        self.w1.setAspectLocked(False)
        self.w1.enableAutoRange('xy', True)
        self.ui.graphicsView.nextRow()

        self.x_waypoints.append(0)
        self.y_waypoints.append(0)

        self.s1 = pg.ScatterPlotItem(size=10, pen=pg.mkPen('w'), pxMode=True)
        self.s1.addPoints(self.x_waypoints, self.y_waypoints, size=10, symbol='t', brush='b')
        self.w1.addItem(self.s1)

    def handle_pose(self, data):
        if not self.first_point:
            self.first_point = True
            self.dy = data.position.y
            self.dx = data.position.x
        # add (x,y) to tempPose queue
        self.tempPose.put(data)

    def add_point_timeout(self):
        while not self.tempPose.empty():
            pose = self.tempPose.get()
            self.new_x = [pose.position.x - self.dx]
            self.new_y = [pose.position.y - self.dy]
            self.s1.addPoints(self.new_x, self.new_y, size=3, symbol='o', brush='w')
            if self.ui.zoomGraph.isChecked():
                self.w1.autoRange()

    def add_way_point(self):
        self.x_waypoints.append(self.new_x[0])
        self.y_waypoints.append(self.new_x[0])
        self.s1.addPoints([self.new_x[0]], [self.new_y[0]], size=10, symbol='t', brush='b')
        if self.ui.zoomGraph.isChecked():
            self.w1.autoRange()

    def clear_map(self):
        self.first_point = False
        self.dx = 0
        self.dy = 0
        
        self.s1.setData([], [], size=10, symbol='o', brush='r')
        self.s1.addPoints(self.x_waypoints, self.y_waypoints, size=10, symbol='t', brush='b')

    def get_signal_quality(self):
        #s = os.popen("ping -c 1 artemis")
        s = os.popen("ping -c 1 localhost")
        s.readline()
        k = s.readline()
        temp = k.split('=')
        res = temp[-1].split(' ')
        result = res[0]
        self.ui.sig_qual.setText("%s ms"%result)

    def set_point_steer(self, boolean):
        self.publisher.setSteerMode(boolean)

    def set_controller_timer(self):
        if self.controller.controller is not None:
            self.controller_timer.start(100)
            rospy.loginfo("Started controller timer")
        else:
            rospy.loginfo("Missing controller, timer aborted")

    def read_controller(self):
        self.controller.update()
        self.profile.update_values()
        self.ui.MainX.setValue(self.controller.a1*1000)
        self.ui.MainY.setValue(-self.controller.a2*1000)
        self.ui.StickRotation.setValue(self.controller.a3*1000)
        self.ui.SecondaryY.setValue(-self.controller.a4*1000)

        if self.profile.param_value["/joystick/camera/pantilt"]:
            self.ui.Camera1Feed.setCurrentIndex(5)
        elif self.profile.param_value["/joystick/camera/arm"]:
            self.ui.Camera1Feed.setCurrentIndex(4)
        elif self.profile.param_value["/joystick/camera/haz_left"]:
            self.ui.Camera1Feed.setCurrentIndex(1)
        elif self.profile.param_value["/joystick/camera/haz_front"]:
            self.ui.Camera1Feed.setCurrentIndex(0)
        elif self.profile.param_value["/joystick/camera/haz_back"]:
            self.ui.Camera1Feed.setCurrentIndex(3)
        elif self.profile.param_value["/joystick/camera/haz_right"]:
            self.ui.Camera1Feed.setCurrentIndex(2)
        if self.profile.param_value["/joystick/coord_system"]:
            self.toggle_coordinate()
        if self.profile.param_value["/joystick/point_steer"]:
            self.ui.pointSteer.setChecked(not self.ui.pointSteer.isChecked())
        if self.profile.param_value["/joystick/drive_mode"]:
            self.set_controller_mode(0)
        elif self.profile.param_value["/joystick/arm_base_mode"]:
            self.set_controller_mode(1)
        elif self.profile.param_value["/joystick/end_effector_mode"]:
            self.set_controller_mode(2)
        elif self.controller.b10:
            self.set_controller_mode(3)
        self.controller.clear_buttons()
        # minus sign to compensate for joystick inherent positive and negative mappings
        self.publish_controls()

    def toggle_coordinate(self):
        self.ui.coordinateSystem.setCurrentIndex((self.ui.coordinateSystem.currentIndex()+1) % 2)

    def publish_controls(self):
        if self.modeId == 0:
            # drive mode
            self.publisher.publish_velocity(self.controller.a1, -self.controller.a2)

        elif self.modeId == 1:
            # arm base mode
            length = -self.controller.a2
            height = self.controller.a1
            angle = self.controller.a3
            cart = False
            vel = False
            if self.ui.coordinateSystem.currentIndex() is 1:
                cart = True
            if self.ui.arm_mode.currentIndex() is 1:
                vel = True
            self.publisher.publish_arm_base_movement(length, height, angle, cart, vel)

        elif self.modeId == 2:
            # end effector mode
            x = -self.controller.a2
            y = self.controller.a3
            rotate = self.controller.a1
            if self.grip == 0:
                if self.controller.b3:
                    self.grip = 1
                elif self.controller.b4:
                    self.grip = -1
            else:
                if self.controller.b3 or self.controller.b4:
                    self.grip = 0
            grip = self.grip

            self.publisher.publish_endEffector(x, y, rotate, grip)

            # end effector mode
            # use joystick to control a1,a2, a3 for rotating motion and some other button for grip motion

    def set_mode0(self):
        self.set_controller_mode(0)

    def set_mode1(self):
        self.set_controller_mode(1)

    def set_mode2(self):
        self.set_controller_mode(2)

    def set_mode3(self):
        self.set_controller_mode(3)

    def set_controller_mode(self, mode_id):
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

    #TODO:Demonstration of desired operation MAKE FUNCTIONAL
    def setFeed1Index(self, newIndex):
        rospy.wait_for_service('/changeFeed')
        try:
            resp = self.switch_feed(1, newIndex)
        except rospy.ServiceException:
            rospy.logwarn("Service call failed")
            self.ui.Camera1Feed.setCurrentIndex(self.feed1index)
            return
        if resp.success is 200:
            self.feed1index = newIndex
        else:
            rospy.loginfo("Switch in camera1 failed")

    def setFeed2Index(self, newIndex):
        rospy.wait_for_service('/changeFeed')
        try:
            resp = self.switch_feed(2, newIndex)
        except rospy.ServiceException:
            rospy.logwarn("Service call failed")
            self.ui.Camera1Feed.setCurrentIndex(self.feed1index)
            return
        if resp.success is 200:
            self.feed1index = newIndex
        else:
            rospy.loginfo("Switch in camera1 failed")

    def setFeed3Index(self, newIndex):
        rospy.wait_for_service('/changeFeed')
        try:
            resp = self.switch_feed(3, newIndex)
        except rospy.ServiceException:
            rospy.logwarn("Service call failed")
            self.ui.Camera1Feed.setCurrentIndex(self.feed1index)
            return
        if resp.success is 200:
            self.feed1index = newIndex
        else:
            rospy.loginfo("Switch in camera1 failed")

    def get_image_topic(self):
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
