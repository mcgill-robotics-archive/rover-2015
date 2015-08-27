#!/usr/bin/python
import sys

from RoverWindow import *
from PyQt4 import QtCore, QtGui
from JoystickController import JoystickController
from publisher import Publisher
import pyqtgraph as pg

import rospy
import Queue
import os
import math

from std_msgs.msg import *
from joystick_profile import ProfileParser
from sensor_msgs.msg import Image
from rover_msgs.msg import MotorControllerMode, MotorStatus, AhrsStatusMessage
from rover_msgs.srv import ResetWatchDog
from sensor_msgs.msg import CompressedImage


def reset_watchdog():
    # reset watchdog
    rospy.wait_for_service("reset_watchdog", 0.1)

    try:
        reset = rospy.ServiceProxy("reset_watchdog", ResetWatchDog)
        response = reset(1, 10)
        if response.Response != 555:
            rospy.logerr("Bad response")

    except rospy.ServiceException, e:
        print "Service call failed: %s", e


def lbl_bg_red(thing):
    """sets a style sheet to the @param thing resulting in a red background"""
    thing.setStyleSheet('background-color:#ff0000')
    thing.setText("Bad")


def lbl_bg_norm(thing):
    """sets a style sheet to the @param thing resulting in a green background"""
    thing.setStyleSheet('background-color:#33CC33')
    thing.setText("Ok")


def format_dms(dec_deg):
    deg = int(dec_deg)
    mins = (abs(dec_deg - deg) * 60)
    mins_i = int(mins)
    sec = (mins - mins_i) * 60

    return "%i%c %i' %.3f''" % (deg, chr(176), mins_i, sec)


def format_euler_angle(angle):

    deg = math.degrees(angle)
    string = "%.2f" % deg
    return string


class CentralUi(QtGui.QMainWindow):

    fl_signal_ok = QtCore.pyqtSignal()
    fr_signal_ok = QtCore.pyqtSignal()
    ml_signal_ok = QtCore.pyqtSignal()
    mr_signal_ok = QtCore.pyqtSignal()
    bl_signal_ok = QtCore.pyqtSignal()
    br_signal_ok = QtCore.pyqtSignal()

    fl_signal_bad = QtCore.pyqtSignal()
    fr_signal_bad = QtCore.pyqtSignal()
    ml_signal_bad = QtCore.pyqtSignal()
    mr_signal_bad = QtCore.pyqtSignal()
    bl_signal_bad = QtCore.pyqtSignal()
    br_signal_bad = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super(CentralUi, self).__init__(parent)

        self.points_counter = 0
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.DriveMode.setChecked(True)
        self.ui.arm_mode.setCurrentIndex(1)
        self.ui.ackMoving.setChecked(False)

        self.controller = JoystickController()
        self.profile = ProfileParser(self.controller)
        self.publisher = Publisher()

        self.quality_timer = None
        self.addPointTimer = None
        self.controller_timer = None
        self.watchdog_timer = None
        self.redraw_signal = None

        self.sub = None
        self.feed_topics = []

        self.modeId = 0
        self.grip = 0

        # image
        self.imageMain = None
        self.imageLeft = None
        self.imageRight = None
        self.main_camera_subscriber = None

        # map place holders
        self.tempPose = Queue.Queue()
        self.new_x = []
        self.new_y = []
        self.w1 = None
        self.s1 = None

        self.x_waypoints = []
        self.y_waypoints = []

        # self.cam_x = 0
        # self.cam_y = 0

        self.first_point = False
        self.dx = 0
        self.dy = 0

        # list for set of points in mini-map
        self.map_point_list = []

        self.init_ros()
        self.init_connects()
        self.init_timers()
        self.setup_minimap()

        rospy.loginfo("HCI initialization completed")

    def init_ros(self):
        rospy.init_node('listener', anonymous=False)
        rospy.Subscriber('ahrs_status', AhrsStatusMessage, self.handle_pose, queue_size=10)
        rospy.Subscriber('/motor_status', MotorStatus, self.motor_status, queue_size=10)
        self.main_camera_subscriber = rospy.Subscriber("/econ", CompressedImage, self.receive_image_main)
        rospy.Subscriber("/left_nav/image_mono/compressed", CompressedImage, self.receive_image_left)
        rospy.Subscriber("/right_nav/image_mono/compressed", CompressedImage, self.receive_image_right)

    def motor_status(self, msg):
        if msg.fl:
            self.fl_signal_ok.emit()
        else:
            self.fl_signal_bad.emit()

        if msg.fr:
            self.fr_signal_ok.emit()
        else:
            self.fr_signal_bad.emit()

        if msg.ml:
            self.ml_signal_ok.emit()
        else:
            self.ml_signal_bad.emit()

        if msg.mr:
            self.mr_signal_ok.emit()
        else:
            self.mr_signal_bad.emit()

        if msg.bl:
            self.bl_signal_ok.emit()
        else:
            self.bl_signal_bad.emit()

        if msg.br:
            self.br_signal_ok.emit()
        else:
            self.br_signal_bad.emit()

    def init_connects(self):
        # joystick mode buttons signal connect
        QtCore.QObject.connect(self.ui.DriveMode, QtCore.SIGNAL("clicked()"),
                               lambda index=0: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.ArmBaseMode, QtCore.SIGNAL("clicked()"),
                               lambda index=1: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.EndEffectorMode, QtCore.SIGNAL("clicked()"),
                               lambda index=2: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.function4, QtCore.SIGNAL("clicked()"),
                               lambda index=3: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.screenshot, QtCore.SIGNAL("clicked()"), self.take_screenshot)
        QtCore.QObject.connect(self.ui.pointSteer, QtCore.SIGNAL("toggled(bool)"), self.set_point_steer)
        QtCore.QObject.connect(self.ui.ackreman, QtCore.SIGNAL("toggled(bool)"), self.set_ackreman)
        QtCore.QObject.connect(self.ui.skid, QtCore.SIGNAL("toggled(bool)"), self.set_skid)
        QtCore.QObject.connect(self.ui.translatory, QtCore.SIGNAL("toggled(bool)"), self.set_translatory)
        QtCore.QObject.connect(self.ui.add_waypoint_dd, QtCore.SIGNAL("clicked()"), self.add_coord_dd)
        QtCore.QObject.connect(self.ui.add_waypoint_dms, QtCore.SIGNAL("clicked()"), self.add_coord_dms)

        # camera feed selection signal connects
        QtCore.QObject.connect(self.ui.waypoint, QtCore.SIGNAL("clicked()"), self.add_way_point)
        QtCore.QObject.connect(self.ui.clearMap, QtCore.SIGNAL("clicked()"), self.clear_map)
        QtCore.QObject.connect(self.ui.driveModeSelection, QtCore.SIGNAL("currentIndexChanged(int)"), self.set_motor_controller_mode)
        QtCore.QObject.connect(self.ui.camera_selector, QtCore.SIGNAL("currentIndexChanged()"), self.change_video_feed)

        # motor readys
        self.fl_signal_ok.connect(lambda lbl=self.ui.fl_ok: lbl_bg_norm(lbl))
        self.fr_signal_ok.connect(lambda lbl=self.ui.fr_ok: lbl_bg_norm(lbl))
        self.ml_signal_ok.connect(lambda lbl=self.ui.ml_ok: lbl_bg_norm(lbl))
        self.mr_signal_ok.connect(lambda lbl=self.ui.mr_ok: lbl_bg_norm(lbl))
        self.bl_signal_ok.connect(lambda lbl=self.ui.bl_ok: lbl_bg_norm(lbl))
        self.br_signal_ok.connect(lambda lbl=self.ui.br_ok: lbl_bg_norm(lbl))

        self.fl_signal_bad.connect(lambda lbl=self.ui.fl_ok: lbl_bg_red(lbl))
        self.fr_signal_bad.connect(lambda lbl=self.ui.fr_ok: lbl_bg_red(lbl))
        self.ml_signal_bad.connect(lambda lbl=self.ui.ml_ok: lbl_bg_red(lbl))
        self.mr_signal_bad.connect(lambda lbl=self.ui.mr_ok: lbl_bg_red(lbl))
        self.bl_signal_bad.connect(lambda lbl=self.ui.bl_ok: lbl_bg_red(lbl))
        self.br_signal_bad.connect(lambda lbl=self.ui.br_ok: lbl_bg_red(lbl))

    def init_timers(self):
        # signal quality timer
        self.quality_timer = QtCore.QTimer()
        QtCore.QObject.connect(self.quality_timer, QtCore.SIGNAL("timeout()"), self.get_signal_quality)
        self.quality_timer.start(1000)

        # add point to map
        self.addPointTimer = QtCore.QTimer()
        QtCore.QObject.connect(self.addPointTimer, QtCore.SIGNAL("timeout()"), self.add_point_timeout)
        self.addPointTimer.start(100)

        # add point to map
        # self.watchdog_timer = QtCore.QTimer()
        # QtCore.QObject.connect(self.watchdog_timer, QtCore.SIGNAL("timeout()"), reset_watchdog)
        # self.watchdog_timer.start(100)

        # button connects
        # controller timer connect

        self.controller_timer = QtCore.QTimer()
        QtCore.QObject.connect(self.controller_timer, QtCore.SIGNAL("timeout()"), self.read_controller)

        self.redraw_signal = QtCore.QTimer(self)
        QtCore.QObject.connect(self.redraw_signal, QtCore.SIGNAL("timeout()"), self.repaint_image)
        self.redraw_signal.start(100)

        if self.controller.controller is not None:
            self.controller_timer.start(100)
            rospy.loginfo("Started controller timer")
        else:
            rospy.loginfo("Missing controller, timer aborted")

    def set_motor_controller_mode(self, modeIndex):
        msg = MotorControllerMode()
        if modeIndex == 0:
            msg.lowSpeed = 1
            msg.medSpeed = 0
            msg.highSpeed = 0
            msg.openLoop = 0
        elif modeIndex == 1:
            msg.lowSpeed = 0
            msg.medSpeed = 1
            msg.highSpeed = 0
            msg.openLoop = 0
        elif modeIndex == 2:
            msg.lowSpeed = 0
            msg.medSpeed = 0
            msg.highSpeed = 1
            msg.openLoop = 0
        elif modeIndex == 3:
            msg.lowSpeed = 0
            msg.medSpeed = 0
            msg.highSpeed = 0
            msg.openLoop = 1
            #
        print msg
        self.publisher.publish_mode(msg)
        pass

    def take_screenshot(self):
        self.sub = rospy.Subscriber("/image_raw", Image, self.screenshot_callback, queue_size=1)
        rospy.loginfo("created screenshot subscriber")

    def screenshot_callback(self, msg):
        rospy.loginfo("shot callback")
        self.sub.unregister()
        image = QtGui.QImage(msg.data, msg.width, msg.height, QtGui.QImage.Format_RGB888)
        save = image.save("screen.jpeg")  # TODO: give right topic for arm camera
        if save:
            rospy.loginfo("save successful")
        else:
            rospy.logwarn("fail save")

    def add_point_set_to_mini_map(self):
        new_set = pg.ScatterPlotItem(size=10, pen=pg.mkPen('w'), pxMode=True)  # create new point set
        self.map_point_list.append(new_set)  # add point set to member list
        self.w1.addItem(self.map_point_list[-1])  # add point set to graph window
        rospy.loginfo("Added new scatter plot item")

    def setup_minimap(self):

        self.w1 = self.ui.graphicsView.addViewBox()
        self.w1.setAspectLocked(False)
        self.w1.enableAutoRange('xy', True)
        self.ui.graphicsView.nextRow()

        self.x_waypoints.append(0)
        self.y_waypoints.append(0)

        self.add_point_set_to_mini_map()
        self.map_point_list[-1].addPoints(self.x_waypoints, self.y_waypoints, size=10, symbol='t', brush='b')

    def handle_pose(self, data):
        if data.gpsLongitude is not 0:
            if not self.first_point:
                self.first_point = True
                self.dx = data.gpsLongitude
                self.dy = data.gpsLatitude

            # add (x,y) to tempPose queue
            self.tempPose.put(data)

    def add_point_timeout(self):
        while not self.tempPose.empty():
            pose = self.tempPose.get()

            self.new_x = [(pose.gpsLongitude - self.dx)]
            self.new_y = [(pose.gpsLatitude - self.dy)]
            self.ui.latActual.setText(format_dms(pose.gpsLatitude))
            self.ui.lonActual.setText(format_dms(pose.gpsLongitude))

            self.ui.pitchLBL.setText(format_euler_angle(pose.pitch))
            self.ui.rollLBL.setText(format_euler_angle(pose.roll))
            self.ui.yawLBL.setText(format_euler_angle(pose.yaw))

            self.points_counter += 1
            if self.points_counter % 1000 == 0:
                self.add_point_set_to_mini_map()

            self.map_point_list[-1].addPoints(self.new_x, self.new_y, size=3, symbol='o', brush='w')
            if self.ui.zoomGraph.isChecked():
                self.w1.autoRange()

    def add_way_point(self):
        self.x_waypoints.append(self.new_x[0])
        self.y_waypoints.append(self.new_y[0])
        self.map_point_list[-1].addPoints([self.new_x[0]], [self.new_y[0]], size=10, symbol='t', brush='b')
        if self.ui.zoomGraph.isChecked():
            self.w1.autoRange()

    def add_coord_dms(self):
        longitude = self.ui.lon_deg.value() + self.ui.lon_min.value() / 60.0 + self.ui.lon_sec.value() / 3600.0
        latitude = self.ui.lat_deg.value() + self.ui.lat_min.value() / 60.0 + self.ui.lat_sec.value() / 3600.0

        if self.ui.lat_sign.currentIndex() == 1:
            latitude = - latitude

        if self.ui.lon_sign.currentIndex() == 1:
            longitude = - longitude

        x = longitude - self.dx
        y = latitude - self.dy
        self.x_waypoints.append(x)
        self.y_waypoints.append(y)
        self.map_point_list[-1].addPoints([x], [y], size=10, symbol='t', brush='r')
        if self.ui.zoomGraph.isChecked():
            self.w1.autoRange()

    def add_coord_dd(self):
        x = self.ui.x.value() - self.dx
        y = self.ui.y.value() - self.dy
        self.x_waypoints.append(x)
        self.y_waypoints.append(y)
        self.map_point_list[-1].addPoints([x], [y], size=10, symbol='t', brush='r')
        if self.ui.zoomGraph.isChecked():
            self.w1.autoRange()

    def clear_map(self):
        self.first_point = False
        self.dx = 0
        self.dy = 0

        for item in self.map_point_list:
            self.w1.removeItem(item)
            self.map_point_list.remove(item)

        self.add_point_set_to_mini_map()

        self.map_point_list[-1].setData([], [], size=10, symbol='o', brush='r')
        self.map_point_list[-1].addPoints(self.x_waypoints, self.y_waypoints, size=10, symbol='t', brush='b')

    def get_signal_quality(self):
        # s = os.popen("ping -c 1 air")
        s = os.popen("ping -c 1 localhost")
        s.readline()
        k = s.readline()
        temp = k.split('=')
        res = temp[-1].split(' ')
        result = res[0]
        self.ui.sig_qual.setText("%s ms"%result)

    def set_point_steer(self, boolean):
        if boolean:
            self.publisher.setSteerMode(0)

    def set_ackreman(self, boolean):
        if boolean:
            self.publisher.setSteerMode(1)

    def set_skid(self, boolean):
        if boolean:
            self.publisher.setSteerMode(3)

    def set_translatory(self, boolean):
        if boolean:
            self.publisher.setSteerMode(2)

    def read_controller(self):
        self.controller.update()
        self.profile.update_values()

        if self.profile.param_value["/joystick/coord_system"]:
            self.toggle_coordinate()
        if self.profile.param_value["/joystick/point_steer"]:
            self.ui.pointSteer.setChecked(not self.ui.pointSteer.isChecked())
        if self.profile.param_value["/joystick/skid_steer"]:
            self.ui.skid.setChecked(not self.ui.skid.isChecked())
        if self.profile.param_value["/joystick/ackreman"]:
            self.ui.ackreman.setChecked(not self.ui.ackreman.isChecked())
        if self.profile.param_value["/joystick/ackreman_moving"]:
            self.ui.ackMoving.setChecked(not self.ui.ackMoving.isChecked())
        if self.profile.param_value["/joystick/drive_mode"]:
            self.set_controller_mode(0)
        elif self.profile.param_value["/joystick/camera_mode"]:
            self.set_controller_mode(3)
        elif self.profile.param_value["/joystick/arm_base_mode"]:
            self.set_controller_mode(1)
        elif self.profile.param_value["/joystick/end_effector_mode"]:
            self.set_controller_mode(2)

        if self.profile.param_value["joystick/prev_cam"]:
            self.ui.camera_selector.setCurrentIndex((self.ui.camera_selector.currentIndex() - 1) % self.ui.camera_selector.count())
        elif self.profile.param_value["joystick/next_cam"]:
            self.ui.camera_selector.setCurrentIndex((self.ui.camera_selector.currentIndex() + 1) % self.ui.camera_selector.count())

        self.controller.clear_buttons()
        self.publish_controls()

    def get_feed_topic_params(self):
        for index in xrange(0, self.ui.camera_selector.count()):
            box_text = self.ui.camera_selector.itemText(index)
            param_value = rospy.get_param(box_text, "")
            self.feed_topics.append(param_value)

    def change_video_feed(self):
        next_topic = self.feed_topics[self.ui.camera_selector.currentIndex()]
        if next_topic is not "":
            self.main_camera_subscriber.unregister()
            self.main_camera_subscriber = rospy.Subscriber(next_topic, CompressedImage, self.receive_image_main)

    def toggle_coordinate(self):
        self.ui.coordinateSystem.setCurrentIndex((self.ui.coordinateSystem.currentIndex()+1) % 2)

    def publish_controls(self):
        if self.modeId == 0:
            # drive mode
            self.publisher.publish_velocity(self.controller.a1, -self.controller.a2, self.ui.ackMoving.isChecked())

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

        elif self.modeId == 3:
            # self.cam_x += self.controller.a1
            # self.cam_y += self.controller.a2
            self.publisher.publish_camera(-self.controller.a1, self.controller.a2)

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

    def receive_image_main(self, data):
        try:
            self.imageMain = data
        finally:
            pass

    def receive_image_left(self, data):
        try:
            self.imageLeft = data
        finally:
            pass

    def receive_image_right(self, data):
        try:
            self.imageRight = data
        finally:
            pass

    def repaint_image(self):
        if self.imageMain is not None:
            try:
                qimageMain = QtGui.QImage.fromData(self.imageMain.data)
                imageMain = QtGui.QPixmap.fromImage(qimageMain)
                #imageMain = imageMain.scaled(QtCore.QSize(self.ui.camera1.width()-1, self.ui.camera1.height()-1),0)

                if self.ui.rot0.isChecked():
                    self.ui.camera1.setPixmap(imageMain)
                elif self.ui.rot90.isChecked():
                    rotated = imageMain.transformed(QtGui.QMatrix().rotate(90), QtCore.Qt.SmoothTransformation)
                    self.ui.camera1.setPixmap(rotated)
                elif self.ui.rot180.isChecked():
                    rotated = imageMain.transformed(QtGui.QMatrix().rotate(180), QtCore.Qt.SmoothTransformation)
                    self.ui.camera1.setPixmap(rotated)
                elif self.ui.rot270.isChecked():
                    rotated = imageMain.transformed(QtGui.QMatrix().rotate(270), QtCore.Qt.SmoothTransformation)
                    self.ui.camera1.setPixmap(rotated)

            finally:
                pass
        else:
            self.ui.camera1.setText("no video feed")

        if self.imageLeft is not None:
            try:
                qimageTop = QtGui.QImage.fromData(self.imageLeft.data)
                imageTop = QtGui.QPixmap.fromImage(qimageTop)
                rotated = imageTop.transformed(QtGui.QMatrix().rotate(-90), QtCore.Qt.SmoothTransformation)
                #transformed(QtGui.QTransform().scale(-1,1))  # mirror on the y axis

            finally:
                pass
            self.ui.camera2.setPixmap(rotated)
        else:
            self.ui.camera2.setText("no video feed")

        if self.imageRight is not None:
            try:
                qimageBottom = QtGui.QImage.fromData(self.imageRight.data)
                imageBottom = QtGui.QPixmap.fromImage(qimageBottom)
                rotated = imageBottom.transformed(QtGui.QMatrix().rotate(90), QtCore.Qt.SmoothTransformation)
            finally:
                pass
            self.ui.camera3.setPixmap(rotated)
        else:
            self.ui.camera3.setText("no video feed")


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())
