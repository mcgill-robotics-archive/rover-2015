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
import math
import numpy

from std_msgs.msg import *
from joystick_profile import ProfileParser
from sensor_msgs.msg import Image
from rover_msgs.msg import MotorControllerMode, MotorStatus, AhrsStatusMessage
from rover_msgs.srv import ResetWatchDog


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


def formatAngle(angle):

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


        self.sub = None

        self.modeId = 0
        self.grip = 0

        # map place holders
        self.tempPose = Queue.Queue()
        self.new_x = []
        self.new_y = []
        self.w1 = None
        self.s1 = None
        self.s2 = None
        self.x_waypoints = []
        self.y_waypoints = []

        # self.cam_x = 0
        # self.cam_y = 0

        self.first_point = False
        self.dx = 0
        self.dy = 0

        self.init_ros()
        self.init_connects()
        self.init_timers()
        self.setup_minimap()

        rospy.loginfo("HCI initialization completed")

    def init_ros(self):
        rospy.init_node('listener', anonymous=False)
        rospy.Subscriber('ahrs_status', AhrsStatusMessage, self.handle_pose, queue_size=10)
        rospy.Subscriber('/motor_status', MotorStatus, self.motor_status, queue_size=10)

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
        QtCore.QObject.connect(self.ui.DriveMode, QtCore.SIGNAL("clicked()"), lambda index = 0: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.ArmBaseMode, QtCore.SIGNAL("clicked()"), lambda index = 1: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.EndEffectorMode, QtCore.SIGNAL("clicked()"), lambda index = 2: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.function4, QtCore.SIGNAL("clicked()"), lambda index = 3: self.set_controller_mode(index))
        QtCore.QObject.connect(self.ui.screenshot, QtCore.SIGNAL("clicked()"), self.take_screenshot)
        QtCore.QObject.connect(self.ui.pointSteer, QtCore.SIGNAL("toggled(bool)"), self.set_point_steer)
        QtCore.QObject.connect(self.ui.ackreman, QtCore.SIGNAL("toggled(bool)"), self.set_ackreman)
        QtCore.QObject.connect(self.ui.skid, QtCore.SIGNAL("toggled(bool)"), self.set_skid)
        QtCore.QObject.connect(self.ui.translatory, QtCore.SIGNAL("toggled(bool)"), self.set_translatory)
        QtCore.QObject.connect(self.ui.addMarkedWaypoint, QtCore.SIGNAL("clicked()"), self.addCoord)

        # camera feed selection signal connects
        QtCore.QObject.connect(self.ui.clearMap, QtCore.SIGNAL("clicked()"), self.add_way_point)
        QtCore.QObject.connect(self.ui.clearMap, QtCore.SIGNAL("clicked()"), self.clear_map)
        QtCore.QObject.connect(self.ui.driveModeSelection, QtCore.SIGNAL("currentIndexChanged(int)"), self.set_motor_controller_mode)

        #motor readys
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

    def setup_minimap(self):

        self.w1 = self.ui.graphicsView.addViewBox()
        self.w1.setAspectLocked(False)
        self.w1.enableAutoRange('xy', True)
        self.ui.graphicsView.nextRow()

        self.x_waypoints.append(0)
        self.y_waypoints.append(0)

        self.s1 = pg.ScatterPlotItem(size=10, pen=pg.mkPen('w'), pxMode=True)
        self.s1.addPoints(self.x_waypoints, self.y_waypoints, size=10, symbol='t', brush='b')
        self.s2 = pg.ScatterPlotItem(size=10, pen=pg.mkPen('r'), pxMode=True)
        self.w1.addItem(self.s1)
        self.w1.addItem(self.s2)

    def handle_pose(self, data):
        if not self.first_point:
            self.first_point = True
            self.dy = data.gpsLongitude
            self.dx = data.gpsLatitude
        # add (x,y) to tempPose queue
        self.tempPose.put(data)

    def add_point_timeout(self):
        while not self.tempPose.empty():
            pose = self.tempPose.get()

            self.new_x = [(pose.gpsLatitude - self.dx)/10000000.0]
            self.new_y = [(pose.gpsLongitude - self.dy)/10000000.0]
            self.ui.xActual.setText(str(pose.gpsLatitude/10000000.0))
            self.ui.yActual.setText(str(pose.gpsLongitude/10000000.0))

            self.ui.pitchLBL.setText(formatAngle(pose.pitch))
            self.ui.rollLBL.setText(formatAngle(pose.roll))
            self.ui.yawLBL.setText(formatAngle(pose.yaw))

            self.points_counter += 1
            #
            # if self.points_counter % 10 == 0:
            #     graphData = self.s2.getData()
            #     # print graphData[0]
            #     # print graphData[0][:-10]
            #     lastXmean = numpy.mean(graphData[0][-10:])
            #     newDataX = graphData[0][:-10].tolist()
            #     newDataX.append(lastXmean)
            #     # print newDataX
            #     # print graphData[1]
            #     # print graphData[1][:-10]
            #     lastYmean = numpy.mean(graphData[1][-10:])
            #     newDataY = graphData[1][:-10].tolist()
            #     newDataY.append(lastYmean)
            #
            #     # print(lastXmean)
            #     # print(lastYmean)
            #
            #     self.s1.setData(newDataX, newDataY, size=3, symbol='o', brush='r')

            self.s1.addPoints(self.new_x, self.new_y, size=3, symbol='o', brush='w')
            if self.ui.zoomGraph.isChecked():
                self.w1.autoRange()

    def add_way_point(self):
        self.x_waypoints.append(self.new_x[0])
        self.y_waypoints.append(self.new_y[0])
        self.s1.addPoints([self.new_x[0]], [self.new_y[0]], size=10, symbol='t', brush='b')
        if self.ui.zoomGraph.isChecked():
            self.w1.autoRange()

    def addCoord(self):
        x = self.ui.x.value() - self.dx
        y = self.ui.y.value() - self.dy
        self.x_waypoints.append(x)
        self.y_waypoints.append(y)
        self.s1.addPoints([x], [y], size=10, symbol='t', brush='b')
        if self.ui.zoomGraph.isChecked():
            self.w1.autoRange()

    def clear_map(self):
        self.first_point = False
        self.dx = 0
        self.dy = 0
        
        self.s1.setData([], [], size=10, symbol='o', brush='r')
        self.s1.addPoints(self.x_waypoints, self.y_waypoints, size=10, symbol='t', brush='b')

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

        self.controller.clear_buttons()
        self.publish_controls()

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


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())
