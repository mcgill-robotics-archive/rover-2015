#!/usr/bin/python

from VideoWindow import *
from PyQt4 import QtCore, QtGui

import sys

import rospy
from sensor_msgs.msg import CompressedImage


class CameraViewer(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(CameraViewer, self).__init__(parent)

        self.feedTopics = []
        self.imageMain = None
        self.imageTop = None
        self.imageBottom = None

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.signal = QtCore.QTimer(self)
        self.signal.timeout.connect(self.repaint_image)
        
        self.ui.camera1.setScaledContents(true)
        self.ui.camera2.setScaledContents(true)
        self.ui.camera3.setScaledContents(true)

        self.ros_init()

        self.signal.start(1000 / 60)

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
                qimageMain = QtGui.QImage.fromData(self.imageMain.data)
                imageMain = QtGui.QPixmap.fromImage(qimageMain)

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

        if self.imageTop is not None:
            try:
                qimageTop = QtGui.QImage.fromData(self.imageTop.data)
                imageTop = QtGui.QPixmap.fromImage(qimageTop)
                rotated = imageTop.transformed(Qtransform().scale(-1,1))  # mirror on the y axis
                                 #.transformed(QtGui.QMatrix().rotate(90), QtCore.Qt.SmoothTransformation)
            finally:
                pass
            self.ui.camera2.setPixmap(rotated)
        else:
            self.ui.camera2.setText("no video feed")

        if self.imageBottom is not None:
            try:
                qimageBottom = QtGui.QImage.fromData(self.imageBottom.data)
                imageBottom = QtGui.QPixmap.fromImage(qimageBottom)
                rotated = imageBottom.transformed(QtGui.QMatrix().rotate(90), QtCore.Qt.SmoothTransformation)
            finally:
                pass
            self.ui.camera3.setPixmap(rotated)
        else:
            self.ui.camera3.setText("no video feed")

    def ros_init(self):
        rospy.init_node('camera_viewer', anonymous=True)
        
        rospy.Subscriber("/front_haz/image_mono/compressed", CompressedImage, self.receiveimageMain)
        rospy.Subscriber("/econ/image_mono/compressed", CompressedImage, self.receiveimageTop)
        rospy.Subscriber("/right_nav/image_mono/compressed", CompressedImage, self.receiveimageBottom)

    # def getimageTopic(self):
    #     self.feedTopics.append(rospy.get_param("feed/topicMain", "/feed1/image_raw"))
    #     self.feedTopics.append(rospy.get_param("feed/topicTop", "/feed2/image_raw"))
    #     self.feedTopics.append(rospy.get_param("feed/topicBottom", "/feed3/image_raw"))
    #     rospy.loginfo(self.feedTopics)


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = CameraViewer()
    AppWindow.show()
    sys.exit(app.exec_())
