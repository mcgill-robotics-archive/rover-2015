# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MainWindow_V3.ui'
#
# Created: Mon Mar  9 21:29:52 2015
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(812, 692)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayout_10 = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout_10.setObjectName(_fromUtf8("verticalLayout_10"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.Minimap = QtGui.QFrame(self.centralwidget)
        self.Minimap.setFrameShape(QtGui.QFrame.NoFrame)
        self.Minimap.setFrameShadow(QtGui.QFrame.Raised)
        self.Minimap.setObjectName(_fromUtf8("Minimap"))
        self.MinimapLabel = QtGui.QLabel(self.Minimap)
        self.MinimapLabel.setGeometry(QtCore.QRect(82, 0, 71, 20))
        self.MinimapLabel.setObjectName(_fromUtf8("MinimapLabel"))
        self.graphicsView = GraphicsLayoutWidget(self.Minimap)
        self.graphicsView.setGeometry(QtCore.QRect(10, 81, 221, 281))
        self.graphicsView.setObjectName(_fromUtf8("graphicsView"))
        self.pushButton = QtGui.QPushButton(self.Minimap)
        self.pushButton.setGeometry(QtCore.QRect(10, 30, 121, 27))
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.pushButton_2 = QtGui.QPushButton(self.Minimap)
        self.pushButton_2.setGeometry(QtCore.QRect(140, 30, 81, 27))
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout_2.addWidget(self.Minimap)
        self.line_5 = QtGui.QFrame(self.centralwidget)
        self.line_5.setFrameShape(QtGui.QFrame.VLine)
        self.line_5.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_5.setObjectName(_fromUtf8("line_5"))
        self.horizontalLayout_2.addWidget(self.line_5)
        self.selectorFrame = QtGui.QFrame(self.centralwidget)
        self.selectorFrame.setMaximumSize(QtCore.QSize(120, 16777215))
        self.selectorFrame.setObjectName(_fromUtf8("selectorFrame"))
        self.verticalLayout_8 = QtGui.QVBoxLayout(self.selectorFrame)
        self.verticalLayout_8.setObjectName(_fromUtf8("verticalLayout_8"))
        self.HandTypeBox = QtGui.QFrame(self.selectorFrame)
        self.HandTypeBox.setMaximumSize(QtCore.QSize(16777215, 124))
        self.HandTypeBox.setFrameShape(QtGui.QFrame.NoFrame)
        self.HandTypeBox.setFrameShadow(QtGui.QFrame.Plain)
        self.HandTypeBox.setObjectName(_fromUtf8("HandTypeBox"))
        self.verticalLayout = QtGui.QVBoxLayout(self.HandTypeBox)
        self.verticalLayout.setSpacing(10)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.label = QtGui.QLabel(self.HandTypeBox)
        self.label.setObjectName(_fromUtf8("label"))
        self.verticalLayout.addWidget(self.label)
        self.HandTypeChoice = QtGui.QComboBox(self.HandTypeBox)
        self.HandTypeChoice.setObjectName(_fromUtf8("HandTypeChoice"))
        self.HandTypeChoice.addItem(_fromUtf8(""))
        self.HandTypeChoice.addItem(_fromUtf8(""))
        self.verticalLayout.addWidget(self.HandTypeChoice)
        self.label_2 = QtGui.QLabel(self.HandTypeBox)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.verticalLayout.addWidget(self.label_2)
        self.ArmModeChoice = QtGui.QComboBox(self.HandTypeBox)
        self.ArmModeChoice.setObjectName(_fromUtf8("ArmModeChoice"))
        self.verticalLayout.addWidget(self.ArmModeChoice)
        self.verticalLayout_8.addWidget(self.HandTypeBox)
        self.line_2 = QtGui.QFrame(self.selectorFrame)
        self.line_2.setFrameShape(QtGui.QFrame.HLine)
        self.line_2.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_2.setObjectName(_fromUtf8("line_2"))
        self.verticalLayout_8.addWidget(self.line_2)
        self.frame_8 = QtGui.QFrame(self.selectorFrame)
        self.frame_8.setMaximumSize(QtCore.QSize(16777215, 111))
        self.frame_8.setFrameShape(QtGui.QFrame.NoFrame)
        self.frame_8.setFrameShadow(QtGui.QFrame.Plain)
        self.frame_8.setObjectName(_fromUtf8("frame_8"))
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.frame_8)
        self.verticalLayout_3.setSpacing(10)
        self.verticalLayout_3.setMargin(0)
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.video_quality = QtGui.QLabel(self.frame_8)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.video_quality.sizePolicy().hasHeightForWidth())
        self.video_quality.setSizePolicy(sizePolicy)
        self.video_quality.setObjectName(_fromUtf8("video_quality"))
        self.verticalLayout_3.addWidget(self.video_quality)
        self.videoQualityChoice = QtGui.QComboBox(self.frame_8)
        self.videoQualityChoice.setObjectName(_fromUtf8("videoQualityChoice"))
        self.videoQualityChoice.addItem(_fromUtf8(""))
        self.videoQualityChoice.addItem(_fromUtf8(""))
        self.videoQualityChoice.addItem(_fromUtf8(""))
        self.verticalLayout_3.addWidget(self.videoQualityChoice)
        self.comboBox_7 = QtGui.QComboBox(self.frame_8)
        self.comboBox_7.setObjectName(_fromUtf8("comboBox_7"))
        self.verticalLayout_3.addWidget(self.comboBox_7)
        self.verticalLayout_8.addWidget(self.frame_8)
        self.horizontalLayout_2.addWidget(self.selectorFrame)
        self.line_4 = QtGui.QFrame(self.centralwidget)
        self.line_4.setFrameShape(QtGui.QFrame.VLine)
        self.line_4.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_4.setObjectName(_fromUtf8("line_4"))
        self.horizontalLayout_2.addWidget(self.line_4)
        self.buttonFrame = QtGui.QFrame(self.centralwidget)
        self.buttonFrame.setMaximumSize(QtCore.QSize(120, 16777215))
        self.buttonFrame.setObjectName(_fromUtf8("buttonFrame"))
        self.verticalLayout_9 = QtGui.QVBoxLayout(self.buttonFrame)
        self.verticalLayout_9.setObjectName(_fromUtf8("verticalLayout_9"))
        self.functionBox = QtGui.QFrame(self.buttonFrame)
        self.functionBox.setMaximumSize(QtCore.QSize(16777215, 155))
        self.functionBox.setFrameShape(QtGui.QFrame.NoFrame)
        self.functionBox.setFrameShadow(QtGui.QFrame.Plain)
        self.functionBox.setObjectName(_fromUtf8("functionBox"))
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.functionBox)
        self.verticalLayout_2.setSpacing(10)
        self.verticalLayout_2.setMargin(0)
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.label_3 = QtGui.QLabel(self.functionBox)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.verticalLayout_2.addWidget(self.label_3)
        self.line = QtGui.QFrame(self.functionBox)
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.verticalLayout_2.addWidget(self.line)
        self.DriveMode = QtGui.QPushButton(self.functionBox)
        self.DriveMode.setCheckable(True)
        self.DriveMode.setObjectName(_fromUtf8("DriveMode"))
        self.verticalLayout_2.addWidget(self.DriveMode)
        self.ArmBaseMode = QtGui.QPushButton(self.functionBox)
        self.ArmBaseMode.setCheckable(True)
        self.ArmBaseMode.setChecked(False)
        self.ArmBaseMode.setObjectName(_fromUtf8("ArmBaseMode"))
        self.verticalLayout_2.addWidget(self.ArmBaseMode)
        self.EndEffectorMode = QtGui.QPushButton(self.functionBox)
        self.EndEffectorMode.setCheckable(True)
        self.EndEffectorMode.setChecked(False)
        self.EndEffectorMode.setObjectName(_fromUtf8("EndEffectorMode"))
        self.verticalLayout_2.addWidget(self.EndEffectorMode)
        self.function4 = QtGui.QPushButton(self.functionBox)
        self.function4.setCheckable(True)
        self.function4.setChecked(False)
        self.function4.setObjectName(_fromUtf8("function4"))
        self.verticalLayout_2.addWidget(self.function4)
        self.verticalLayout_9.addWidget(self.functionBox)
        self.pointSteer = QtGui.QCheckBox(self.buttonFrame)
        self.pointSteer.setObjectName(_fromUtf8("pointSteer"))
        self.verticalLayout_9.addWidget(self.pointSteer)
        self.line_3 = QtGui.QFrame(self.buttonFrame)
        self.line_3.setFrameShape(QtGui.QFrame.HLine)
        self.line_3.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_3.setObjectName(_fromUtf8("line_3"))
        self.verticalLayout_9.addWidget(self.line_3)
        self.frame_9 = QtGui.QFrame(self.buttonFrame)
        self.frame_9.setMaximumSize(QtCore.QSize(16777215, 80))
        self.frame_9.setFrameShape(QtGui.QFrame.NoFrame)
        self.frame_9.setFrameShadow(QtGui.QFrame.Plain)
        self.frame_9.setObjectName(_fromUtf8("frame_9"))
        self.verticalLayout_4 = QtGui.QVBoxLayout(self.frame_9)
        self.verticalLayout_4.setSpacing(10)
        self.verticalLayout_4.setMargin(0)
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.pushButton_5 = QtGui.QPushButton(self.frame_9)
        self.pushButton_5.setObjectName(_fromUtf8("pushButton_5"))
        self.verticalLayout_4.addWidget(self.pushButton_5)
        self.pushButton_6 = QtGui.QPushButton(self.frame_9)
        self.pushButton_6.setObjectName(_fromUtf8("pushButton_6"))
        self.verticalLayout_4.addWidget(self.pushButton_6)
        self.verticalLayout_9.addWidget(self.frame_9)
        self.horizontalLayout_2.addWidget(self.buttonFrame)
        self.line_6 = QtGui.QFrame(self.centralwidget)
        self.line_6.setFrameShape(QtGui.QFrame.VLine)
        self.line_6.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_6.setObjectName(_fromUtf8("line_6"))
        self.horizontalLayout_2.addWidget(self.line_6)
        self.OtherData = QtGui.QFrame(self.centralwidget)
        self.OtherData.setFrameShape(QtGui.QFrame.NoFrame)
        self.OtherData.setFrameShadow(QtGui.QFrame.Raised)
        self.OtherData.setObjectName(_fromUtf8("OtherData"))
        self.OtherDataLabel = QtGui.QLabel(self.OtherData)
        self.OtherDataLabel.setGeometry(QtCore.QRect(60, 0, 123, 17))
        self.OtherDataLabel.setObjectName(_fromUtf8("OtherDataLabel"))
        self.StickRotation = QtGui.QDial(self.OtherData)
        self.StickRotation.setGeometry(QtCore.QRect(20, 50, 50, 64))
        self.StickRotation.setMinimum(-1000)
        self.StickRotation.setMaximum(1000)
        self.StickRotation.setObjectName(_fromUtf8("StickRotation"))
        self.MainY = QtGui.QSlider(self.OtherData)
        self.MainY.setGeometry(QtCore.QRect(80, 40, 22, 160))
        self.MainY.setMinimum(-1000)
        self.MainY.setMaximum(1000)
        self.MainY.setOrientation(QtCore.Qt.Vertical)
        self.MainY.setObjectName(_fromUtf8("MainY"))
        self.MainX = QtGui.QSlider(self.OtherData)
        self.MainX.setGeometry(QtCore.QRect(10, 110, 160, 22))
        self.MainX.setMinimum(-1000)
        self.MainX.setMaximum(1000)
        self.MainX.setOrientation(QtCore.Qt.Horizontal)
        self.MainX.setInvertedAppearance(False)
        self.MainX.setObjectName(_fromUtf8("MainX"))
        self.SecondaryY = QtGui.QSlider(self.OtherData)
        self.SecondaryY.setGeometry(QtCore.QRect(180, 40, 22, 160))
        self.SecondaryY.setMinimum(-1000)
        self.SecondaryY.setMaximum(1000)
        self.SecondaryY.setOrientation(QtCore.Qt.Vertical)
        self.SecondaryY.setObjectName(_fromUtf8("SecondaryY"))
        self.layoutWidget = QtGui.QWidget(self.OtherData)
        self.layoutWidget.setGeometry(QtCore.QRect(40, 220, 149, 19))
        self.layoutWidget.setObjectName(_fromUtf8("layoutWidget"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout_3.setMargin(0)
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label_4 = QtGui.QLabel(self.layoutWidget)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.horizontalLayout_3.addWidget(self.label_4)
        self.sig_qual = QtGui.QLabel(self.layoutWidget)
        self.sig_qual.setObjectName(_fromUtf8("sig_qual"))
        self.horizontalLayout_3.addWidget(self.sig_qual)
        self.horizontalLayoutWidget = QtGui.QWidget(self.OtherData)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(10, 300, 201, 271))
        self.horizontalLayoutWidget.setObjectName(_fromUtf8("horizontalLayoutWidget"))
        self.verticalLayout_6 = QtGui.QVBoxLayout(self.horizontalLayoutWidget)
        self.verticalLayout_6.setMargin(0)
        self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
        self.Camera2Feed = QtGui.QComboBox(self.horizontalLayoutWidget)
        self.Camera2Feed.setMaximumSize(QtCore.QSize(16777215, 26))
        self.Camera2Feed.setObjectName(_fromUtf8("Camera2Feed"))
        self.Camera2Feed.addItem(_fromUtf8(""))
        self.Camera2Feed.addItem(_fromUtf8(""))
        self.Camera2Feed.addItem(_fromUtf8(""))
        self.Camera2Feed.addItem(_fromUtf8(""))
        self.Camera2Feed.addItem(_fromUtf8(""))
        self.Camera2Feed.addItem(_fromUtf8(""))
        self.verticalLayout_6.addWidget(self.Camera2Feed)
        self.Camera1Feed = QtGui.QComboBox(self.horizontalLayoutWidget)
        self.Camera1Feed.setMaximumSize(QtCore.QSize(16777215, 26))
        self.Camera1Feed.setObjectName(_fromUtf8("Camera1Feed"))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.verticalLayout_6.addWidget(self.Camera1Feed)
        self.Camera3Feed = QtGui.QComboBox(self.horizontalLayoutWidget)
        self.Camera3Feed.setMaximumSize(QtCore.QSize(16777215, 26))
        self.Camera3Feed.setObjectName(_fromUtf8("Camera3Feed"))
        self.Camera3Feed.addItem(_fromUtf8(""))
        self.Camera3Feed.addItem(_fromUtf8(""))
        self.Camera3Feed.addItem(_fromUtf8(""))
        self.Camera3Feed.addItem(_fromUtf8(""))
        self.Camera3Feed.addItem(_fromUtf8(""))
        self.Camera3Feed.addItem(_fromUtf8(""))
        self.verticalLayout_6.addWidget(self.Camera3Feed)
        self.horizontalLayout_2.addWidget(self.OtherData)
        self.verticalLayout_10.addLayout(self.horizontalLayout_2)
        self.line_7 = QtGui.QFrame(self.centralwidget)
        self.line_7.setFrameShape(QtGui.QFrame.HLine)
        self.line_7.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_7.setObjectName(_fromUtf8("line_7"))
        self.verticalLayout_10.addWidget(self.line_7)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 812, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.menuWindow = QtGui.QMenu(self.menubar)
        self.menuWindow.setObjectName(_fromUtf8("menuWindow"))
        MainWindow.setMenuBar(self.menubar)
        self.toolBar = QtGui.QToolBar(MainWindow)
        self.toolBar.setObjectName(_fromUtf8("toolBar"))
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)
        self.menubar.addAction(self.menuWindow.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.MinimapLabel.setText(_translate("MainWindow", "MIniMap", None))
        self.pushButton.setText(_translate("MainWindow", "Add Waypoint", None))
        self.pushButton_2.setText(_translate("MainWindow", "Clear", None))
        self.label.setText(_translate("MainWindow", "Hand type", None))
        self.HandTypeChoice.setItemText(0, _translate("MainWindow", "Manipulator", None))
        self.HandTypeChoice.setItemText(1, _translate("MainWindow", "Sampling", None))
        self.label_2.setText(_translate("MainWindow", "Arm mode", None))
        self.video_quality.setText(_translate("MainWindow", "video quality", None))
        self.videoQualityChoice.setItemText(0, _translate("MainWindow", "ultra", None))
        self.videoQualityChoice.setItemText(1, _translate("MainWindow", "high", None))
        self.videoQualityChoice.setItemText(2, _translate("MainWindow", "normal", None))
        self.label_3.setText(_translate("MainWindow", "Control Layout", None))
        self.DriveMode.setText(_translate("MainWindow", "Drive / Camera", None))
        self.ArmBaseMode.setText(_translate("MainWindow", "Arm Base", None))
        self.EndEffectorMode.setText(_translate("MainWindow", "End Effector", None))
        self.function4.setText(_translate("MainWindow", "function 4", None))
        self.pointSteer.setText(_translate("MainWindow", "Point steer", None))
        self.pushButton_5.setText(_translate("MainWindow", "PushButton", None))
        self.pushButton_6.setText(_translate("MainWindow", "PushButton", None))
        self.OtherDataLabel.setText(_translate("MainWindow", "display other data", None))
        self.label_4.setText(_translate("MainWindow", "Signal quality", None))
        self.sig_qual.setText(_translate("MainWindow", "123 ms", None))
        self.Camera2Feed.setItemText(0, _translate("MainWindow", "Front Hazcam", None))
        self.Camera2Feed.setItemText(1, _translate("MainWindow", "Left Hazcam", None))
        self.Camera2Feed.setItemText(2, _translate("MainWindow", "Right Hazcam", None))
        self.Camera2Feed.setItemText(3, _translate("MainWindow", "Back Hazcam", None))
        self.Camera2Feed.setItemText(4, _translate("MainWindow", "Arm", None))
        self.Camera2Feed.setItemText(5, _translate("MainWindow", "Pan/Tilt", None))
        self.Camera1Feed.setItemText(0, _translate("MainWindow", "Front Hazcam", None))
        self.Camera1Feed.setItemText(1, _translate("MainWindow", "Left Hazcam", None))
        self.Camera1Feed.setItemText(2, _translate("MainWindow", "Right Hazcam", None))
        self.Camera1Feed.setItemText(3, _translate("MainWindow", "Back Hazcam", None))
        self.Camera1Feed.setItemText(4, _translate("MainWindow", "Arm", None))
        self.Camera1Feed.setItemText(5, _translate("MainWindow", "Pan/Tilt", None))
        self.Camera3Feed.setItemText(0, _translate("MainWindow", "Front Hazcam", None))
        self.Camera3Feed.setItemText(1, _translate("MainWindow", "Left Hazcam", None))
        self.Camera3Feed.setItemText(2, _translate("MainWindow", "Right Hazcam", None))
        self.Camera3Feed.setItemText(3, _translate("MainWindow", "Back Hazcam", None))
        self.Camera3Feed.setItemText(4, _translate("MainWindow", "Arm", None))
        self.Camera3Feed.setItemText(5, _translate("MainWindow", "Pan/Tilt", None))
        self.menuWindow.setTitle(_translate("MainWindow", "window", None))
        self.toolBar.setWindowTitle(_translate("MainWindow", "toolBar", None))

from pyqtgraph import GraphicsLayoutWidget
