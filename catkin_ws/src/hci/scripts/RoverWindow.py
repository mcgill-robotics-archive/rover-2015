# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MainWindow_V4.ui'
#
# Created: Wed May 27 22:50:45 2015
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
        MainWindow.resize(907, 669)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout_8 = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_8.setObjectName(_fromUtf8("horizontalLayout_8"))
        self.verticalLayout_3 = QtGui.QVBoxLayout()
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.MinimapLabel = QtGui.QLabel(self.centralwidget)
        self.MinimapLabel.setObjectName(_fromUtf8("MinimapLabel"))
        self.verticalLayout_3.addWidget(self.MinimapLabel)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.pushButton = QtGui.QPushButton(self.centralwidget)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.horizontalLayout.addWidget(self.pushButton)
        self.pushButton_2 = QtGui.QPushButton(self.centralwidget)
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
        self.horizontalLayout.addWidget(self.pushButton_2)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        self.horizontalLayout_5 = QtGui.QHBoxLayout()
        self.horizontalLayout_5.setObjectName(_fromUtf8("horizontalLayout_5"))
        self.label_6 = QtGui.QLabel(self.centralwidget)
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.horizontalLayout_5.addWidget(self.label_6)
        self.x = QtGui.QSpinBox(self.centralwidget)
        self.x.setObjectName(_fromUtf8("x"))
        self.horizontalLayout_5.addWidget(self.x)
        self.label_7 = QtGui.QLabel(self.centralwidget)
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.horizontalLayout_5.addWidget(self.label_7)
        self.y = QtGui.QSpinBox(self.centralwidget)
        self.y.setObjectName(_fromUtf8("y"))
        self.horizontalLayout_5.addWidget(self.y)
        self.addMarkedWaypoint = QtGui.QPushButton(self.centralwidget)
        self.addMarkedWaypoint.setObjectName(_fromUtf8("addMarkedWaypoint"))
        self.horizontalLayout_5.addWidget(self.addMarkedWaypoint)
        self.verticalLayout_3.addLayout(self.horizontalLayout_5)
        self.graphicsView = GraphicsLayoutWidget(self.centralwidget)
        self.graphicsView.setObjectName(_fromUtf8("graphicsView"))
        self.verticalLayout_3.addWidget(self.graphicsView)
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setObjectName(_fromUtf8("label"))
        self.verticalLayout_3.addWidget(self.label)
        self.horizontalLayout_7 = QtGui.QHBoxLayout()
        self.horizontalLayout_7.setObjectName(_fromUtf8("horizontalLayout_7"))
        self.label_12 = QtGui.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_12.setFont(font)
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.horizontalLayout_7.addWidget(self.label_12)
        self.xActual = QtGui.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.xActual.setFont(font)
        self.xActual.setObjectName(_fromUtf8("xActual"))
        self.horizontalLayout_7.addWidget(self.xActual)
        self.label_15 = QtGui.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_15.setFont(font)
        self.label_15.setObjectName(_fromUtf8("label_15"))
        self.horizontalLayout_7.addWidget(self.label_15)
        self.yActual = QtGui.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.yActual.setFont(font)
        self.yActual.setObjectName(_fromUtf8("yActual"))
        self.horizontalLayout_7.addWidget(self.yActual)
        self.label_17 = QtGui.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_17.setFont(font)
        self.label_17.setObjectName(_fromUtf8("label_17"))
        self.horizontalLayout_7.addWidget(self.label_17)
        self.headingActual = QtGui.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.headingActual.setFont(font)
        self.headingActual.setObjectName(_fromUtf8("headingActual"))
        self.horizontalLayout_7.addWidget(self.headingActual)
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.label_5 = QtGui.QLabel(self.centralwidget)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.gridLayout.addWidget(self.label_5, 0, 0, 1, 1)
        self.Camera1Feed = QtGui.QComboBox(self.centralwidget)
        self.Camera1Feed.setMaximumSize(QtCore.QSize(16777215, 26))
        self.Camera1Feed.setObjectName(_fromUtf8("Camera1Feed"))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.gridLayout.addWidget(self.Camera1Feed, 1, 0, 1, 1)
        self.horizontalLayout_7.addLayout(self.gridLayout)
        self.verticalLayout_3.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_8.addLayout(self.verticalLayout_3)
        self.verticalLayout_4 = QtGui.QVBoxLayout()
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.OtherData = QtGui.QFrame(self.centralwidget)
        self.OtherData.setMinimumSize(QtCore.QSize(191, 161))
        self.OtherData.setFrameShape(QtGui.QFrame.NoFrame)
        self.OtherData.setFrameShadow(QtGui.QFrame.Raised)
        self.OtherData.setObjectName(_fromUtf8("OtherData"))
        self.StickRotation = QtGui.QDial(self.OtherData)
        self.StickRotation.setGeometry(QtCore.QRect(10, 10, 50, 64))
        self.StickRotation.setMinimum(-1000)
        self.StickRotation.setMaximum(1000)
        self.StickRotation.setObjectName(_fromUtf8("StickRotation"))
        self.MainY = QtGui.QSlider(self.OtherData)
        self.MainY.setGeometry(QtCore.QRect(70, 0, 22, 160))
        self.MainY.setMinimum(-1000)
        self.MainY.setMaximum(1000)
        self.MainY.setOrientation(QtCore.Qt.Vertical)
        self.MainY.setObjectName(_fromUtf8("MainY"))
        self.MainX = QtGui.QSlider(self.OtherData)
        self.MainX.setGeometry(QtCore.QRect(0, 70, 160, 22))
        self.MainX.setMinimum(-1000)
        self.MainX.setMaximum(1000)
        self.MainX.setOrientation(QtCore.Qt.Horizontal)
        self.MainX.setInvertedAppearance(False)
        self.MainX.setObjectName(_fromUtf8("MainX"))
        self.SecondaryY = QtGui.QSlider(self.OtherData)
        self.SecondaryY.setGeometry(QtCore.QRect(170, 0, 22, 160))
        self.SecondaryY.setMinimum(-1000)
        self.SecondaryY.setMaximum(1000)
        self.SecondaryY.setOrientation(QtCore.Qt.Vertical)
        self.SecondaryY.setObjectName(_fromUtf8("SecondaryY"))
        self.verticalLayout_4.addWidget(self.OtherData)
        self.horizontalLayout_6 = QtGui.QHBoxLayout()
        self.horizontalLayout_6.setObjectName(_fromUtf8("horizontalLayout_6"))
        self.verticalLayout_5 = QtGui.QVBoxLayout()
        self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
        self.zoomGraph = QtGui.QCheckBox(self.centralwidget)
        self.zoomGraph.setChecked(True)
        self.zoomGraph.setObjectName(_fromUtf8("zoomGraph"))
        self.verticalLayout_5.addWidget(self.zoomGraph)
        self.ackMoving = QtGui.QCheckBox(self.centralwidget)
        self.ackMoving.setChecked(True)
        self.ackMoving.setObjectName(_fromUtf8("ackMoving"))
        self.verticalLayout_5.addWidget(self.ackMoving)
        self.horizontalLayout_6.addLayout(self.verticalLayout_5)
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.ackreman = QtGui.QRadioButton(self.centralwidget)
        self.ackreman.setChecked(True)
        self.ackreman.setObjectName(_fromUtf8("ackreman"))
        self.verticalLayout_2.addWidget(self.ackreman)
        self.pointSteer = QtGui.QRadioButton(self.centralwidget)
        self.pointSteer.setObjectName(_fromUtf8("pointSteer"))
        self.verticalLayout_2.addWidget(self.pointSteer)
        self.translatory = QtGui.QRadioButton(self.centralwidget)
        self.translatory.setObjectName(_fromUtf8("translatory"))
        self.verticalLayout_2.addWidget(self.translatory)
        self.horizontalLayout_6.addLayout(self.verticalLayout_2)
        self.verticalLayout_4.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label_4 = QtGui.QLabel(self.centralwidget)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.horizontalLayout_3.addWidget(self.label_4)
        self.sig_qual = QtGui.QLabel(self.centralwidget)
        self.sig_qual.setObjectName(_fromUtf8("sig_qual"))
        self.horizontalLayout_3.addWidget(self.sig_qual)
        self.verticalLayout_4.addLayout(self.horizontalLayout_3)
        self.label_3 = QtGui.QLabel(self.centralwidget)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.verticalLayout_4.addWidget(self.label_3)
        self.line = QtGui.QFrame(self.centralwidget)
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.verticalLayout_4.addWidget(self.line)
        self.gridLayout_3 = QtGui.QGridLayout()
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        self.DriveMode = QtGui.QPushButton(self.centralwidget)
        self.DriveMode.setCheckable(True)
        self.DriveMode.setObjectName(_fromUtf8("DriveMode"))
        self.gridLayout_3.addWidget(self.DriveMode, 0, 0, 1, 1)
        self.EndEffectorMode = QtGui.QPushButton(self.centralwidget)
        self.EndEffectorMode.setCheckable(True)
        self.EndEffectorMode.setChecked(False)
        self.EndEffectorMode.setObjectName(_fromUtf8("EndEffectorMode"))
        self.gridLayout_3.addWidget(self.EndEffectorMode, 0, 1, 1, 1)
        self.ArmBaseMode = QtGui.QPushButton(self.centralwidget)
        self.ArmBaseMode.setCheckable(True)
        self.ArmBaseMode.setChecked(False)
        self.ArmBaseMode.setObjectName(_fromUtf8("ArmBaseMode"))
        self.gridLayout_3.addWidget(self.ArmBaseMode, 1, 0, 1, 1)
        self.function4 = QtGui.QPushButton(self.centralwidget)
        self.function4.setCheckable(True)
        self.function4.setChecked(False)
        self.function4.setObjectName(_fromUtf8("function4"))
        self.gridLayout_3.addWidget(self.function4, 1, 1, 1, 1)
        self.verticalLayout_4.addLayout(self.gridLayout_3)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.gridLayout_2 = QtGui.QGridLayout()
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.armSetX = QtGui.QDoubleSpinBox(self.centralwidget)
        self.armSetX.setObjectName(_fromUtf8("armSetX"))
        self.gridLayout_2.addWidget(self.armSetX, 1, 0, 1, 1)
        self.armSetY = QtGui.QDoubleSpinBox(self.centralwidget)
        self.armSetY.setObjectName(_fromUtf8("armSetY"))
        self.gridLayout_2.addWidget(self.armSetY, 1, 1, 1, 1)
        self.armSetZ = QtGui.QDoubleSpinBox(self.centralwidget)
        self.armSetZ.setObjectName(_fromUtf8("armSetZ"))
        self.gridLayout_2.addWidget(self.armSetZ, 1, 2, 1, 1)
        self.label_9 = QtGui.QLabel(self.centralwidget)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.gridLayout_2.addWidget(self.label_9, 0, 0, 1, 1)
        self.label_10 = QtGui.QLabel(self.centralwidget)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.gridLayout_2.addWidget(self.label_10, 0, 1, 1, 1)
        self.label_11 = QtGui.QLabel(self.centralwidget)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.gridLayout_2.addWidget(self.label_11, 0, 2, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_2)
        self.sendCoords = QtGui.QPushButton(self.centralwidget)
        self.sendCoords.setObjectName(_fromUtf8("sendCoords"))
        self.verticalLayout.addWidget(self.sendCoords)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setMaximumSize(QtCore.QSize(67, 16777215))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_2.addWidget(self.label_2)
        self.arm_mode = QtGui.QComboBox(self.centralwidget)
        self.arm_mode.setObjectName(_fromUtf8("arm_mode"))
        self.arm_mode.addItem(_fromUtf8(""))
        self.arm_mode.addItem(_fromUtf8(""))
        self.horizontalLayout_2.addWidget(self.arm_mode)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.label_8 = QtGui.QLabel(self.centralwidget)
        self.label_8.setMaximumSize(QtCore.QSize(71, 16777215))
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.horizontalLayout_4.addWidget(self.label_8)
        self.coordinateSystem = QtGui.QComboBox(self.centralwidget)
        self.coordinateSystem.setObjectName(_fromUtf8("coordinateSystem"))
        self.coordinateSystem.addItem(_fromUtf8(""))
        self.coordinateSystem.addItem(_fromUtf8(""))
        self.horizontalLayout_4.addWidget(self.coordinateSystem)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.verticalLayout_4.addLayout(self.verticalLayout)
        self.screenshot = QtGui.QPushButton(self.centralwidget)
        self.screenshot.setObjectName(_fromUtf8("screenshot"))
        self.verticalLayout_4.addWidget(self.screenshot)
        self.horizontalLayout_8.addLayout(self.verticalLayout_4)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 907, 22))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QObject.connect(self.armSetX, QtCore.SIGNAL(_fromUtf8("editingFinished()")), self.sendCoords.click)
        QtCore.QObject.connect(self.armSetY, QtCore.SIGNAL(_fromUtf8("editingFinished()")), self.sendCoords.click)
        QtCore.QObject.connect(self.armSetZ, QtCore.SIGNAL(_fromUtf8("editingFinished()")), self.sendCoords.click)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.MinimapLabel.setText(_translate("MainWindow", "MIniMap", None))
        self.pushButton.setText(_translate("MainWindow", "Add Waypoint", None))
        self.pushButton_2.setText(_translate("MainWindow", "Clear", None))
        self.label_6.setText(_translate("MainWindow", "X", None))
        self.label_7.setText(_translate("MainWindow", "Y", None))
        self.addMarkedWaypoint.setText(_translate("MainWindow", "Add Waypoint", None))
        self.label.setText(_translate("MainWindow", "Current position", None))
        self.label_12.setText(_translate("MainWindow", "X", None))
        self.xActual.setText(_translate("MainWindow", "0.00", None))
        self.label_15.setText(_translate("MainWindow", "Y", None))
        self.yActual.setText(_translate("MainWindow", "0.00", None))
        self.label_17.setText(_translate("MainWindow", "Heading", None))
        self.headingActual.setText(_translate("MainWindow", "0.00", None))
        self.label_5.setText(_translate("MainWindow", "Main Screen", None))
        self.Camera1Feed.setItemText(0, _translate("MainWindow", "Front Hazcam", None))
        self.Camera1Feed.setItemText(1, _translate("MainWindow", "Left Hazcam", None))
        self.Camera1Feed.setItemText(2, _translate("MainWindow", "Right Hazcam", None))
        self.Camera1Feed.setItemText(3, _translate("MainWindow", "Back Hazcam", None))
        self.Camera1Feed.setItemText(4, _translate("MainWindow", "Arm", None))
        self.Camera1Feed.setItemText(5, _translate("MainWindow", "Pan/Tilt", None))
        self.zoomGraph.setText(_translate("MainWindow", "Zoom graph", None))
        self.ackMoving.setText(_translate("MainWindow", "Ackreman Moving", None))
        self.ackreman.setText(_translate("MainWindow", "Ackreman", None))
        self.pointSteer.setText(_translate("MainWindow", "Point steer ?", None))
        self.translatory.setText(_translate("MainWindow", "Translatory", None))
        self.label_4.setText(_translate("MainWindow", "Signal quality", None))
        self.sig_qual.setText(_translate("MainWindow", "123 ms", None))
        self.label_3.setText(_translate("MainWindow", "Control Layout", None))
        self.DriveMode.setText(_translate("MainWindow", "Drive / Camera", None))
        self.EndEffectorMode.setText(_translate("MainWindow", "End Effector", None))
        self.ArmBaseMode.setText(_translate("MainWindow", "Arm Base", None))
        self.function4.setText(_translate("MainWindow", "function 4", None))
        self.label_9.setText(_translate("MainWindow", "X", None))
        self.label_10.setText(_translate("MainWindow", "Y", None))
        self.label_11.setText(_translate("MainWindow", "Z", None))
        self.sendCoords.setText(_translate("MainWindow", "Send", None))
        self.label_2.setText(_translate("MainWindow", "Arm mode", None))
        self.arm_mode.setItemText(0, _translate("MainWindow", "Position Control", None))
        self.arm_mode.setItemText(1, _translate("MainWindow", "Velocity Control", None))
        self.label_8.setText(_translate("MainWindow", "Coordinate", None))
        self.coordinateSystem.setItemText(0, _translate("MainWindow", "Cylindrical", None))
        self.coordinateSystem.setItemText(1, _translate("MainWindow", "Cartesian", None))
        self.screenshot.setText(_translate("MainWindow", "Capture Arm cam", None))

from pyqtgraph import GraphicsLayoutWidget
