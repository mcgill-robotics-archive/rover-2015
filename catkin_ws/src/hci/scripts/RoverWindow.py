# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '../ui_files/MainWindow_V4.ui'
#
# Created: Tue Jul 21 12:26:47 2015
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
        MainWindow.resize(907, 683)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout_9 = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_9.setObjectName(_fromUtf8("horizontalLayout_9"))
        self.verticalLayout_3 = QtGui.QVBoxLayout()
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.MinimapLabel = QtGui.QLabel(self.centralwidget)
        self.MinimapLabel.setObjectName(_fromUtf8("MinimapLabel"))
        self.verticalLayout_3.addWidget(self.MinimapLabel)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.waypoint = QtGui.QPushButton(self.centralwidget)
        self.waypoint.setObjectName(_fromUtf8("waypoint"))
        self.horizontalLayout.addWidget(self.waypoint)
        self.clearMap = QtGui.QPushButton(self.centralwidget)
        self.clearMap.setObjectName(_fromUtf8("clearMap"))
        self.horizontalLayout.addWidget(self.clearMap)
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
        self.zoomGraph = QtGui.QCheckBox(self.centralwidget)
        self.zoomGraph.setChecked(True)
        self.zoomGraph.setObjectName(_fromUtf8("zoomGraph"))
        self.horizontalLayout_5.addWidget(self.zoomGraph)
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
        self.verticalLayout_3.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_9.addLayout(self.verticalLayout_3)
        self.verticalLayout_6 = QtGui.QVBoxLayout()
        self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
        self.horizontalLayout_6 = QtGui.QHBoxLayout()
        self.horizontalLayout_6.setObjectName(_fromUtf8("horizontalLayout_6"))
        self.verticalLayout_5 = QtGui.QVBoxLayout()
        self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
        self.ackMoving = QtGui.QCheckBox(self.centralwidget)
        self.ackMoving.setChecked(True)
        self.ackMoving.setObjectName(_fromUtf8("ackMoving"))
        self.verticalLayout_5.addWidget(self.ackMoving)
        self.screenshot = QtGui.QPushButton(self.centralwidget)
        self.screenshot.setObjectName(_fromUtf8("screenshot"))
        self.verticalLayout_5.addWidget(self.screenshot)
        self.horizontalLayout_6.addLayout(self.verticalLayout_5)
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.ackreman = QtGui.QRadioButton(self.centralwidget)
        self.ackreman.setChecked(False)
        self.ackreman.setObjectName(_fromUtf8("ackreman"))
        self.verticalLayout_2.addWidget(self.ackreman)
        self.skid = QtGui.QRadioButton(self.centralwidget)
        self.skid.setChecked(True)
        self.skid.setObjectName(_fromUtf8("skid"))
        self.verticalLayout_2.addWidget(self.skid)
        self.pointSteer = QtGui.QRadioButton(self.centralwidget)
        self.pointSteer.setObjectName(_fromUtf8("pointSteer"))
        self.verticalLayout_2.addWidget(self.pointSteer)
        self.translatory = QtGui.QRadioButton(self.centralwidget)
        self.translatory.setObjectName(_fromUtf8("translatory"))
        self.verticalLayout_2.addWidget(self.translatory)
        self.horizontalLayout_6.addLayout(self.verticalLayout_2)
        self.verticalLayout_6.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label_4 = QtGui.QLabel(self.centralwidget)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.horizontalLayout_3.addWidget(self.label_4)
        self.sig_qual = QtGui.QLabel(self.centralwidget)
        self.sig_qual.setObjectName(_fromUtf8("sig_qual"))
        self.horizontalLayout_3.addWidget(self.sig_qual)
        self.verticalLayout_6.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_8 = QtGui.QHBoxLayout()
        self.horizontalLayout_8.setObjectName(_fromUtf8("horizontalLayout_8"))
        self.label_5 = QtGui.QLabel(self.centralwidget)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.horizontalLayout_8.addWidget(self.label_5)
        self.driveModeSelection = QtGui.QComboBox(self.centralwidget)
        self.driveModeSelection.setObjectName(_fromUtf8("driveModeSelection"))
        self.driveModeSelection.addItem(_fromUtf8(""))
        self.driveModeSelection.addItem(_fromUtf8(""))
        self.driveModeSelection.addItem(_fromUtf8(""))
        self.driveModeSelection.addItem(_fromUtf8(""))
        self.horizontalLayout_8.addWidget(self.driveModeSelection)
        self.verticalLayout_6.addLayout(self.horizontalLayout_8)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_6.addItem(spacerItem)
        self.frame = QtGui.QFrame(self.centralwidget)
        self.frame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtGui.QFrame.Raised)
        self.frame.setObjectName(_fromUtf8("frame"))
        self.verticalLayout_4 = QtGui.QVBoxLayout(self.frame)
        self.verticalLayout_4.setSpacing(0)
        self.verticalLayout_4.setMargin(0)
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.label_3 = QtGui.QLabel(self.frame)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.verticalLayout_4.addWidget(self.label_3)
        self.line = QtGui.QFrame(self.frame)
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.verticalLayout_4.addWidget(self.line)
        self.gridLayout_3 = QtGui.QGridLayout()
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        self.DriveMode = QtGui.QPushButton(self.frame)
        self.DriveMode.setCheckable(True)
        self.DriveMode.setObjectName(_fromUtf8("DriveMode"))
        self.gridLayout_3.addWidget(self.DriveMode, 0, 0, 1, 1)
        self.EndEffectorMode = QtGui.QPushButton(self.frame)
        self.EndEffectorMode.setCheckable(True)
        self.EndEffectorMode.setChecked(False)
        self.EndEffectorMode.setObjectName(_fromUtf8("EndEffectorMode"))
        self.gridLayout_3.addWidget(self.EndEffectorMode, 0, 1, 1, 1)
        self.ArmBaseMode = QtGui.QPushButton(self.frame)
        self.ArmBaseMode.setCheckable(True)
        self.ArmBaseMode.setChecked(False)
        self.ArmBaseMode.setObjectName(_fromUtf8("ArmBaseMode"))
        self.gridLayout_3.addWidget(self.ArmBaseMode, 1, 0, 1, 1)
        self.function4 = QtGui.QPushButton(self.frame)
        self.function4.setCheckable(True)
        self.function4.setChecked(False)
        self.function4.setObjectName(_fromUtf8("function4"))
        self.gridLayout_3.addWidget(self.function4, 1, 1, 1, 1)
        self.verticalLayout_4.addLayout(self.gridLayout_3)
        self.verticalLayout_6.addWidget(self.frame)
        self.frame_2 = QtGui.QFrame(self.centralwidget)
        self.frame_2.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_2.setObjectName(_fromUtf8("frame_2"))
        self.verticalLayout = QtGui.QVBoxLayout(self.frame_2)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label_2 = QtGui.QLabel(self.frame_2)
        self.label_2.setMaximumSize(QtCore.QSize(67, 16777215))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.horizontalLayout_2.addWidget(self.label_2)
        self.arm_mode = QtGui.QComboBox(self.frame_2)
        self.arm_mode.setObjectName(_fromUtf8("arm_mode"))
        self.arm_mode.addItem(_fromUtf8(""))
        self.arm_mode.addItem(_fromUtf8(""))
        self.horizontalLayout_2.addWidget(self.arm_mode)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.label_8 = QtGui.QLabel(self.frame_2)
        self.label_8.setMaximumSize(QtCore.QSize(71, 16777215))
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.horizontalLayout_4.addWidget(self.label_8)
        self.coordinateSystem = QtGui.QComboBox(self.frame_2)
        self.coordinateSystem.setObjectName(_fromUtf8("coordinateSystem"))
        self.coordinateSystem.addItem(_fromUtf8(""))
        self.coordinateSystem.addItem(_fromUtf8(""))
        self.horizontalLayout_4.addWidget(self.coordinateSystem)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.line_2 = QtGui.QFrame(self.frame_2)
        self.line_2.setFrameShape(QtGui.QFrame.HLine)
        self.line_2.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_2.setObjectName(_fromUtf8("line_2"))
        self.verticalLayout.addWidget(self.line_2)
        self.gridLayout_2 = QtGui.QGridLayout()
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.armSetX = QtGui.QDoubleSpinBox(self.frame_2)
        self.armSetX.setObjectName(_fromUtf8("armSetX"))
        self.gridLayout_2.addWidget(self.armSetX, 1, 0, 1, 1)
        self.armSetY = QtGui.QDoubleSpinBox(self.frame_2)
        self.armSetY.setObjectName(_fromUtf8("armSetY"))
        self.gridLayout_2.addWidget(self.armSetY, 1, 1, 1, 1)
        self.armSetZ = QtGui.QDoubleSpinBox(self.frame_2)
        self.armSetZ.setObjectName(_fromUtf8("armSetZ"))
        self.gridLayout_2.addWidget(self.armSetZ, 1, 2, 1, 1)
        self.label_9 = QtGui.QLabel(self.frame_2)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.gridLayout_2.addWidget(self.label_9, 0, 0, 1, 1)
        self.label_10 = QtGui.QLabel(self.frame_2)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.gridLayout_2.addWidget(self.label_10, 0, 1, 1, 1)
        self.label_11 = QtGui.QLabel(self.frame_2)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.gridLayout_2.addWidget(self.label_11, 0, 2, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout_2)
        self.sendCoords = QtGui.QPushButton(self.frame_2)
        self.sendCoords.setObjectName(_fromUtf8("sendCoords"))
        self.verticalLayout.addWidget(self.sendCoords)
        self.verticalLayout_6.addWidget(self.frame_2)
        self.horizontalLayout_9.addLayout(self.verticalLayout_6)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 907, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.menuFile = QtGui.QMenu(self.menubar)
        self.menuFile.setObjectName(_fromUtf8("menuFile"))
        MainWindow.setMenuBar(self.menubar)
        self.actionQuit = QtGui.QAction(MainWindow)
        self.actionQuit.setObjectName(_fromUtf8("actionQuit"))
        self.actionRestart = QtGui.QAction(MainWindow)
        self.actionRestart.setObjectName(_fromUtf8("actionRestart"))
        self.menuFile.addAction(self.actionRestart)
        self.menuFile.addAction(self.actionQuit)
        self.menubar.addAction(self.menuFile.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QObject.connect(self.armSetX, QtCore.SIGNAL(_fromUtf8("editingFinished()")), self.sendCoords.click)
        QtCore.QObject.connect(self.armSetY, QtCore.SIGNAL(_fromUtf8("editingFinished()")), self.sendCoords.click)
        QtCore.QObject.connect(self.armSetZ, QtCore.SIGNAL(_fromUtf8("editingFinished()")), self.sendCoords.click)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.MinimapLabel.setText(_translate("MainWindow", "MIniMap", None))
        self.waypoint.setText(_translate("MainWindow", "Add Waypoint", None))
        self.clearMap.setText(_translate("MainWindow", "Clear", None))
        self.label_6.setText(_translate("MainWindow", "X", None))
        self.label_7.setText(_translate("MainWindow", "Y", None))
        self.addMarkedWaypoint.setText(_translate("MainWindow", "Add Waypoint", None))
        self.zoomGraph.setText(_translate("MainWindow", "Zoom graph", None))
        self.label.setText(_translate("MainWindow", "Current position", None))
        self.label_12.setText(_translate("MainWindow", "X", None))
        self.xActual.setText(_translate("MainWindow", "0.00", None))
        self.label_15.setText(_translate("MainWindow", "Y", None))
        self.yActual.setText(_translate("MainWindow", "0.00", None))
        self.label_17.setText(_translate("MainWindow", "Heading", None))
        self.headingActual.setText(_translate("MainWindow", "0.00", None))
        self.ackMoving.setText(_translate("MainWindow", "Motor Enable", None))
        self.screenshot.setText(_translate("MainWindow", "Capture Arm cam", None))
        self.ackreman.setText(_translate("MainWindow", "Ackreman", None))
        self.skid.setText(_translate("MainWindow", "Skid", None))
        self.pointSteer.setText(_translate("MainWindow", "Point steer ?", None))
        self.translatory.setText(_translate("MainWindow", "Translatory", None))
        self.label_4.setText(_translate("MainWindow", "Signal quality", None))
        self.sig_qual.setText(_translate("MainWindow", "123 ms", None))
        self.label_5.setText(_translate("MainWindow", "Drive motor mode", None))
        self.driveModeSelection.setItemText(0, _translate("MainWindow", "Low Speed", None))
        self.driveModeSelection.setItemText(1, _translate("MainWindow", "Med Speed", None))
        self.driveModeSelection.setItemText(2, _translate("MainWindow", "High Speed", None))
        self.driveModeSelection.setItemText(3, _translate("MainWindow", "Open Loop", None))
        self.label_3.setText(_translate("MainWindow", "Joystick Control Function", None))
        self.DriveMode.setText(_translate("MainWindow", "Drive", None))
        self.EndEffectorMode.setText(_translate("MainWindow", "End Effector", None))
        self.ArmBaseMode.setText(_translate("MainWindow", "Arm Base", None))
        self.function4.setText(_translate("MainWindow", "Camera", None))
        self.label_2.setText(_translate("MainWindow", "Arm mode", None))
        self.arm_mode.setItemText(0, _translate("MainWindow", "Position Control", None))
        self.arm_mode.setItemText(1, _translate("MainWindow", "Velocity Control", None))
        self.label_8.setText(_translate("MainWindow", "Coordinate", None))
        self.coordinateSystem.setItemText(0, _translate("MainWindow", "Cylindrical", None))
        self.coordinateSystem.setItemText(1, _translate("MainWindow", "Cartesian", None))
        self.label_9.setText(_translate("MainWindow", "X", None))
        self.label_10.setText(_translate("MainWindow", "Y", None))
        self.label_11.setText(_translate("MainWindow", "Z", None))
        self.sendCoords.setText(_translate("MainWindow", "Send", None))
        self.menuFile.setTitle(_translate("MainWindow", "File", None))
        self.actionQuit.setText(_translate("MainWindow", "Quit", None))
        self.actionRestart.setText(_translate("MainWindow", "Restart", None))

from pyqtgraph import GraphicsLayoutWidget
