# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MainWindow_V2.ui'
#
# Created: Fri Dec 19 13:05:29 2014
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
        MainWindow.resize(825, 566)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.gridLayout = QtGui.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.frame_6 = QtGui.QFrame(self.centralwidget)
        self.frame_6.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_6.setObjectName(_fromUtf8("frame_6"))
        self.gridLayout_2 = QtGui.QGridLayout(self.frame_6)
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.functionBox = QtGui.QFrame(self.frame_6)
        self.functionBox.setFrameShape(QtGui.QFrame.StyledPanel)
        self.functionBox.setFrameShadow(QtGui.QFrame.Raised)
        self.functionBox.setObjectName(_fromUtf8("functionBox"))
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.functionBox)
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.function1 = QtGui.QPushButton(self.functionBox)
        self.function1.setObjectName(_fromUtf8("function1"))
        self.verticalLayout_2.addWidget(self.function1)
        self.function2 = QtGui.QPushButton(self.functionBox)
        self.function2.setObjectName(_fromUtf8("function2"))
        self.verticalLayout_2.addWidget(self.function2)
        self.function3 = QtGui.QPushButton(self.functionBox)
        self.function3.setObjectName(_fromUtf8("function3"))
        self.verticalLayout_2.addWidget(self.function3)
        self.function4 = QtGui.QPushButton(self.functionBox)
        self.function4.setObjectName(_fromUtf8("function4"))
        self.verticalLayout_2.addWidget(self.function4)
        self.gridLayout_2.addWidget(self.functionBox, 4, 3, 1, 1)
        self.frame_8 = QtGui.QFrame(self.frame_6)
        self.frame_8.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_8.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_8.setObjectName(_fromUtf8("frame_8"))
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.frame_8)
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.comboBox_5 = QtGui.QComboBox(self.frame_8)
        self.comboBox_5.setObjectName(_fromUtf8("comboBox_5"))
        self.verticalLayout_3.addWidget(self.comboBox_5)
        self.comboBox_7 = QtGui.QComboBox(self.frame_8)
        self.comboBox_7.setObjectName(_fromUtf8("comboBox_7"))
        self.verticalLayout_3.addWidget(self.comboBox_7)
        self.gridLayout_2.addWidget(self.frame_8, 5, 2, 1, 1)
        self.Camera2Feed = QtGui.QComboBox(self.frame_6)
        self.Camera2Feed.setObjectName(_fromUtf8("Camera2Feed"))
        self.Camera2Feed.addItem(_fromUtf8(""))
        self.Camera2Feed.addItem(_fromUtf8(""))
        self.Camera2Feed.addItem(_fromUtf8(""))
        self.gridLayout_2.addWidget(self.Camera2Feed, 0, 2, 1, 1)
        self.camera1 = QtGui.QLabel(self.frame_6)
        self.camera1.setObjectName(_fromUtf8("camera1"))
        self.gridLayout_2.addWidget(self.camera1, 2, 0, 2, 2)
        self.OtherData = QtGui.QFrame(self.frame_6)
        self.OtherData.setFrameShape(QtGui.QFrame.StyledPanel)
        self.OtherData.setFrameShadow(QtGui.QFrame.Raised)
        self.OtherData.setObjectName(_fromUtf8("OtherData"))
        self.OtherDataLabel = QtGui.QLabel(self.OtherData)
        self.OtherDataLabel.setGeometry(QtCore.QRect(60, 0, 123, 17))
        self.OtherDataLabel.setObjectName(_fromUtf8("OtherDataLabel"))
        self.gridLayout_2.addWidget(self.OtherData, 4, 4, 2, 2)
        self.camera2 = QtGui.QLabel(self.frame_6)
        self.camera2.setObjectName(_fromUtf8("camera2"))
        self.gridLayout_2.addWidget(self.camera2, 2, 2, 2, 2)
        self.HandTypeBox = QtGui.QFrame(self.frame_6)
        self.HandTypeBox.setFrameShape(QtGui.QFrame.StyledPanel)
        self.HandTypeBox.setFrameShadow(QtGui.QFrame.Raised)
        self.HandTypeBox.setObjectName(_fromUtf8("HandTypeBox"))
        self.verticalLayout = QtGui.QVBoxLayout(self.HandTypeBox)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.label = QtGui.QLabel(self.HandTypeBox)
        self.label.setObjectName(_fromUtf8("label"))
        self.verticalLayout.addWidget(self.label)
        self.HandTypeChoice = QtGui.QComboBox(self.HandTypeBox)
        self.HandTypeChoice.setObjectName(_fromUtf8("HandTypeChoice"))
        self.verticalLayout.addWidget(self.HandTypeChoice)
        self.label_2 = QtGui.QLabel(self.HandTypeBox)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.verticalLayout.addWidget(self.label_2)
        self.ArmModeChoice = QtGui.QComboBox(self.HandTypeBox)
        self.ArmModeChoice.setObjectName(_fromUtf8("ArmModeChoice"))
        self.verticalLayout.addWidget(self.ArmModeChoice)
        self.gridLayout_2.addWidget(self.HandTypeBox, 4, 2, 1, 1)
        self.Minimap = QtGui.QFrame(self.frame_6)
        self.Minimap.setFrameShape(QtGui.QFrame.StyledPanel)
        self.Minimap.setFrameShadow(QtGui.QFrame.Raised)
        self.Minimap.setObjectName(_fromUtf8("Minimap"))
        self.MinimapLabel = QtGui.QLabel(self.Minimap)
        self.MinimapLabel.setGeometry(QtCore.QRect(90, 0, 63, 17))
        self.MinimapLabel.setObjectName(_fromUtf8("MinimapLabel"))
        self.gridLayout_2.addWidget(self.Minimap, 4, 0, 2, 2)
        self.Camera1Feed = QtGui.QComboBox(self.frame_6)
        self.Camera1Feed.setObjectName(_fromUtf8("Camera1Feed"))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.Camera1Feed.addItem(_fromUtf8(""))
        self.gridLayout_2.addWidget(self.Camera1Feed, 0, 0, 1, 1)
        self.frame_9 = QtGui.QFrame(self.frame_6)
        self.frame_9.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_9.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_9.setObjectName(_fromUtf8("frame_9"))
        self.verticalLayout_4 = QtGui.QVBoxLayout(self.frame_9)
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.pushButton_5 = QtGui.QPushButton(self.frame_9)
        self.pushButton_5.setObjectName(_fromUtf8("pushButton_5"))
        self.verticalLayout_4.addWidget(self.pushButton_5)
        self.pushButton_6 = QtGui.QPushButton(self.frame_9)
        self.pushButton_6.setObjectName(_fromUtf8("pushButton_6"))
        self.verticalLayout_4.addWidget(self.pushButton_6)
        self.gridLayout_2.addWidget(self.frame_9, 5, 3, 1, 1)
        self.Camera3Feed = QtGui.QComboBox(self.frame_6)
        self.Camera3Feed.setObjectName(_fromUtf8("Camera3Feed"))
        self.Camera3Feed.addItem(_fromUtf8(""))
        self.Camera3Feed.addItem(_fromUtf8(""))
        self.Camera3Feed.addItem(_fromUtf8(""))
        self.gridLayout_2.addWidget(self.Camera3Feed, 0, 4, 1, 1)
        self.camera3 = QtGui.QLabel(self.frame_6)
        self.camera3.setObjectName(_fromUtf8("camera3"))
        self.gridLayout_2.addWidget(self.camera3, 2, 4, 2, 2)
        self.gridLayout.addWidget(self.frame_6, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 825, 25))
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
        self.function1.setText(_translate("MainWindow", "function 1", None))
        self.function2.setText(_translate("MainWindow", "function 2", None))
        self.function3.setText(_translate("MainWindow", "function 3", None))
        self.function4.setText(_translate("MainWindow", "function 4", None))
        self.Camera2Feed.setItemText(0, _translate("MainWindow", "feed 1", None))
        self.Camera2Feed.setItemText(1, _translate("MainWindow", "feed 2", None))
        self.Camera2Feed.setItemText(2, _translate("MainWindow", "feed n", None))
        self.camera1.setText(_translate("MainWindow", "cam 1", None))
        self.OtherDataLabel.setText(_translate("MainWindow", "display other data", None))
        self.camera2.setText(_translate("MainWindow", "cam 2", None))
        self.label.setText(_translate("MainWindow", "Hand type", None))
        self.label_2.setText(_translate("MainWindow", "Arm mode", None))
        self.MinimapLabel.setText(_translate("MainWindow", "mini map", None))
        self.Camera1Feed.setItemText(0, _translate("MainWindow", "feed 1", None))
        self.Camera1Feed.setItemText(1, _translate("MainWindow", "feed 2", None))
        self.Camera1Feed.setItemText(2, _translate("MainWindow", "feed n", None))
        self.pushButton_5.setText(_translate("MainWindow", "PushButton", None))
        self.pushButton_6.setText(_translate("MainWindow", "PushButton", None))
        self.Camera3Feed.setItemText(0, _translate("MainWindow", "feed 1", None))
        self.Camera3Feed.setItemText(1, _translate("MainWindow", "feed 2", None))
        self.Camera3Feed.setItemText(2, _translate("MainWindow", "feed n", None))
        self.camera3.setText(_translate("MainWindow", "cam 3", None))
        self.menuWindow.setTitle(_translate("MainWindow", "window", None))
        self.toolBar.setWindowTitle(_translate("MainWindow", "toolBar", None))

