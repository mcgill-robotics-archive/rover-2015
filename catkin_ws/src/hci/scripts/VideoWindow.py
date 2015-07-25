# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '../ui_files/VideoWindow.ui'
#
# Created: Thu Jul 23 00:15:20 2015
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
        MainWindow.resize(747, 595)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.gridLayout = QtGui.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.camera2 = QtGui.QLabel(self.centralwidget)
        self.camera2.setFrameShape(QtGui.QFrame.Box)
        self.camera2.setObjectName(_fromUtf8("camera2"))
        self.horizontalLayout_2.addWidget(self.camera2)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.camera1 = QtGui.QLabel(self.centralwidget)
        self.camera1.setMinimumSize(QtCore.QSize(0, 510))
        self.camera1.setFrameShape(QtGui.QFrame.Box)
        self.camera1.setObjectName(_fromUtf8("camera1"))
        self.verticalLayout.addWidget(self.camera1)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.rot0 = QtGui.QRadioButton(self.centralwidget)
        self.rot0.setMaximumSize(QtCore.QSize(16777215, 18))
        self.rot0.setObjectName(_fromUtf8("rot0"))
        self.horizontalLayout.addWidget(self.rot0)
        self.rot90 = QtGui.QRadioButton(self.centralwidget)
        self.rot90.setMaximumSize(QtCore.QSize(16777215, 18))
        self.rot90.setObjectName(_fromUtf8("rot90"))
        self.horizontalLayout.addWidget(self.rot90)
        self.rot180 = QtGui.QRadioButton(self.centralwidget)
        self.rot180.setMaximumSize(QtCore.QSize(16777215, 18))
        self.rot180.setObjectName(_fromUtf8("rot180"))
        self.horizontalLayout.addWidget(self.rot180)
        self.rot270 = QtGui.QRadioButton(self.centralwidget)
        self.rot270.setMaximumSize(QtCore.QSize(16777215, 18))
        self.rot270.setObjectName(_fromUtf8("rot270"))
        self.horizontalLayout.addWidget(self.rot270)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2.addLayout(self.verticalLayout)
        self.camera3 = QtGui.QLabel(self.centralwidget)
        self.camera3.setFrameShape(QtGui.QFrame.Box)
        self.camera3.setObjectName(_fromUtf8("camera3"))
        self.horizontalLayout_2.addWidget(self.camera3)
        self.horizontalLayout_2.setStretch(0, 1)
        self.horizontalLayout_2.setStretch(1, 1)
        self.horizontalLayout_2.setStretch(2, 1)
        self.gridLayout.addLayout(self.horizontalLayout_2, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 747, 31))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.menuFile = QtGui.QMenu(self.menubar)
        self.menuFile.setObjectName(_fromUtf8("menuFile"))
        MainWindow.setMenuBar(self.menubar)
        self.actionReset = QtGui.QAction(MainWindow)
        self.actionReset.setObjectName(_fromUtf8("actionReset"))
        self.actionQuit = QtGui.QAction(MainWindow)
        self.actionQuit.setObjectName(_fromUtf8("actionQuit"))
        self.menuFile.addAction(self.actionReset)
        self.menuFile.addAction(self.actionQuit)
        self.menubar.addAction(self.menuFile.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QObject.connect(self.actionQuit, QtCore.SIGNAL(_fromUtf8("triggered()")), MainWindow.close)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.camera2.setText(_translate("MainWindow", "camLeft", None))
        self.camera1.setText(_translate("MainWindow", "camMain", None))
        self.rot0.setText(_translate("MainWindow", "0", None))
        self.rot90.setText(_translate("MainWindow", "90", None))
        self.rot180.setText(_translate("MainWindow", "180", None))
        self.rot270.setText(_translate("MainWindow", "270", None))
        self.camera3.setText(_translate("MainWindow", "camRight", None))
        self.menuFile.setTitle(_translate("MainWindow", "File", None))
        self.actionReset.setText(_translate("MainWindow", "Reset", None))
        self.actionQuit.setText(_translate("MainWindow", "Close", None))

