# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'VideoWindow.ui'
#
# Created: Mon May 25 21:41:57 2015
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
        MainWindow.resize(747, 577)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.camera2 = QtGui.QLabel(self.centralwidget)
        self.camera2.setFrameShape(QtGui.QFrame.Box)
        self.camera2.setObjectName(_fromUtf8("camera2"))
        self.horizontalLayout.addWidget(self.camera2)
        self.camera1 = QtGui.QLabel(self.centralwidget)
        self.camera1.setFrameShape(QtGui.QFrame.Box)
        self.camera1.setObjectName(_fromUtf8("camera1"))
        self.horizontalLayout.addWidget(self.camera1)
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.camera3 = QtGui.QLabel(self.centralwidget)
        self.camera3.setFrameShape(QtGui.QFrame.Box)
        self.camera3.setObjectName(_fromUtf8("camera3"))
        self.verticalLayout_2.addWidget(self.camera3)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 747, 22))
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
        self.camera3.setText(_translate("MainWindow", "camRight", None))
        self.menuFile.setTitle(_translate("MainWindow", "File", None))
        self.actionReset.setText(_translate("MainWindow", "Reset", None))
        self.actionQuit.setText(_translate("MainWindow", "Close", None))

