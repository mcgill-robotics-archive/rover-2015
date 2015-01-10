# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_files/Controller_Feedback.ui'
#
# Created: Fri Jan  9 14:41:21 2015
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

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(210, 178)
        self.StickRotation = QtGui.QDial(Form)
        self.StickRotation.setGeometry(QtCore.QRect(20, 20, 50, 64))
        self.StickRotation.setMinimum(-1)
        self.StickRotation.setMaximum(1)
        self.StickRotation.setObjectName(_fromUtf8("StickRotation"))
        self.MainY = QtGui.QSlider(Form)
        self.MainY.setGeometry(QtCore.QRect(80, 10, 22, 160))
        self.MainY.setMinimum(-1)
        self.MainY.setMaximum(1)
        self.MainY.setOrientation(QtCore.Qt.Vertical)
        self.MainY.setObjectName(_fromUtf8("MainY"))
        self.MainX = QtGui.QSlider(Form)
        self.MainX.setGeometry(QtCore.QRect(10, 80, 160, 22))
        self.MainX.setMinimum(-1)
        self.MainX.setMaximum(1)
        self.MainX.setOrientation(QtCore.Qt.Horizontal)
        self.MainX.setInvertedAppearance(False)
        self.MainX.setObjectName(_fromUtf8("MainX"))
        self.SecondaryY = QtGui.QSlider(Form)
        self.SecondaryY.setGeometry(QtCore.QRect(180, 10, 22, 160))
        self.SecondaryY.setMinimum(-1)
        self.SecondaryY.setMaximum(1)
        self.SecondaryY.setOrientation(QtCore.Qt.Vertical)
        self.SecondaryY.setObjectName(_fromUtf8("SecondaryY"))

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))

