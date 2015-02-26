__author__ = 'David'

import serial
from PyQt4 import QtCore, QtGui
from Window import Ui_MainWindow

import sys, time

class MainWindow(QtGui.QMainWindow):
    def __init__(self, parent = None):
        super(MainWindow,self).__init__(parent)

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)


        try:
            self.ser = serial.Serial(sys.argv[1], 9600, timeout=None)
        except Exception as e:
            print e
            exit()


        QtCore.QObject.connect(self.ui.AG_SETPT, QtCore.SIGNAL("editingFinished()"), lambda regId=0, value=self.ui.AG_SETPT : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.ENPOL, QtCore.SIGNAL("editingFinished()"), lambda regId=1, value=self.ui.ENPOL : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.DIRPOL, QtCore.SIGNAL("editingFinished()"), lambda regId=2, value=self.ui.DIRPOL : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.BRKPOL, QtCore.SIGNAL("editingFinished()"), lambda regId=3, value=self.ui.BRKPOL : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.SYNRECT, QtCore.SIGNAL("editingFinished()"), lambda regId=4, value=self.ui.SYNRECT : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.PWMF, QtCore.SIGNAL("editingFinished()"), lambda regId=5, value=self.ui.PWMF : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.SPDMODE, QtCore.SIGNAL("editingFinished()"), lambda regId=6, value=self.ui.SPDMODE : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.FGSEL, QtCore.SIGNAL("editingFinished()"), lambda regId=7, value=self.ui.FGSEL : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.BRKMOD, QtCore.SIGNAL("editingFinished()"), lambda regId=8, value=self.ui.BRKMOD : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.RETRY, QtCore.SIGNAL("editingFinished()"), lambda regId=9, value=self.ui.RETRY : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.ADVANCE, QtCore.SIGNAL("editingFinished()"), lambda regId=10, value=self.ui.ADVANCE : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.SPDREVS, QtCore.SIGNAL("editingFinished()"), lambda regId=11, value=self.ui.SPDREVS : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.MINSPD, QtCore.SIGNAL("editingFinished()"), lambda regId=12, value=self.ui.MINSPD : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.BASIC, QtCore.SIGNAL("editingFinished()"), lambda regId=13, value=self.ui.BASIC : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.SPEDTH, QtCore.SIGNAL("editingFinished()"), lambda regId=14, value=self.ui.SPEDTH : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.MOD120, QtCore.SIGNAL("editingFinished()"), lambda regId=15, value=self.ui.MOD120 : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.LRTIME, QtCore.SIGNAL("editingFinished()"), lambda regId=16, value=self.ui.LRTIME : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.HALLRST, QtCore.SIGNAL("editingFinished()"), lambda regId=17, value=self.ui.HALLRST : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.DELAY, QtCore.SIGNAL("editingFinished()"), lambda regId=18, value=self.ui.DELAY : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.AUTOADV, QtCore.SIGNAL("editingFinished()"), lambda regId=19, value=self.ui.AUTOADV : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.AUTOGAIN, QtCore.SIGNAL("editingFinished()"), lambda regId=20, value=self.ui.AUTOGAIN : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.ENSINE, QtCore.SIGNAL("editingFinished()"), lambda regId=21, value=self.ui.ENSINE : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.TDRIVE, QtCore.SIGNAL("editingFinished()"), lambda regId=22, value=self.ui.TDRIVE : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.DTIME, QtCore.SIGNAL("editingFinished()"), lambda regId=23, value=self.ui.DTIME : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.IDRIVE, QtCore.SIGNAL("editingFinished()"), lambda regId=24, value=self.ui.IDRIVE : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.INTCLK, QtCore.SIGNAL("editingFinished()"), lambda regId=25, value=self.ui.INTCLK : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.SPDGAIN, QtCore.SIGNAL("editingFinished()"), lambda regId=26, value=self.ui.SPDGAIN : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.HALLPOL, QtCore.SIGNAL("editingFinished()"), lambda regId=27, value=self.ui.HALLPOL : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.BYPFILT, QtCore.SIGNAL("editingFinished()"), lambda regId=28, value=self.ui.BYPFILT : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.FILK1, QtCore.SIGNAL("editingFinished()"), lambda regId=29, value=self.ui.FILK1 : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.FILK2, QtCore.SIGNAL("editingFinished()"), lambda regId=30, value=self.ui.FILK2 : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.BYPCOMP, QtCore.SIGNAL("editingFinished()"), lambda regId=31, value=self.ui.BYPCOMP : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.COMK1, QtCore.SIGNAL("editingFinished()"), lambda regId=32, value=self.ui.COMK1 : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.AA_SETPT, QtCore.SIGNAL("editingFinished()"), lambda regId=33, value=self.ui.AA_SETPT : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.COMK2, QtCore.SIGNAL("editingFinished()"), lambda regId=34, value=self.ui.COMK2 : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.OCPDEG, QtCore.SIGNAL("editingFinished()"), lambda regId=35, value=self.ui.OCPDEG : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.OCPTH, QtCore.SIGNAL("editingFinished()"), lambda regId=36, value=self.ui.OCPTH : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.OVTH, QtCore.SIGNAL("editingFinished()"), lambda regId=37, value=self.ui.OVTH : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.VREG_EN, QtCore.SIGNAL("editingFinished()"), lambda regId=38, value=self.ui.VREG_EN : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.LOOPGAIN, QtCore.SIGNAL("editingFinished()"), lambda regId=39, value=self.ui.LOOPGAIN : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.SPED, QtCore.SIGNAL("editingFinished()"), lambda regId=40, value=self.ui.SPED : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.RLOCK, QtCore.SIGNAL("editingFinished()"), lambda regId=41, value=self.ui.RLOCK : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.VMOV, QtCore.SIGNAL("editingFinished()"), lambda regId=42, value=self.ui.VMOV : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.CPFAIL, QtCore.SIGNAL("editingFinished()"), lambda regId=43, value=self.ui.CPFAIL : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.UVLO, QtCore.SIGNAL("editingFinished()"), lambda regId=44, value=self.ui.UVLO : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.OTS, QtCore.SIGNAL("editingFinished()"), lambda regId=45, value=self.ui.OTS : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.CPOC, QtCore.SIGNAL("editingFinished()"), lambda regId=46, value=self.ui.CPOC : self.setRegister(regId, value))
        QtCore.QObject.connect(self.ui.OCP, QtCore.SIGNAL("editingFinished()"), lambda regId=47, value=self.ui.OCP : self.setRegister(regId, value))

        QtCore.QObject.connect(self.ui.RESET, QtCore.SIGNAL("clicked()"), self.reset)
        QtCore.QObject.connect(self.ui.fault, QtCore.SIGNAL("clicked()"), self.fault)
        QtCore.QObject.connect(self.ui.ENABLE, QtCore.SIGNAL("clicked()"), self.enable)
        QtCore.QObject.connect(self.ui.DISABLE, QtCore.SIGNAL("clicked()"), self.disable)
        QtCore.QObject.connect(self.ui.speedSlider, QtCore.SIGNAL("sliderReleased()"), self.setSpeed)
        QtCore.QObject.connect(self.ui.speedValue, QtCore.SIGNAL("editingFinished()"), self.setSpeed)
        QtCore.QObject.connect(self.ui.readAll, QtCore.SIGNAL("clicked()"), self.readRegs)
      

    def setSpeed(self):
        self.ui.logbox.append(str(self.ui.speedSlider.value()))
        self.ser.write(str(200))
        self.ui.logbox.append(self.ser.readline())
        time.sleep(0.010)
        self.ser.write(str(self.ui.speedSlider.value()))
        self.clearSerial()


    def clearSerial(self):
        while self.ser.inWaiting():
            self.ui.logbox.append(self.ser.readline())

    def setRegister(self, registerId, value):
        self.ui.logbox.append("reg ID %d" %registerId)
        self.ui.logbox.append("waiting")
        self.ser.write(str(registerId))
        self.ui.logbox.append(self.ser.readline())
        time.sleep(0.010)
        self.ser.write(str(value.value()))
        self.ui.logbox.append("value %d" %value.value())
        self.ui.logbox.append("waiting")
        self.ui.logbox.append(self.ser.readline())
        self.ui.logbox.append(self.ser.readline())
        self.clearSerial()

    def reset(self):
        self.ui.logbox.append("waiting")
        self.ser.write(str(100))
        self.ui.logbox.append(self.ser.readline())
        time.sleep(0.010)
        self.ser.write(str(10))
        self.ui.logbox.append("waiting")
        self.ui.logbox.append(self.ser.readline())
        self.ui.logbox.append(self.ser.readline())
        self.clearSerial()

    def fault(self):
        self.ui.logbox.append("waiting")
        self.ser.write(str(100))
        self.ui.logbox.append(self.ser.readline())
        time.sleep(0.010)
        self.ser.write(str(13))
        self.ui.logbox.append("waiting")
        self.ui.logbox.append(self.ser.readline())
        self.clearSerial()

    def disable(self):
        self.ui.logbox.append("waiting")
        self.ser.write(str(100))
        self.ui.logbox.append(self.ser.readline())
        time.sleep(0.010)
        self.ser.write(str(11))
        self.ui.logbox.append("waiting")
        self.ui.logbox.append(self.ser.readline())
        self.ui.logbox.append(self.ser.readline())
        self.clearSerial()

    def enable(self):
        self.ui.logbox.append("waiting")
        self.ser.write(str(100))
        self.ui.logbox.append(self.ser.readline())
        time.sleep(0.010)
        self.ser.write(str(12))
        self.ui.logbox.append("waiting")
        self.ui.logbox.append(self.ser.readline())
        self.ui.logbox.append(self.ser.readline())
        self.clearSerial()

    def readRegs(self):
        self.ui.logbox.append("waiting")
        self.ser.write(str(100))
        self.ui.logbox.append(self.ser.readline())
        time.sleep(0.010)
        self.ser.write(str(14))
        self.ui.logbox.append(self.ser.readline())
        self.ui.logbox.append(self.ser.readline())
        self.ui.logbox.append("waiting")

        self.ui.AG_SETPT.setValue(int(self.ser.readline()))
        self.ui.ENPOL.setValue(int(self.ser.readline()))
        self.ui.DIRPOL.setValue(int(self.ser.readline()))
        self.ui.BRKPOL.setValue(int(self.ser.readline()))
        self.ui.SYNRECT.setValue(int(self.ser.readline()))
        self.ui.PWMF.setValue(int(self.ser.readline()))
        self.ui.SPDMODE.setValue(int(self.ser.readline()))
        self.ui.FGSEL.setValue(int(self.ser.readline()))
        self.ui.BRKMOD.setValue(int(self.ser.readline()))
        self.ui.RETRY.setValue(int(self.ser.readline()))
        self.ui.ADVANCE.setValue(int(self.ser.readline()))
        self.ui.SPDREVS.setValue(int(self.ser.readline()))
        self.ui.MINSPD.setValue(int(self.ser.readline()))
        self.ui.BASIC.setValue(int(self.ser.readline()))
        self.ui.SPEDTH.setValue(int(self.ser.readline()))
        self.ui.MOD120.setValue(int(self.ser.readline()))
        self.ui.LRTIME.setValue(int(self.ser.readline()))
        self.ui.HALLRST.setValue(int(self.ser.readline()))
        self.ui.DELAY.setValue(int(self.ser.readline()))
        self.ui.AUTOADV.setValue(int(self.ser.readline()))
        self.ui.AUTOGAIN.setValue(int(self.ser.readline()))
        self.ui.ENSINE.setValue(int(self.ser.readline()))
        self.ui.TDRIVE.setValue(int(self.ser.readline()))
        self.ui.DTIME.setValue(int(self.ser.readline()))
        self.ui.IDRIVE.setValue(int(self.ser.readline()))
        self.ui.INTCLK.setValue(int(self.ser.readline()))
        self.ui.SPDGAIN.setValue(int(self.ser.readline()))
        self.ui.HALLPOL.setValue(int(self.ser.readline()))
        self.ui.BYPFILT.setValue(int(self.ser.readline()))
        self.ui.FILK1.setValue(int(self.ser.readline()))
        self.ui.FILK2.setValue(int(self.ser.readline()))
        self.ui.BYPCOMP.setValue(int(self.ser.readline()))
        self.ui.COMK1.setValue(int(self.ser.readline()))
        self.ui.AA_SETPT.setValue(int(self.ser.readline()))
        self.ui.COMK2.setValue(int(self.ser.readline()))
        self.ui.OCPDEG.setValue(int(self.ser.readline()))
        self.ui.OCPTH.setValue(int(self.ser.readline()))
        self.ui.OVTH.setValue(int(self.ser.readline()))
        self.ui.VREG_EN.setValue(int(self.ser.readline()))
        self.ui.LOOPGAIN.setValue(int(self.ser.readline()))
        self.ui.SPED.setValue(int(self.ser.readline()))
        self.ui.RLOCK.setValue(int(self.ser.readline()))
        self.ui.VMOV.setValue(int(self.ser.readline()))
        self.ui.CPFAIL.setValue(int(self.ser.readline()))
        self.ui.UVLO.setValue(int(self.ser.readline()))
        self.ui.OTS.setValue(int(self.ser.readline()))
        self.ui.CPOC.setValue(int(self.ser.readline()))
        self.ui.OCP.setValue(int(self.ser.readline()))

        self.ui.logbox.append(self.ser.readline())

        self.ui.logbox.append("Done")
        self.clearSerial()

if __name__=="__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = MainWindow()
    AppWindow.show()
    sys.exit(app.exec_())