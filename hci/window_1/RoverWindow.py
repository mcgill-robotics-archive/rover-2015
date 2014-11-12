from PyQt4 import QtGui, QtCore
import pyqtgraph as pg
import sys
import numpy as np
import time


class Window(QtGui.QWidget):

    def __init__(self):
        super(Window,self).__init__()
        self.initUI()
        
    def initUI(self):
        #setup the window
        self.main_menu = QtGui.QComboBox()
        self.second_menu = QtGui.QComboBox()
        self.third_menu = QtGui.QComboBox()
        self.main_menu_label = QtGui.QLabel("Main Video Feed")
        self.second_menu_label = QtGui.QLabel("Secondary Video Feed")
        self.third_menu_label = QtGui.QLabel("Third Video Feed")
        self.main_feed = QtGui.QPixmap("Rover_View.jpg")
        self.second_feed = QtGui.QPixmap("Rover_View_2.jpg")
        self.third_feed = QtGui.QPixmap("Rover_View_3.jpg")
        self.main_label = QtGui.QLabel(self)
        self.second_label = QtGui.QLabel(self)
        self.third_label = QtGui.QLabel(self)
        
        self.miniMap = pg.GraphicsWindow(title='Minimap')
        self.trail = self.miniMap.addPlot()
        self.trail.setLabel('bottom','Mini Map')
        self.trail.showAxis('left', False)
        
        self.battery = QtGui.QLabel("Battery Life: ")
        self.bat_life = QtGui.QLabel("\n\n\n\n\nOVER 9000 years")
        
        self.Function_1 = QtGui.QPushButton('Function 1')
        self.Function_2 = QtGui.QPushButton('Function 2')
        self.Function_3 = QtGui.QPushButton('Function 3')
        self.Function_4 = QtGui.QPushButton('Function 4')
        self.Function_5 = QtGui.QPushButton('Function 5')

        self.handType = QtGui.QComboBox()
        self.handType_label = QtGui.QLabel("Hand Type:")
        
        self.wristInfo = QtGui.QLabel("Wrist Information: ")
        self.theta = QtGui.QLabel("Theta: ")
        self.distance = QtGui.QLabel("Distance: ")
        self.height = QtGui.QLabel("Height: ")
        
        
        self.main_label.setGeometry(0,0,650,460)
        self.second_label.setGeometry(0,0,500,200)
        self.third_label.setGeometry(0,0,500,200)
        #self.main_label.setSizePolicy(QtGui.QSizePolicy.Preferred,QtGui.QSizePolicy.Preferred)
        
        self.scaled_main_feed = self.main_feed.scaled(self.main_label.size())
        self.scaled_second_feed = self.second_feed.scaled(self.second_label.size())
        self.scaled_third_feed = self.third_feed.scaled(self.third_label.size())
#size(), QtCore.Qt.KeepAspectRatio
        
        self.main_label.setPixmap(self.scaled_main_feed)
        self.second_label.setPixmap(self.scaled_second_feed)
        self.third_label.setPixmap(self.scaled_third_feed)
        
        self.main_menu.addItem("Head Camera")
        self.main_menu.addItem("Cam 2")
        self.main_menu.addItem("Cam 3")
        self.second_menu.addItem("Left Wheel Camera")
        self.second_menu.addItem("Cam 4")
        self.second_menu.addItem("Cam 5")
        self.third_menu.addItem("Rear Camera")
        self.third_menu.addItem("Cam 6")
        
        self.handType.addItem("Drill")
        self.handType.addItem("Strong hand")
        self.handType.addItem("Precise hand")
        
        self.functionsLayout = QtGui.QHBoxLayout()
        self.functionsLayout.addStretch(1)
        self.functionsLayout.addWidget(self.Function_1)
        self.functionsLayout.addWidget(self.Function_2)
        self.functionsLayout.addWidget(self.Function_3)
        self.functionsLayout.addWidget(self.Function_4)
        self.functionsLayout.addWidget(self.Function_5)
        
        self.handbatLayout = QtGui.QVBoxLayout()
        self.handbatLayout.addWidget(self.handType_label)
        self.handbatLayout.addWidget(self.handType)
        self.handbatLayout.addWidget(self.battery)
        
        self.botLeftLayout = QtGui.QHBoxLayout()
        self.botLeftLayout.addWidget(self.miniMap)
        self.botLeftLayout.addLayout(self.handbatLayout)
        
        self.leftLayout = QtGui.QVBoxLayout()
        self.leftLayout.addWidget(self.main_menu_label)
        self.leftLayout.addWidget(self.main_menu)
        self.leftLayout.addWidget(self.main_label)
        self.leftLayout.addLayout(self.botLeftLayout)
        self.leftLayout.addStretch(1)
        
        self.infoLayout = QtGui.QVBoxLayout()
        self.infoLayout.addWidget(self.wristInfo)
        self.infoLayout.addWidget(self.theta)
        self.infoLayout.addWidget(self.distance)
        self.infoLayout.addWidget(self.height)
        
        self.handLayout = QtGui.QHBoxLayout()
        self.handLayout.addWidget(self.bat_life)
        self.handLayout.addLayout(self.infoLayout)
        
        self.rightLayout = QtGui.QVBoxLayout()
        self.rightLayout.addWidget(self.second_menu_label)
        self.rightLayout.addWidget(self.second_menu)
        self.rightLayout.addWidget(self.second_label)
        self.rightLayout.addWidget(self.third_menu_label)
        self.rightLayout.addWidget(self.third_menu)
        self.rightLayout.addWidget(self.third_label)
        self.rightLayout.addLayout(self.functionsLayout)
        self.rightLayout.addLayout(self.handLayout)
        self.rightLayout.addStretch(1)
        
        self.horLayout = QtGui.QHBoxLayout()
        self.horLayout.addLayout(self.leftLayout)
        self.horLayout.addLayout(self.rightLayout)
        self.horLayout.addStretch(1)
        
        self.setLayout(self.horLayout)
        self.resize(700,700)
        self.setWindowTitle('Rover Interaction Window')
        self.show()

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = Window()
    sys.exit(app.exec_())
    
    
    
    
    
    
    
    
    
    
    
    
