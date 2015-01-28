# -*- coding: utf-8 -*-
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np

class Minimap(object):

    def __init__(self):

        self.app = QtGui.QApplication([])
        self.mw = QtGui.QMainWindow()
        self.mw.resize(400,400)
        self.view = pg.GraphicsLayoutWidget()  ## GraphicsView with GraphicsLayout inserted by default
        self.mw.setCentralWidget(self.view)
        self.mw.show()
        self.mw.setWindowTitle('Minimap')

        self.w2 = self.view.addViewBox()
        self.w2.setAspectLocked(False)
        self.w2.enableAutoRange('xy',True)
        self.view.nextRow()


        self.s2 = pg.ScatterPlotItem(size=10, pen=pg.mkPen('w'), pxMode=True)
        self.x = []
        self.y = []
        self.waypoints_x = [0]
        self.waypoints_y = [0]
        self.s2.addPoints(self.waypoints_x,self.waypoints_y,size=30,symbol='t',brush='b')
        self.w2.addItem(self.s2)

        while(1):
            s = raw_input('Enter: x,y, or waypoint: ')
            if(s == 'waypoint'):
                print "waypoint at %s,%s" %(self.x[-1],self.y[-1])
                self.addWaypoint(self.x[-1],self.y[-1])
            else:
                try:
                    a,b = [float(i) for i in s.split(',')]
                    self.addPoint(a,b)
                except ValueError:
                    print "please enter either 'waypoint' or a valid x,y coordinate"
            self.w2.autoRange()
            
    def addPoint(self,x_coord,y_coord):
        self.x.append(x_coord)
        self.y.append(y_coord)
        self.s2.addPoints(self.x,self.y,size=10,symbol='o',brush='r')
        
    def addWaypoint(self,x_coord,y_coord):
        self.waypoints_x.append(x_coord)
        self.waypoints_y.append(y_coord)
        self.s2.addPoints(self.waypoints_x,self.waypoints_y,size=30,symbol='t',brush='b')


## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    minimap = Minimap()
