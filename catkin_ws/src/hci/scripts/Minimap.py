# -*- coding: utf-8 -*-
"""
Example demonstrating a variety of scatter plot features.
"""


from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np
import Tkinter as tk


class Keypress:
    def __init__(self):
        self.root = tk.Tk()
        self.root.geometry('300x200')
        self.root.bind('<KeyPress>', self.onKeyPress)

    def onKeyPress(self, event):
        self.key = event.char

    def __eq__(self, other):
        return self.key == other

    def __str__(self):
        return self.key

app = QtGui.QApplication([])
mw = QtGui.QMainWindow()
mw.resize(400,400)
view = pg.GraphicsLayoutWidget()  ## GraphicsView with GraphicsLayout inserted by default
mw.setCentralWidget(view)
mw.show()
mw.setWindowTitle('Minimap')

## create four areas to add plots
w2 = view.addViewBox()
w2.setAspectLocked(False)
w2.enableAutoRange('xy',True)
view.nextRow()
print("Generating data, this takes a few seconds...")

## Make all plots clickable
lastClicked = []
def clicked(plot, points):
    global lastClicked
    for p in lastClicked:
        p.resetPen()
    print("clicked points", points)
    for p in points:
        p.setPen('b', width=2)
    lastClicked = points



## 2) Spots are transform-invariant, but not identical (top-right plot). 
## In this case, drawing is as fast as 1), but there is more startup overhead
## and memory usage since each spot generates its own pre-rendered image.

s2 = pg.ScatterPlotItem(size=10, pen=pg.mkPen('w'), pxMode=True)
#pos = np.random.normal(size=(2,n), scale=1e-5)
x = []
y = []
waypoints_x = [0]
waypoints_y = [0]
#spots = [{'pos': pos[:,i], 'data': 1, 'brush':pg.intColor(i, n), 'symbol': i%5, 'size': 5+i/10.} for i in range(n)]
#s2.addPoints(spots)
s2.addPoints(waypoints_x,waypoints_y,size=30,symbol='t',brush='b')
w2.addItem(s2)
s2.sigClicked.connect(clicked)

while(1):
    s = raw_input('Enter: x,y, or waypoint: ')
    if(s == 'waypoint'):
        waypoints_x.append(x[-1])
        waypoints_y.append(y[-1])
        print "waypoint at %s,%s" %(x[-1],y[-1])
        #s2.setSize(30,waypoints_x)
        #s2.setSize(30,waypoints_y)
        s2.addPoints(waypoints_x,waypoints_y,size=30,symbol='t',brush='b')
    else:
        a,b = [float(i) for i in s.split(',')]
        x.append(a)
        y.append(b)
        s2.addPoints(x,y,size=10,symbol='o',brush='r')
    w2.autoRange()


## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
