#!/usr/bin/python

from RoverWindow import *
##from no_imu import *
from PyQt4 import QtCore, QtGui
##import publisher
from JoystickController import JoystickController
from VARIABLES import *
from publisher import Publisher
import pyqtgraph as pg

import sys
import rospy
import Queue

from std_msgs.msg import String  # ros message types
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose
from geometry_msgs.msg import Pose

class CentralUi(QtGui.QMainWindow):
	def __init__(self, parent=None):
		super(CentralUi,self).__init__(parent)

		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
		self.controller = JoystickController()
		self.publisher = Publisher()
		self.modeId = 0 
		self.grip = 0

		# feed 1 holders
		self.__image1=None
		self.__screen1Subscriber=None
		# feed 2 holders
		self.__image2=None
		self.__screen2Subscriber=None
		# feed 3 holders
		self.__image3=None
		self.__screen3Subscriber=None

		## List of topic names for the camera feeds, compiled from parameter server
		self.camera_topic_list = []
                
                self.tempPose = Queue.Queue()

		self.controller_timer = QtCore.QTimer()
		self.addPointTimer = QtCore.QTimer()
		
		self.setControllerTimer()
		rospy.loginfo("Starting joystick acquisition")

		##button connects
		# controller timer connect
		QtCore.QObject.connect(self.controller_timer, QtCore.SIGNAL("timeout()"), self.readController)
            	QtCore.QObject.connect(self.addPointTimer, QtCore.SIGNAL("timeout()"), self.addPointTimeout)
                
                self.addPointTimer.start(100)
                # joystick mode buttons signal connect
		QtCore.QObject.connect(self.ui.DriveMode, QtCore.SIGNAL("clicked()"), self.setMode0)
		QtCore.QObject.connect(self.ui.ArmBaseMode, QtCore.SIGNAL("clicked()"), self.setMode1)
		QtCore.QObject.connect(self.ui.EndEffectorMode, QtCore.SIGNAL("clicked()"), self.setMode2)
		QtCore.QObject.connect(self.ui.function4, QtCore.SIGNAL("clicked()"), self.setMode3)
		# connecto for point steer mode
		QtCore.QObject.connect(self.ui.pointSteer, QtCore.SIGNAL("toggled(bool)"), self.setPointSteer)
		# camera feed selection signal connects
		QtCore.QObject.connect(self.ui.Camera1Feed, QtCore.SIGNAL("currentIndexChanged(int)"), self.setFeed1Index)
		QtCore.QObject.connect(self.ui.Camera2Feed, QtCore.SIGNAL("currentIndexChanged(int)"), self.setFeed2Index)
		QtCore.QObject.connect(self.ui.Camera3Feed, QtCore.SIGNAL("currentIndexChanged(int)"), self.setFeed3Index)
		
		self.ui.pushButton.clicked.connect(self.addWayPoint)
		self.ui.pushButton_2.clicked.connect(self.clearMap)

		self.screen_redraw_timer=QtCore.QTimer(self)
		self.screen_redraw_timer.timeout.connect(self.repaint_image)
		self.screen_redraw_timer.start(100)
		self.setupMinimap()

		self.ros_init()
		rospy.Subscriber('pose', Pose, self.handlePose)
		rospy.loginfo("HCI initialization completed")
		
	def setupMinimap(self):
		self.x = []
		self.y = []
		self.x_waypoints = [0]
		self.y_waypoints = [0]
		self.w1 = self.ui.graphicsView.addViewBox()
		self.w1.setAspectLocked(False)
		self.w1.enableAutoRange('xy',True)
		self.ui.graphicsView.nextRow()
		
		self.s1 = pg.ScatterPlotItem(size=10, pen=pg.mkPen('w'), pxMode=True)
		self.s1.addPoints(self.x_waypoints,self.y_waypoints,size=10,symbol='t',brush='b')
		self.w1.addItem(self.s1)
	
	def handlePose(self,data):
                #add (x,y) to tempPose queue
                self.tempPose.put(data)
		
        def addPointTimeout(self):
                while not self.tempPose.empty():
                    pose=self.tempPose.get()
                    self.newx=[pose.position.x]
                    self.newy=[pose.position.y]
                    self.s1.addPoints(self.newx,self.newy, size=3, symbol='o', brush='w')
                    self.w1.autoRange()


	def addPoint(self,x_coord,y_coord):
		self.x.append(x_coord)
		self.y.append(y_coord)
		self.newx = [x_coord]
		self.newy = [y_coord]
		self.s1.addPoints(self.newx,self.newy,size=3,symbol='o',brush='w')
		self.w1.autoRange()
		
	def addWayPoint(self):
		self.x_waypoints.append(self.newx[0])
		self.y_waypoints.append(self.newx[0])
		self.x_waypoint = [self.newx[0]]
		self.y_waypoint = [self.newy[0]]
		self.s1.addPoints(self.x_waypoint,self.y_waypoint,size=10,symbol='t',brush='b')
		self.w1.autoRange()
		
	def clearMap(self):
		self.x = []
		self.y = []
		self.x_waypoints = [0]
		self.y_waypoints = [0]
		self.s1.setData(self.x,self.y,size=10,symbol='o',brush='r')
		self.s1.addPoints(self.x_waypoints,self.y_waypoints,size=10,symbol='t',brush='b')

	def setPointSteer(self, boolean):
		self.publisher.setSteerMode(boolean)

	def setControllerTimer(self):
		if self.controller.controller is not None:
			self.controller_timer.start(100)
			rospy.loginfo("Started controller timer")
		else:
			rospy.loginfo("Missing controller, timer aborted")

	def readController(self):
		self.controller.update()
		self.ui.MainX.setValue(self.controller.a1*1000)
		self.ui.MainY.setValue(-self.controller.a2*1000)
		self.ui.StickRotation.setValue(self.controller.a3*1000)
		self.ui.SecondaryY.setValue(-self.controller.a4*1000)

		if self.controller.b3:
			self.ui.Camera1Feed.setCurrentIndex(5)
		elif self.controller.b4:
			self.ui.Camera1Feed.setCurrentIndex(4)
		elif self.controller.b2:
			self.ui.pointSteer.setChecked(not self.ui.pointSteer.isChecked())
		elif self.controller.b7:
			self.setControllerMode(0)
		elif self.controller.b8:
			self.setControllerMode(1)
		elif self.controller.b9:
			self.setControllerMode(2)
		elif self.controller.b10:
			self.setControllerMode(3)
		elif self.controller.hat ==(-1,0):
			self.ui.Camera1Feed.setCurrentIndex(1)
		elif self.controller.hat ==(0,1):
			self.ui.Camera1Feed.setCurrentIndex(0)
		elif self.controller.hat ==(0,-1):
			self.ui.Camera1Feed.setCurrentIndex(3)
		elif self.controller.hat ==(1,0):
			self.ui.Camera1Feed.setCurrentIndex(2)
		self.controller.clear_buttons()
		#minus sign to compensate for joystick inherent positive and negative mappings
		self.publishControlls()

	def publishControlls(self):
		if self.modeId == 0:
			self.publisher.publish_velocity(self.controller.a1, -self.controller.a2)
		elif self.modeId == 1:
			length = -self.controller.a2
			height = self.controller.a1
			angle = self.controller.a3
			self.publisher.publish_arm_base_movement(length,height,angle)
		elif self.modeId == 2:
			x = -self.controller.a2
			y = self.controller.a3
			rotate = self.controller.a1
			if self.grip == 0 :
				if self.controller.b3 :
					self.grip = 1
				elif self.controller.b4 :
					self.grip = -1
			else :
				if self.controller.b3 or self.controller.b4 :
					self.grip = 0
			grip = self.grip

			self.publisher.publish_endEffector(x,y,rotate,grip)

			#end effector mode
			#use joystick to controll a1,a2, a3 for rotating motion and someother button for grip motion

	def setMode0(self):
		self.setControllerMode(0)

	def setMode1(self):
		self.setControllerMode(1)

	def setMode2(self):
		self.setControllerMode(2)

	def setMode3(self):
		self.setControllerMode(3)

	def setControllerMode(self, mode_id):
		self.modeId = mode_id
		if mode_id == 0:
			self.ui.DriveMode.setChecked(True)
			self.ui.ArmBaseMode.setChecked(False)
			self.ui.EndEffectorMode.setChecked(False)
			self.ui.function4.setChecked(False)
		if mode_id == 1:
			self.ui.DriveMode.setChecked(False)
			self.ui.ArmBaseMode.setChecked(True)
			self.ui.EndEffectorMode.setChecked(False)
			self.ui.function4.setChecked(False)
		if mode_id == 2:
			self.ui.DriveMode.setChecked(False)
			self.ui.ArmBaseMode.setChecked(False)
			self.ui.EndEffectorMode.setChecked(True)
			self.ui.function4.setChecked(False)
		if mode_id == 3:
			self.ui.DriveMode.setChecked(False)
			self.ui.ArmBaseMode.setChecked(False)
			self.ui.EndEffectorMode.setChecked(False)
			self.ui.function4.setChecked(True)


	def callbackScreen3(self, data):
		try:
			self.__image3 = data
		finally:
			pass


	def callbackScreen2(self, data):
		try:
			self.__image2=data
		finally:
			pass


	def callbackScreen1(self, data):
		try:
			self.__image1=data
		finally:
			pass


	def setFeed1Index(self, newIndex):
		try:
			topic = self.camera_topic_list[newIndex]
		except IndexError :
			rospy.logwarn("New camera feed topic name not found")
			return
		self.__screen1Subscriber.unsubscribe()
		self.__screen1Subscriber = rospy.Subscriber(topic, Image, self.callbackScreen1)


	def setFeed2Index(self, newIndex):
		try:
			topic = self.camera_topic_list[newIndex]
		except IndexError :
			rospy.logwarn("New camera feed topic name not found")
			return
		self.__screen2Subscriber.unsubscribe()
		self.__screen2Subscriber = rospy.Subscriber(topic, Image, self.callbackScreen1)


	def setFeed3Index(self, newIndex):
		try:
			topic = self.camera_topic_list[newIndex]
		except IndexError :
			rospy.logwarn("New camera feed topic name not found")
			return
		self.__screen3Subscriber.unsubscribe()
		self.__screen3Subscriber = rospy.Subscriber(topic, Image, self.callbackScreen1)


	def repaint_image(self):
		if self.__image2 is not None:
			try:
				qimage2 = QtGui.QImage(self.__image2.data, self.__image2.width/3, self.__image2.height/3,QtGui.QImage.Format_RGB888)
				#TODO: FIGURE OUT THIS DIVIDED BY 3 !!! 
				# wiskey-tango-foxtrot
				image2 = QtGui.QPixmap.fromImage(qimage2)     
			finally:
				pass
			self.ui.camera2.setPixmap(image2)
		else:
			self.ui.camera2.setText("no video feed")
		if self.__image3 is not None:
			try: 
				qimage3 = QtGui.QImage(self.__image3.data, self.__image3.width/3, self.__image3.height/3,QtGui.QImage.Format_RGB888)
				#TODO: FIGURE OUT THIS DIVIDED BY 3 !!! 
				# wiskey-tango-foxtrot
				image3 = QtGui.QPixmap.fromImage(qimage3)       
			finally:
				pass
			self.ui.camera3.setPixmap(image3)
		else:
			self.ui.camera3.setText("no video feed")
		if self.__image1 is not None:
			try: 
				qimage1 = QtGui.QImage(self.__image1.data, self.__image1.width/3, self.__image1.height/3,QtGui.QImage.Format_RGB888)
				#TODO: FIGURE OUT THIS DIVIDED BY 3 !!! 
				# wiskey-tango-foxtro
				image1 = QtGui.QPixmap.fromImage(qimage1)     
			finally:
				pass
			self.ui.camera1.setPixmap(image1)
		else:
			self.ui.camera1.setText("no video feed")


	def ros_init(self):
		rospy.init_node('listener',anonymous=False)
		self.getImageTopic()
		if len(self.camera_topic_list)>2:
			self.__screen3Subscriber=rospy.Subscriber(self.camera_topic_list[2],Image, self.callbackScreen3)
			self.__screen2Subscriber=rospy.Subscriber(self.camera_topic_list[1],Image, self.callbackScreen2)
			self.__screen1Subscriber=rospy.Subscriber(self.camera_topic_list[0],Image, self.callbackScreen1)


	def getImageTopic(self):
		self.camera_topic_list.append(rospy.get_param("camera/topic_haz_front", "/hazcam_front/camera/image_raw"))
		self.camera_topic_list.append(rospy.get_param("camera/topic_haz_left", "/hazcam_left/camera/image_raw"))
		self.camera_topic_list.append(rospy.get_param("camera/topic_haz_right", "/hazcam_right/camera/image_raw"))
		self.camera_topic_list.append(rospy.get_param("camera/topic_haz_back", "/hazcam_back/camera/image_raw"))
		self.camera_topic_list.append(rospy.get_param("camera/topic_arm", "/camera_arm/camera/image_raw"))
		self.camera_topic_list.append(rospy.get_param("camera/topic_pan_tilt", "/camera_pan_tilt/camera/image_raw"))
		rospy.loginfo(self.camera_topic_list)


if __name__ == "__main__":
	app = QtGui.QApplication(sys.argv)
	AppWindow = CentralUi()
	AppWindow.show()
	sys.exit(app.exec_())
