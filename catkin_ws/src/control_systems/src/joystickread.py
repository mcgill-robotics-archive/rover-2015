#!/usr/bin/env python
#convert linear,angular speed to wheel settings
import rospy #for reading and publishing to topics
from mappingsteer import steer, pointTurn, translationalMotion 
from geometry_msgs.msg import Twist #type of joystick input
from control_systems.msg import SetPoints,Moving,MotionType #type of wheel setting output
from std_msgs.msg import Int8

class JoystickReader(object):
	def __init__(self):
		self.value = [0,0] #Default settings so vehicle should not move
		self.settings = SetPoints() #Type of output
		self.motion = MotionType()
		self.moving = Moving()
		#self.motion.data = 0
		rospy.init_node('joystick_reader') #Name of this node

		#Open publisher - whih publishes to wheel
		self.pubwheels = rospy.Publisher('/wheels',SetPoints,queue_size=10,latch=True)
		#whether or not wheels should be moving
		self.pubmovement = rospy.Publisher('/movement',Moving, queue_size=10,latch=True)
		#Subscribe to the topic "/cmd_vel", and print out output to function
		rospy.Subscriber('/cmd_vel',Twist,self.update_value_settings,queue_size=10)
		#type of motion
		rospy.Subscriber('/cmd_motion',MotionType,self.update_value_motion,queue_size=10)
		
	#update_settings depending on reading from topic
	def update_value_settings(self,msg):
		#read in values from twist
		self.value[0] = msg.linear.x
		self.value[1] = msg.angular.z

		#translational motion
		if self.motion.TRANSLATORY:
			output = translationalMotion(self.value[0],self.value[1])
		#point steering (around middle)
		elif self.motion.POINT == 1:
			output = pointTurn(self.value[1])
		#ackermann steering
		else:
			output = steer(self.value[0],self.value[1])
		
		#Convert output of function to setpoint variable type
		self.moving.move = output['movement']
		self.settings.thetaFL = output['pfsa']
		self.settings.thetaFR = output['sfsa']
		self.settings.thetaRL = output['prsa']
		self.settings.thetaRR = output['srsa']
		self.settings.speedFL = output['pfrv']
		self.settings.speedFR = output['sfrv']
		self.settings.speedML = output['pmrv']
		self.settings.speedMR = output['smrv']
		self.settings.speedRL = output['prrv']
		self.settings.speedRR = output['srrv']


	def update_value_motion(self,msg): 
		#read in values from Int8
		self.motion = msg

	#function publishes
	def run(self):
		#calculate required wheel angles, speeds
		r = rospy.Rate(10)
	#continue endlessly
		while not rospy.is_shutdown():
			#log wheel settings
			rospy.loginfo(self.moving)
			rospy.loginfo(self.settings)
			#publish it
			self.pubwheels.publish(self.settings)
			self.pubmovement.publish(self.moving)
			#10 Hz rate regardless of joystick rate
			r.sleep()

if __name__ == '__main__':
	print "Initializing Node"
	joystickreader1 = JoystickReader()
	print "Running Node"
	joystickreader1.run()
	rospy.spin()
	#try:
		#joystickTranslator = Node()
		#also try to read from joystick 
	#except rospy.ROSInterruptException: pass



#old attempt at subscribing and publishing
	"""
def forwardJoystick():
	x,w=0,0 #Default settings so vehicle does not move if no connection
	input = steer(x,w)
	pub = rospy.Publisher('wheels',SetPoints,queue_size=10)
	sub = rospy.Subscriber('cmd_vel',Twist,queue_size=10)
	rospy.init_node('joystick_reader',anonymous=True,callback)
	r = rospy.Rate(10) #UPDATE FFINAL!
	while not rospy.is_shutdown():
		sub

		#Declare variable to store wheel settings
		settings = SetPoints()
		#Declare 
		settings.thetaFL = input[0]
		settings.thetaFR = input[1]
		settings.thetaRL = input[2]
		settings.thetaRR = input[3]
		settings.speedMW = input[4]
		rospy.loginfo(settings)
		pub.publish(settings)
		r.sleep()
	return 0
"""