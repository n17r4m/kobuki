#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

class Scan_msg:

    
	def __init__(self):
		'''Initializes an object of this class.

		The constructor creates a publisher, a twist message.
		3 integer variables are created to keep track of where obstacles exist.
		3 dictionaries are to keep track of the movement and log messages.'''
		self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)
		self.msg = Twist()
		self.go = False
		self.sect_1 = 0
		self.sect_2 = 0
		self.sect_3 = 0
		self.sect_4 = 0
		fast = 1
		medi = 0.5
		slow = 0.2
		stop = 0
		self.move = {
			   0: (math.sin(time.time()), fast),
			   1: (-medi, medi),
			  10: (fast*self.rdir(), stop),
			  11: (-fast, slow),
			 100: ( medi, medi),
			 101: ( stop, slow),
			 110: (-fast, stop),
			 111: (-fast, stop),
			"stop": (stop, stop)
		}


	def rdir(self):
		return max(-1, min(1, 100.0*math.sin(time.time()*10000)))


	def reset_sect(self):
		'''Resets the below variables before each new scan message is read'''
		self.sect_1 = 0
		self.sect_2 = 0
		self.sect_3 = 0
		self.sect_4 = 0

	def sort(self, laserscan):
		'''Goes through 'ranges' array in laserscan message and determines 
		where obstacles are located. The class variables sect_1, sect_2, 
		and sect_3 are updated as either '0' (no obstacles within 0.7 m)
		or '1' (obstacles within 0.7 m)

		Parameter laserscan is a laserscan message.'''
		entries = len(laserscan.ranges)
		for entry in range(0,entries):
			if -0.1 < laserscan.ranges[entry] < 1.0:
				"""
				if (0 < entry < entries*(1.0/4.0)):
					self.sect_1 = 1
				elif (entries*(1.0/4.0) < entry < entries*(1.0/2.0)):
					self.sect_2 = 1
				elif (entries*(1.0/2.0) < entry < entries*(3.0/4.0)):
					self.sect_3 = 1
				elif (entries*(3.0/4.0) < entry < entries):
					self.sect_4 = 1
				"""
				if (0 < entry < entries*(1.0/3.0)):
					self.sect_1 = 1
				elif (entries*(1.0/3.0) < entry < entries*(2.0/3.0)):
					self.sect_2 = 1
				elif (entries*(2.0/3.0) < entry < entries):
					self.sect_3 = 1

	def movement(self):
		'''Uses the information known about the obstacles to move robot.

		Parameters are class variables and are used to assign a value to
		variable sect and then	set the appropriate angular and linear 
		velocities, and log messages.
		These are published and the sect variables are reset.'''
		s2 = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3))
		if self.go:
			#sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3) + str(self.sect_4))
			sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3))
		else:
			sect = "stop" 
			
		
		rospy.loginfo("Sect = " + str(s2)) 

		z = ((6 * self.msg.angular.z) + self.move[sect][0] * 2) / 7
		x = ((14 * self.msg.linear.x) + self.move[sect][1]) / 15

		self.msg.angular.z = z
		self.msg.linear.x = x


		self.pub.publish(self.msg)

		self.reset_sect()
	 
	def for_callback_laser(self,laserscan):
		'''Passes laserscan onto function sort which gives the sect 
		variables the proper values.  Then the movement function is run 
		with the class sect variables as parameters.

		Parameter laserscan is received from callback function.'''
		self.sort(laserscan)
		self.movement()
	
	def for_callback_joy(self,joystick):
		print("joy buttons")
		if joystick.buttons[0]:
			self.go = not self.go

			

def call_back_laser(scanmsg):
	'''Passes laser scan message to for_callback function of sub_obj.
	Parameter scanmsg is laserscan message.'''
	sub_obj.for_callback_laser(scanmsg)

def call_back_joy(joymsg):
	'''Passes joystick message to for_callback function of sub_obj.
	Parameter joymsg is joystick message.'''
	sub_obj.for_callback_joy(joymsg)

def listener():
	'''Initializes node, creates subscriber, and states callback 
	function.'''
	rospy.init_node('navigation_sensors')
	rospy.loginfo("Subscriber Starting")
	sub = rospy.Subscriber('/scan', LaserScan, call_back_laser)
	sub2 = rospy.Subscriber('/joy', Joy, call_back_joy)
	rospy.spin()

if __name__ == "__main__":
	'''A Scan_msg class object called sub_obj is created and listener
	function is run''' 
	sub_obj = Scan_msg()
	listener()
