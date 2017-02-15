#!/usr/bin/env python
import rospy
import math
import time
import cv2
import numpy as np

import matplotlib.pyplot as plt
from skimage import data
try:
    from skimage import filters
except ImportError:
    from skimage import filter as filters
from skimage import exposure

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy


class Scan_msg:

    
	def __init__(self):

		self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)
		self.msg = Twist()
		self.go = False
		self.prev_scan = None
		self.x = 0
		self.z = 0


	def normalize(self, ranges):
		inds = np.where(np.isnan(ranges))
		ranges[inds] = 15
		return ranges

	def threshold(self, scan):
		blur = cv2.GaussianBlur(scan,(5,1),0)
		thresh = filters.threshold_otsu(blur)
		foreground = blur < thresh
		return blur * foreground
		"""
		blur = cv2.GaussianBlur(ranges,(5,1),0)
		grey = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
		ret,thresh = cv2.threshold(grey,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
		"""
	
	def convolve(self, scan):
		return np.real(np.fft.ifft( np.fft.fft(scan)*np.fft.fft(self.prev_scan) ))
	
	def correlate(self, scan):
		return np.fft.ifft(np.fft.fft(scan) * np.fft.fft(self.prev_scan).conj()).real
	
	def quad(self, scan):
		p = np.polyfit(range(len(scan)), scan, 2)
		return -(p[1]) / (2*p[0])
		
	
	def quadmin(self, scan):
		poly = np.poly1d(np.polyfit(range(len(scan)), scan, 2))
		print poly
		crit = poly.deriv().r
		r_crit = crit[crit.imag==0].real
		test = poly.deriv(2)(r_crit) 
		x_min = r_crit[test>0]
		y_min = poly(x_min)
		return x_min[np.argmin(y_min)]
	
	def quadmax(self, scan):
		poly = np.poly1d(np.polyfit(range(len(scan)), scan, 2))
		crit = poly.deriv().r
		r_crit = crit[crit.imag==0].real
		test = poly.deriv(2)(r_crit) 
		x_max = r_crit[test<0]
		y_max = poly(x_max)
		return x_max[np.argmax(y_max)]
	
	def process(self, laserscan):
		scan = np.array(laserscan.ranges)
		scan = np.nan_to_num(scan)
		#scan = self.normalize(scan)
		#scan = self.threshold(scan)
		if self.prev_scan is not None:
			dist = self.correlate(scan)
			#blur = cv2.GaussianBlur(dist,(19,1),0)
			#peak = self.quadmin(scan)
			peak = self.quad(dist)
			print peak
			point = float(peak) - (float(len(scan)) / 2.0)
			print point
			self.z = float(point) / (float(len(scan)) / 120.0)
			print self.z
		self.prev_scan = scan


	def movement(self):


		z = ((6 * self.msg.angular.z) + self.z) / 7
		x = ((14 * self.msg.linear.x) + self.x) / 15

		self.msg.angular.z = z
		self.msg.linear.x = x

		#rospy.loginfo(self.dbgmsg[sect])
		if self.go:
			self.pub.publish(self.msg)

	 
	def for_callback_laser(self,laserscan):
		'''Passes laserscan onto function sort which gives the sect 
		variables the proper values.  Then the movement function is run 
		with the class sect variables as parameters.

		Parameter laserscan is received from callback function.'''
		self.process(laserscan)
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
