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
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Joy

DUMMY_FIELD_PREFIX = '__'

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')), (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')), (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)
 
 # sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                 PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}
 
def pointcloud2_to_dtype(cloud_msg):
     '''Convert a list of PointFields to a numpy record datatype.
     '''
     offset = 0
     np_dtype_list = []
     for f in cloud_msg.fields:
         while offset < f.offset:
             # might be extra padding between fields
             np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
             offset += 1
         np_dtype_list.append((f.name, pftype_to_nptype[f.datatype]))
         offset += pftype_sizes[f.datatype]
 
     # might be extra padding between points
     while offset < cloud_msg.point_step:
         np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
         offset += 1
         
     return np_dtype_list


class Scan_msg:

    
	def __init__(self):

		self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)
		self.msg = Twist()
		self.go = False
		self.prev_scan = None
		self.x = 0
		self.z = 0


	def normalize_ranges(self, ranges):
		inds = np.where(np.isnan(ranges))
		ranges[inds] = 10
		return ranges

	def threshold(self, laserscan):
		ranges = self.normalize_ranges(np.array(laserscan.ranges))
		blur = cv2.GaussianBlur(ranges,(5,1),0)
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
	
	def cloud2array(self, points):
		
		gen = pc2.read_points(points, skip_nans=False, field_names=("x", "y", "z"))		
		arr = []
		for point in gen:
			arr.append([point[0], point[1], point[2]])
		return np.nan_to_num(np.array(arr))
	
	def process(self, points):
		scan = self.cloud2array(points)
		scan = scan.reshape(-1)
		print scan.shape
		if self.prev_scan is not None:
			dist = self.convolve(scan)
			#blur = cv2.GaussianBlur(dist,(11,1),0)
			peak = np.argmax(dist)
			point = (float(len(scan)) / 2.0) - float(peak)
			self.z = float(point) / (float(len(scan)) / 4.0) 
			print self.z
		
		self.prev_scan = scan


	def movement(self):


		z = ((6 * self.msg.angular.z) + self.z) / 7
		x = ((14 * self.msg.linear.x) + self.x) / 15

		self.msg.angular.z = z
		self.msg.linear.x = x

		#rospy.loginfo(self.dbgmsg[sect])
		self.pub.publish(self.msg)

	 
	def for_callback_points(self,scan):
		'''Passes laserscan onto function sort which gives the sect 
		variables the proper values.  Then the movement function is run 
		with the class sect variables as parameters.

		Parameter laserscan is received from callback function.'''
		self.process(scan)
		self.movement()
	
	def for_callback_joy(self,joystick):
		print("joy buttons")
		if joystick.buttons[0]:
			self.go = not self.go

			

def call_back_points(scanmsg):
	'''Passes laser scan message to for_callback function of sub_obj.
	Parameter scanmsg is laserscan message.'''
	sub_obj.for_callback_points(scanmsg)

def call_back_joy(joymsg):
	sub_obj.for_callback_joy(joymsg)

def listener():

	rospy.init_node('navigation_sensors')
	rospy.loginfo("Subscriber Starting")
	sub = rospy.Subscriber('/camera/depth/points', PointCloud2, call_back_points)
	sub2 = rospy.Subscriber('/joy', Joy, call_back_joy)
	rospy.spin()

if __name__ == "__main__":
	'''A Scan_msg class object called sub_obj is created and listener
	function is run''' 
	sub_obj = Scan_msg()
	listener()
