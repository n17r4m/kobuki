#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import math
import time

class Track:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.cam_info_sub = rospy.Subscriber('/cv_camera/camera_info', CameraInfo, self.info_cb)
		self.img_sub = rospy.Subscriber('/cv_camera/image_raw', Image, self.img_cb)
		self.marker = rospy.Subscriber('visualization_marker', Marker, self.marker_cb)
		self.axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,-0.1],[0,0,0]]).reshape(-1,3)
		self.rvecs = None
		self.tvecs = None
		self.D = None
		self.K = None
		#self.imgpts = None
		self.img = None

	def info_cb(self, msg):
		self.K = np.array(msg.K).reshape(3,3)
		self.D = np.array(msg.D)

	def img_cb(self, msg):
		self.img  = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		self.img  = self.img[50:500]
		#cv2.imshow('image',self.img)
		#cv2.waitKey()

	def marker_cb(self, msg):
		if (msg.id == 4):
			px = msg.pose.position.x
			py = msg.pose.position.y
			pz = msg.pose.position.z
			ox = msg.pose.orientation.x
			oy = msg.pose.orientation.y
			oz = msg.pose.orientation.z
			ow = msg.pose.orientation.w

			tvecs = np.array([px, py, pz])

			angle = 2 * math.acos(ow)
			x = ox / math.sqrt(1 - ow*ow)
			y = oy / math.sqrt(1 - ow*ow)
			z = oz / math.sqrt(1 - ow*ow)
			ratio = math.sqrt(x*x + y*y + z*z)

			#normalize them
			x = x / ratio*angle
			y = y / ratio*angle
			z = z / ratio*angle
			rvecs = np.array([x, y, z])

			#rvecs = msg.getAngle().getAxis() bullet
			#print(rvecs)
			imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.K, self.D)
			print(imgpts)

			image = cv2.line(self.img, tuple(imgpts[3].ravel()), tuple(imgpts[0].ravel()), (255,0,0), 5)
			image = cv2.line(self.img, tuple(imgpts[3].ravel()), tuple(imgpts[1].ravel()), (0,255,0), 5)
			image = cv2.line(self.img, tuple(imgpts[3].ravel()), tuple(imgpts[2].ravel()), (0,0,255), 5)
		cv2.imshow('img',self.img)
		k = cv2.waitKey(1) & 0xff




if __name__ == "__main__":
	rospy.init_node('track')
	track = Track()
	rospy.spin()
