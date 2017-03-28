#!/usr/bin/env python

# date: 8th March
# author: Noni Hua
# reference: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_pose/py_pose.html

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import math
import time
from matplotlib import pyplot as plt
import os
import smach
import smach_ros

from numpy import cross, eye, dot
from scipy.linalg import expm3, norm
import imutils

MIN_MATCH_COUNT = 10

def R(axis, theta):
	return expm3(cross(eye(3), axis/norm(axis)*theta))

# define state Foo
class TemplateMatcher(object):
	def __init__(self, template_filename):
		self.bridge = cv_bridge.CvBridge()
		path = rospy.get_param("/pkg_path")
		self.name = template_filename
		self.template = cv2.imread(path + "/img/" + template_filename, 0)
		self.template = cv2.Canny(self.template, 50, 200)
		self.th, self.tw =  self.template.shape[:2]

	def process(self, msg, onFoundFn):
		img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		
		found = None
		# loop over the scales of the image
		for scale in np.linspace(0.2, 1.0, 20)[::-1]:
			# resize the image according to the scale, and keep track
			# of the ratio of the resizing
			resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
			r = gray.shape[1] / float(resized.shape[1])
			
			# if the resized image is smaller than the template, then break
			# from the loop
			if resized.shape[0] < self.th or resized.shape[1] < self.tw:
				break
			
			# detect edges in the resized, grayscale image and apply template
			# matching to find the template in the image
			edged = cv2.Canny(resized, 50, 200)
			result = cv2.matchTemplate(edged, self.template, cv2.TM_CCOEFF_NORMED)
			(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
		
			# if we have found a new maximum correlation value, then update
			# the bookkeeping variable
			if found is None or maxVal > found[0]:
				found = (maxVal, maxLoc, r)
		
		# unpack the bookkeeping varaible and compute the (x, y) coordinates
		# of the bounding box based on the resized ratio
		(maxVal, maxLoc, r) = found
		
		print maxVal
		(startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
		(endX, endY) = (int((maxLoc[0] + self.tw) * r), int((maxLoc[1] + self.th) * r))
		
		# draw a bounding box around the detected result and display the image
		image = self.template.copy()
		cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
		cv2.imshow(self.name, image)
		cv2.waitKey(1)




class Comp4:
	def __init__(self):
		
		self.UA_Tracker = TemplateMatcher("ua.png")
		#self.AR_Tracker = TemplateMatcher("ar.png")
	
		self.webcam_info_sub = rospy.Subscriber('/cv_camera/camera_info', CameraInfo, self.webcam_info_cb)
		self.webcam_sub = rospy.Subscriber('/cv_camera/image_rect_color', Image, self.webcam_cb)

		
		self.imgpts = np.zeros((3, 1, 2), dtype=np.int)
		self.imgpts2 = np.zeros((3, 1, 2), dtype=np.int)
		# Initiate STAR detector
		#self.orb = cv2.ORB_create(200)
		# find the keypoints with ORB
		#self.kp = self.orb.detect(self.target_image,None)
		# compute the descriptors with ORB
		#self.kp, self.des = self.orb.compute(self.target_image, self.kp)
		#self.des = np.float32(self.des)


		self.eye = np.identity(3)
		self.axis = np.float32([[30,0,0], [0,30,0], [0,0,-30]]).reshape(-1,3)
		self.axis2 = np.float32([[-30,0,0], [0,-30,0], [0,0,30]]).reshape(-1,3)
		
		
		self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
		self.webcam_K = None
		self.webcam_D = None
		self.found = False
		self.rot = None
		self.trans = None

	
	# SIDE CAMERA (webcam)
	
	def webcam_info_cb(self, msg):
		self.webcam_K = np.array(msg.K).reshape(3,3)
		self.webcam_D = np.array(msg.D)

	def webcam_cb(self, msg):
		self.UA_Tracker.process(msg, self.found_webcam_match)
	
	def found_webcam_match(self):
		print "FOUND IT!"
	
	# FRONT CAMERA (kinect)
	
	def info_cb(self, msg):
		self.K = np.array(msg.K).reshape(3,3)
		self.D = np.array(msg.D)
	
	def img_cb(self, msg):
		
		img  = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		img  = img[50:500]
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		img2 = gray
		img3 = gray


		orb = cv2.ORB_create(800)
		kp = orb.detect(gray,None)
		kp, des = self.orb.compute(gray, kp)
		des = np.float32(des)

		
		FLANN_INDEX_KDTREE = 0
		index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
		search_params = dict(checks = 50)

		flann = cv2.FlannBasedMatcher(index_params, search_params)
		matches = flann.knnMatch(self.des, des, k=2)
		
		
		# store all the good matches as per Lowe's ratio test.
		good = []
		for m,n in matches:
			if m.distance < 0.75*n.distance:
				good.append(m)

		if len(good)>MIN_MATCH_COUNT:
			src_pts = np.float32([ self.kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
			dst_pts = np.float32([ kp[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

			M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
			matchesMask = mask.ravel().tolist()

			h,w = self.target_image.shape #gray.shape
			rect = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
			rect3d = np.float32([ [0,0,0],[0,h-1,0],[w-1,h-1,0],[w-1,0,0] ]).reshape(-1,1,3)
			rect = cv2.perspectiveTransform(rect,M)


			img2 = cv2.polylines(gray,[np.int32(rect)],True,255,3, cv2.LINE_AA)
			
			
			dst2 = dst_pts[matchesMask].reshape(dst_pts.shape[0], 2)
			src2 = src_pts[matchesMask].reshape(dst_pts.shape[0], 2)
			#src2 = np.concatenate(src2, [0], axis=1)
			
			
			pnp = cv2.solvePnPRansac(rect3d, rect, self.K, self.D)
			#pnp = cv2.solvePnPRansac(src2, dst2, self.K, self.D)
			rvecs, tvecs, inliers = pnp[1], pnp[2], pnp[3]
			
			# gives central position
			imgpts, jac = cv2.projectPoints(self.axis + [w/2,h/2,0], rvecs, tvecs, self.K, self.D)
			imgpts2, jac = cv2.projectPoints(self.axis2 + [w/2,h/2,0], rvecs, tvecs, self.K, self.D)
			img3 = self.draw(img3, imgpts, imgpts2, rect)
			
			
		else:
			print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
			matchesMask = None
			rect = np.zeros((4, 1, 2), dtype=np.int)
			imgpts = np.zeros((3, 1, 2), dtype=np.int)
			imgpts2 = imgpts
			
		
		draw_params = dict(matchColor = (0,255,0), # draw matches in green color
					   singlePointColor = None,
					   matchesMask = matchesMask, # draw only inliers
					   flags = 2)
		
		#img3 = cv2.cvtColor(img3, cv2.COLOR_GRAY2BGR)
		
		img3 = cv2.drawMatches(self.target_image, self.kp, gray, kp, good, None, **draw_params)
		
		
		cv2.imshow("result", img3)
		

		k = cv2.waitKey(1) & 0xff
		
	def navi(self, tvec, rvec):
        print tvec
        print rvec
        #self.twist.angular.z = - theta[0]  * 180 / 3.1415 / 10
        #self.twist.linear.x = (dist[-1]- 15) / 100
        z = 0
        if theta[0] > 0.2:
            dist[0] -= 6
        elif theta[0] < -0.2:
            dist[0] += 6

        if 0 > dist[0]:
            z = 0.2
        elif 0 < dist[0]:
            z = -0.2
        else:
            z = 0


        if dist[-1] > 10:
            x = 0.2
        else:
            x = 0

        self.twist.angular.z = (3*self.twist.angular.z + z) / 4
        self.twist.linear.x = (3*self.twist.linear.x + x) / 4

        self.cmd_vel_pub.publish(self.twist)


	def draw(self, img, imgpts, imgpts2, rect):
		
		offset = np.array([0,0]) #np.absolute((rect[2] - rect[0]) / 2) # np.array([self.target_image.shape[0], self.target_image.shape[1]/2])
		#mid = tuple(np.array([np.mean(corners[:,:,0]), np.mean(corners[:,:,1])]).astype(int) + offset)
		img = self.line(img, tuple((imgpts[0] + offset).astype(int).ravel()), tuple((imgpts2[0] + offset).astype(int).ravel()), (255,255,255), 5)
		img = self.line(img, tuple((imgpts[1] + offset).astype(int).ravel()), tuple((imgpts2[1] + offset).astype(int).ravel()), (150,150,150), 5)
		img = self.line(img, tuple((imgpts[2] + offset).astype(int).ravel()), tuple((imgpts2[2] + offset).astype(int).ravel()), (100,100,100), 5)
		
		"""
		mid = tuple(np.array([np.mean(corners[:,:,0]), np.mean(corners[:,:,1])]).astype(int))
		img = cv2.line(img, mid, tuple((imgpts[0]).astype(int).ravel()), (255,0,0), 5)
		img = cv2.line(img, mid, tuple((imgpts[1]).astype(int).ravel()), (0,255,0), 5)
		img = cv2.line(img, mid, tuple((imgpts[2]).astype(int).ravel()), (0,0,255), 5)
		"""
		return img
	
	def line(self, img, p1, p2, c, w):
		return cv2.line(img, tuple(np.maximum(p1, 1)), tuple(np.maximum(p2, 1)), c, w)

if __name__ == "__main__":
	rospy.init_node('comp4')
	comp4 = Comp4()
	rospy.spin()
