#!/usr/bin/env python

# date: 8th March
# author: Noni Hua
# reference: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_pose/py_pose.html

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist

import math
import time
from matplotlib import pyplot as plt
import os
import smach
import smach_ros
import actionlib
import copy
import tf
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from numpy import cross, eye, dot
from scipy.linalg import expm3, norm
import imutils

returning_points = []

class TemplateMatcher:

    def __init__(self, template_name, threshold = 0.2):

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
        rospy.Subscriber('/cv_camera/image_rect_color', Image, self.webcam_cb)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)

        self.bridge = cv_bridge.CvBridge()
        path = rospy.get_param("/pkg_path")
        self.template = cv2.imread(path + "/img/" + template_name, 0)
        self.template = cv2.Canny(self.template, 50, 200)
        self.th, self.tw =  self.template.shape[:2]
        self.threshold = threshold
        self.status = 'searching'
        self.twist = Twist()
        self.pose = None

    def amcl_cb(self, msg):
        self.pose = msg.pose.pose

    def webcam_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = imutils.resize(img, width = int(img.shape[1] * 0.5))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found = None
        for scale in np.linspace(0.1, 1.0, 10)[::-1]:
            resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
            r = gray.shape[1] / float(resized.shape[1])
            if resized.shape[0] < self.th or resized.shape[1] < self.tw:
                break
            edged = cv2.Canny(resized, 50, 200)
            result = cv2.matchTemplate(edged, self.template, cv2.TM_CCOEFF_NORMED)
            (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
            if found is None or maxVal > found[0]:
                found = (maxVal, maxLoc, r)
        (maxVal, maxLoc, r) = found
        (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
        (endX, endY) = (int((maxLoc[0] + self.tw) * r), int((maxLoc[1] + self.th) * r))
        cv2.rectangle(img, (startX, startY), (endX, endY), (0, 0, 255), 2)
        cv2.imshow('templateresult', img)
        cv2.waitKey(1)

        if maxVal > self.threshold and startX > 640 /3:
            self.status = 'found'

        self.navi()

    def navi(self):
        if self.status == 'searching':
            # cue the dummy searching
            self.twist.angular.z = 0
            self.twist.linear.x = 0.1
            self.cmd_vel_pub.publish(self.twist)
        elif self.status == 'found':
            # mark down current position
            global returning_point
            returning_points.append(self.pose)
            self.status = 'ready2dock'


class OrbTracker:
    def __init__(self, template_name):

        self.K = None # set externally
        self.D = None # set externally

        self.cam_info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.cam_info_cb)
        self.cam_sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.cam_cb)
        self.bridge = cv_bridge.CvBridge()
        path = rospy.get_param("/pkg_path")
        self.template = cv2.imread(path + "/img/"+ template_name, 0)
        self.th, self.tw =  self.template.shape[:2]

        self.imgpts = np.zeros((3, 1, 2), dtype=np.int)
        self.imgpts2 = np.zeros((3, 1, 2), dtype=np.int)

        #nfeatures[, scaleFactor[, nlevels[, edgeThreshold[, firstLevel[, WTA_K[, scoreType[, patchSize]
        self.orb = cv2.ORB_create(300, 1.2, 10, 31, 0, 3, cv2.NORM_HAMMING2, 31)
        self.kp = self.orb.detect(self.template,None)
        self.kp, self.des = self.orb.compute(self.template, self.kp)
        self.des = np.float32(self.des)

        self.eye = np.identity(3)
        self.axis = np.float32([[30,0,0], [0,30,0], [0,0,-30]]).reshape(-1,3)
        self.axis2 = np.float32([[-30,0,0], [0,-30,0], [0,0,30]]).reshape(-1,3)

        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.found = False
        self.rot = None
        self.trans = None

        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
        self.status = 'notyet'

        self.matching_counter = 0
        self.min_match_count = 10

    def cam_info_cb(self, msg):
        self.K = np.array(msg.K).reshape(3,3)
        self.D = np.array(msg.D)

    def cam_cb(self, msg):
        img  = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img2 = gray
        img3 = gray

        orb = cv2.ORB_create(800)
        kp = orb.detect(gray, None)
        kp, des = self.orb.compute(gray, kp)
        des = np.float32(des)

        bf = cv2.BFMatcher()
        matches = bf.match(self.des,des)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m in matches:
            if m.distance < 0.7:
                good.append(m)

        if len(good) > self.min_match_count:
            src_pts = np.float32([ self.kp[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h,w = self.template.shape
            rect = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            rect3d = np.float32([ [0,0,0],[0,h-1,0],[w-1,h-1,0],[w-1,0,0] ]).reshape(-1,1,3)
            rect = cv2.perspectiveTransform(rect,M)

            img2 = cv2.polylines(gray,[np.int32(rect)],True,255,3, cv2.LINE_AA)

            pnp = cv2.solvePnPRansac(rect3d, rect, self.K, self.D)
            rvecs, tvecs, inliers = pnp[1], pnp[2], pnp[3]
            # gives central position
            imgpts, jac = cv2.projectPoints(self.axis + [w/2,h/2,0], rvecs, tvecs, self.K, self.D)
            imgpts2, jac = cv2.projectPoints(self.axis2 + [w/2,h/2,0], rvecs, tvecs, self.K, self.D)
            img3 = self.draw(img3, imgpts, imgpts2, rect)

            self.matching_counter += 1
            if self.matching_counter > 3:
                self.status = 'docking'
                self.navi(rvecs, tvecs / 900)

        else:
            self.matching_counter -= 1
            print "Not enough matches are found - %d/%d" % (len(good),self.min_match_count)
            matchesMask = None
            rect = np.zeros((4, 1, 2), dtype=np.int)
            imgpts = np.zeros((3, 1, 2), dtype=np.int)
            imgpts2 = imgpts

        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                       singlePointColor = None,
                       matchesMask = matchesMask, # draw only inliers
                       flags = 2)

        img3 = cv2.drawMatches(self.template, self.kp, gray, kp, good, None, **draw_params)

        cv2.imshow("orb result", img3)
        k = cv2.waitKey(1) & 0xff

    def navi(self, rvec, tvec):
        if tvec[-1] > 0.2:
            self.twist.angular.z = 0
            self.twist.linear.x = 0.1
            self.cmd_vel_pub.publish(self.twist)
        else:
            self.twist.angular.z = 0
            self.twist.linear.x = 0
            self.cmd_vel_pub.publish(self.twist)
            self.status = 'docked'

    def draw(self, img, imgpts, imgpts2, rect):
        img = self.line(img, tuple((imgpts[0]).astype(int).ravel()), tuple((imgpts2[0]).astype(int).ravel()), (255,255,255), 5)
        img = self.line(img, tuple((imgpts[1]).astype(int).ravel()), tuple((imgpts2[1]).astype(int).ravel()), (150,150,150), 5)
        img = self.line(img, tuple((imgpts[2]).astype(int).ravel()), tuple((imgpts2[2]).astype(int).ravel()), (100,100,100), 5)
        return img

    def line(self, img, p1, p2, c, w):
        return cv2.line(img, tuple(np.maximum(p1, 1)), tuple(np.maximum(p2, 1)), c, w)


if __name__ == "__main__":
    pass
