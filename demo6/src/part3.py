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

from numpy import cross, eye, dot
from scipy.linalg import expm3, norm

MIN_MATCH_COUNT = 10

def R(axis, theta):
    return expm3(cross(eye(3), axis/norm(axis)*theta))

class Part3:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.cam_info_sub = rospy.Subscriber('/camera/camera_info', CameraInfo, self.info_cb)
        self.img_sub = rospy.Subscriber('/camera/image_rect_color', Image, self.img_cb)

        # Load the target image
        img_path = rospy.get_param("/pkg_path")

        print img_path+ "/img/UA-1C-SOLID.png"
        self.target_image = cv2.imread(img_path + "/img/UA-1C-SOLID.png", 0)

        self.imgpts = np.zeros((3, 1, 2), dtype=np.int)
        self.imgpts2 = np.zeros((3, 1, 2), dtype=np.int)
        # Initiate STAR detector
        self.orb = cv2.ORB_create(200)
        # find the keypoints with ORB
        self.kp = self.orb.detect(self.target_image,None)
        # compute the descriptors with ORB
        self.kp, self.des = self.orb.compute(self.target_image, self.kp)
        self.des = np.float32(self.des)


        self.eye = np.identity(3)
        self.axis = np.float32([[30,0,0], [0,30,0], [0,0,-30]]).reshape(-1,3)
        self.axis2 = np.float32([[-30,0,0], [0,-30,0], [0,0,30]]).reshape(-1,3)


        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.K = None
        self.D = None
        self.found = False
        self.rot = None
        self.trans = None

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

            imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.K, self.D)
            imgpts2, jac = cv2.projectPoints(self.axis2, rvecs, tvecs, self.K, self.D)
            img3 = self.draw(img3, imgpts, imgpts2, rect)

            imgpts, jac = cv2.projectPoints(self.axis + [0,h,0], rvecs, tvecs, self.K, self.D)
            imgpts2, jac = cv2.projectPoints(self.axis2 + [0,h,0], rvecs, tvecs, self.K, self.D)
            img3 = self.draw(img3, imgpts, imgpts2, rect)

            imgpts, jac = cv2.projectPoints(self.axis + [w,0,0], rvecs, tvecs, self.K, self.D)
            imgpts2, jac = cv2.projectPoints(self.axis2 + [w,0,0], rvecs, tvecs, self.K, self.D)
            img3 = self.draw(img3, imgpts, imgpts2, rect)

            imgpts, jac = cv2.projectPoints(self.axis + [w,h,0], rvecs, tvecs, self.K, self.D)
            imgpts2, jac = cv2.projectPoints(self.axis2 + [w,h,0], rvecs, tvecs, self.K, self.D)
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
    rospy.init_node('part3')
    part3 = Part3()
    rospy.spin()
