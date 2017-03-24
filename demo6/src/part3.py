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
        self.target_image = cv2.imread("/home/martin/Documents/Classes/Cmput412/src/demo6/img/UA-1C-SOLID.png", 0)
        
        
        # Initiate STAR detector
        self.orb = cv2.ORB_create()
        # find the keypoints with ORB
        self.kp = self.orb.detect(self.target_image,None)
        # compute the descriptors with ORB
        self.kp, self.des = self.orb.compute(self.target_image, self.kp)
        self.des = np.float32(self.des)


        self.eye = np.identity(3)
        self.axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
        
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


        orb = cv2.ORB_create()
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
            rect3d = np.float32([ [0,0,0],[0,h-1,0],[w-1,h-1,0],[w-1,0,0] ])
            rect = cv2.perspectiveTransform(rect,M)
            #M = M * (self.eye * 0.25)
            

            img2 = cv2.polylines(gray,[np.int32(rect)],True,255,3, cv2.LINE_AA)
            
    
            """
            pts = np.array([kp[idx].pt for idx in range(len(kp))],dtype=np.float).reshape(-1,1,2)
            ipts = np.array([self.kp[idx].pt for idx in range(len(self.kp))],dtype=np.float).reshape(-1,1,2)
            """
            
            dst2 = dst_pts[matchesMask].reshape(dst_pts.shape[0], 2)
            src2 = src_pts[matchesMask].reshape(dst_pts.shape[0], 2)
            
            print dst2.shape, len(dst2), len(src2)
            
            
            
            pnp = cv2.solvePnPRansac(rect3d, rect, self.K, self.D)
            rvecs, tvecs, inliers = pnp[1], pnp[2], pnp[3]
            print pnp
            imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.K, self.D)
            img3 = self.draw(img3, rect, imgpts)
            
        else:
            print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
            matchesMask = None
        
        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                       singlePointColor = None,
                       matchesMask = matchesMask, # draw only inliers
                       flags = 2)
        
        img3 = cv2.drawMatches(self.target_image, self.kp, gray, kp, good, None, **draw_params)
        #plt.imshow(img3, 'gray'),plt.show()
        cv2.imshow("result", img3)
            
            
            
        
        

        """
        ret, corners = cv2.findChessboardCorners(gray, (8,6), cv2.CALIB_CB_FAST_CHECK)

        if ret == True: #and not self.found:
            print "found it"
            #self.found = True
            corners2 = cv2.cornerSubPix(gray,corners,(3,3),(-1,-1), self.criteria)
            #z = np.array([0.0,0.0,0.0,0.0,0.0])
            # Find the rotation and translation vectors.
            shits = cv2.solvePnPRansac(self.objp, corners2, self.K, self.D)#, cv2.SOLVEPNP_UPNP)
            rvecs, tvecs, inliers = shits[1], shits[2], shits[3]
            self.rot = rvecs
            self.trans = tvecs
            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.K, self.D)

            img = self.draw(img,corners2,imgpts)

            cv2.imshow('img',img)
            k = cv2.waitKey(1) & 0xff

        else:
            tvecs = np.zeros((3,))
            rvecs = np.zeros((3,))

        """

        k = cv2.waitKey(1) & 0xff


    def draw(self, img, corners, imgpts):
        corner = tuple([np.mean(corners[:,:,0]), np.mean(corners[:,:,1])])
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
        return img


if __name__ == "__main__":
    rospy.init_node('part3')
    part3 = Part3()
    rospy.spin()
