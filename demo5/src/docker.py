#!/usr/bin/env python

# date: 8th March
# author: Noni Hua
# reference: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_pose/py_pose.html

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Docker:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.img_sub = rospy.Subscriber('camera/rgb/image_rect_color', Image, self.img_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
        self.twist = Twist()
        self.eye = np.identity(3)
        self.axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
        self.objp = np.zeros((6*7,3), np.float32)
        self.objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    def img_cb(self, msg):
        img  = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (8,6), None)
        
        if ret == True:
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), self.criteria)
            
            # Find the rotation and translation vectors.
            rvecs, tvecs, inliers = cv2.solvePnPRansac(self.objp, corners2, self.eye)
    
            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.eye)
    
            img = self.draw(img,corners2,imgpts)
            cv2.imshow('img',img)
            k = cv2.waitKey(0) & 0xff
            self.navi(0, 0) # todo
        else:
            cv2.imshow('img',img)
            k = cv2.waitKey(0) & 0xff
    
    def navi(self, dist, theta):
        pass
        
    def draw(self, img, corners, imgpts):
        corner = tuple(corners[0].ravel())
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
        return img


if __name__ == "__main__":
    rospy.init_node('docker')
    docker = Docker()
    rospy.spin()