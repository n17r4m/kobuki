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

class Docker:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.cam_info_sub = rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.info_cb)
        self.img_sub = rospy.Subscriber('camera/rgb/image_rect_color', Image, self.img_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
        self.twist = Twist()
        self.eye = np.identity(3)
        self.axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
        self.objp = np.zeros((6*8,3), np.float32)
        self.objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.K = None
        self.D = None

    def info_cb(self, msg):
        self.K = np.array(msg.K).reshape(3,3)
        self.D = np.array(msg.D)

    def img_cb(self, msg):
        #np_arr = np.fromstring(msg.data, np.uint8)
        #img_np = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        img  = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (8,6))

        if ret == True:
            print "found it"
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), self.criteria)
            z = np.array([0.0,0.0,0.0,0.0,0.0])
            # Find the rotation and translation vectors.
            shits = cv2.solvePnPRansac(self.objp, corners2, self.K, self.D)
            rvecs, tvecs, inliers = shits[1], shits[2], shits[3]
            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.K, self.D)

            img = self.draw(img,corners2,imgpts)

            cv2.imshow('img',img)
            k = cv2.waitKey(1) & 0xff

            self.navi(tvecs, rvecs) # todo
        else:
            cv2.imshow('img',img)
            k = cv2.waitKey(1) & 0xff

    def navi(self, dist, theta):
        print dist, theta
        self.twist.angular.z = 0
        self.twist.linear.x = 0
        if theta[0] < 0:
            self.twist.angular.z = 0.3
        else:
            self.twist.angular.z = -0.3
        if dist[-1] > 20:
            self.twist.linear.x = 0.3
        else:
            self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)


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
