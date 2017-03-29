#!/usr/bin/env python

# date: 8th March
# author: Noni Hua
# reference: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_pose/py_pose.html

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
import math
import time
from matplotlib import pyplot as plt
import os
import smach
import smach_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from numpy import cross, eye, dot
from scipy.linalg import expm3, norm
import imutils
import threading

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

def set_interval(func, sec):
    def func_wrapper():
        set_interval(func, sec)
        func()
    t = threading.Timer(sec, func_wrapper)
    t.daemon = True
    t.start()
    return t


class OrbTracker(object):
    def __init__(self, template_filename, min_match_count = 10):
        
        self.K = None # set externally
        self.D = None # set externally 
        
        self.bridge = cv_bridge.CvBridge()
        path = rospy.get_param("/pkg_path")
        self.name = template_filename
        self.template = cv2.imread(path + "/img/" + template_filename, 0)
        self.th, self.tw =  self.template.shape[:2]
        self.min_match_count = min_match_count
        self.imgpts = np.zeros((3, 1, 2), dtype=np.int)
        self.imgpts2 = np.zeros((3, 1, 2), dtype=np.int)
        
        self.orb = cv2.ORB_create(200)
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
        
        
    def process(self, msg, found_cb):
        
        img  = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        img = imutils.resize(img, width = int(img.shape[1] * 1))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img2 = gray
        img3 = gray
        
        orb = cv2.ORB_create(800)
        kp = orb.detect(gray, None)
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

            #dst2 = dst_pts[matchesMask].reshape(dst_pts.shape[0], 2)
            #src2 = src_pts[matchesMask].reshape(dst_pts.shape[0], 2)
            #src2 = np.concatenate(src2, [0], axis=1)
            pnp = cv2.solvePnPRansac(rect3d, rect, self.K, self.D)
            #pnp = cv2.solvePnPRansac(src2, dst2, self.K, self.D)
            rvecs, tvecs, inliers = pnp[1], pnp[2], pnp[3]
            # gives central position
            imgpts, jac = cv2.projectPoints(self.axis + [w/2,h/2,0], rvecs, tvecs, self.K, self.D)
            imgpts2, jac = cv2.projectPoints(self.axis2 + [w/2,h/2,0], rvecs, tvecs, self.K, self.D)
            img3 = self.draw(img3, imgpts, imgpts2, rect)
        
            found_cb(rvecs, tvecs, self.name)
            
        else:
            #print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
            matchesMask = None
            rect = np.zeros((4, 1, 2), dtype=np.int)
            imgpts = np.zeros((3, 1, 2), dtype=np.int)
            imgpts2 = imgpts
            
        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                       singlePointColor = None,
                       matchesMask = matchesMask, # draw only inliers
                       flags = 2)
        
        #img3 = cv2.cvtColor(img3, cv2.COLOR_GRAY2BGR)
        img3 = cv2.drawMatches(self.template, self.kp, gray, kp, good, None, **draw_params)
        
        cv2.imshow(self.name, img3)
        k = cv2.waitKey(1) & 0xff
        
    def draw(self, img, imgpts, imgpts2, rect):
        img = self.line(img, tuple((imgpts[0]).astype(int).ravel()), tuple((imgpts2[0]).astype(int).ravel()), (255,255,255), 5)
        img = self.line(img, tuple((imgpts[1]).astype(int).ravel()), tuple((imgpts2[1]).astype(int).ravel()), (150,150,150), 5)
        img = self.line(img, tuple((imgpts[2]).astype(int).ravel()), tuple((imgpts2[2]).astype(int).ravel()), (100,100,100), 5)
        return img
    
    def line(self, img, p1, p2, c, w):
        return cv2.line(img, tuple(np.maximum(p1, 1)), tuple(np.maximum(p2, 1)), c, w)
        
        

class TemplateMatcher(object):
    
    def __init__(self, template_filename, threshold = 0.2):
        self.bridge = cv_bridge.CvBridge()
        path = rospy.get_param("/pkg_path")
        self.name = template_filename
        self.template = cv2.imread(path + "/img/" + template_filename, 0)
        self.template = cv2.Canny(self.template, 50, 200)
        self.th, self.tw =  self.template.shape[:2]
        self.threshold = threshold

    def process(self, msg, found_cb):
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
        cv2.imshow(self.name, img)
        cv2.waitKey(1)
        
        if maxVal > self.threshold:
            found_cb(startX, startY, endX, endY, maxVal, self.name)
        

class Comp4:
    def __init__(self):
        
        self.UA_Template_Tracker = TemplateMatcher("ua_small.png", 0.2)
        self.AR_Template_Tracker = TemplateMatcher("ar_small.png", 0.3)
        
        self.UA_ORB_Tracker = OrbTracker("ua.png")
        self.AR_ORB_Tracker = OrbTracker("ar.png")
        
        self.webcam_info_sub = rospy.Subscriber('/cv_camera/camera_info', CameraInfo, self.webcam_info_cb)
        self.webcam_sub = rospy.Subscriber('/cv_camera/image_rect_color', Image, self.webcam_cb)
        
        self.kinect_info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.kinect_info_cb)
        self.kinect_sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.kinect_cb)

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)

        """
        States are: 
            searching (wandering around, wall crawling)
            turning   (turning towards wall)
            locking   (moving forward, waiting for rvecs & tvecs)
            docking   (moving towards goal computed from locking)
        """
        self.state = "locking"
        self.found = "ua" 
        self.vec_measures = 0
        self.tvecs = None
        self.rvecs = None
        
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
        self.twist = Twist()
        self.pose = None
        
        self.move = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move.wait_for_server()

    
    # SIDE CAMERA (webcam)
    
    def webcam_info_cb(self, msg):
        pass

    def webcam_cb(self, msg):
        if self.state == "searching":
            self.UA_Template_Tracker.process(msg, self.found_webcam_match)
            self.AR_Template_Tracker.process(msg, self.found_webcam_match)
    
    def found_webcam_match(self, x1, y1, x2, y2, name):
        print "[webcam] FOUND IT:", x1, y1, x2, y2, name
        if x1 > 150:
            if name == "ua_small.png":
                self.found = "ua"
            if name == "ar_small.png":
                self.found = "ar"
            self.template_found_at = [x1, y1, x2, y2]
            self.state = "turning"
    
    # FRONT CAMERA (kinect)
    
    def kinect_info_cb(self, msg):
        self.UA_ORB_Tracker.K = np.array(msg.K).reshape(3,3)
        self.AR_ORB_Tracker.K = np.array(msg.K).reshape(3,3)
        self.UA_ORB_Tracker.D = np.array(msg.D)
        self.AR_ORB_Tracker.D = np.array(msg.D)
    
    def kinect_cb(self, msg):
        if self.state == "locking":
            if self.found == "ua":
                self.UA_ORB_Tracker.process(msg, self.found_kinect_match)
            if self.found == "ar":
                self.AR_ORB_Tracker.process(msg, self.found_kinect_match)
    
    def found_kinect_match(self, rvecs, tvecs, name):
        measures_needed = 10
        if self.vec_measures == 0:
            self.rvecs = (1/measures_needed) * rvecs
            self.tvecs = (1/measures_needed) * tvecs
        self.vec_measures += 1
        if self.vec_measures < measures_needed:
            self.rvecs += (1/measures_needed) * rvecs
            self.tvecs += (1/measures_needed) * tvecs
        else:
            self.state = "locking" # should be docking
            self.vec_measures = 0
            
        print "[kinect] FOUND IT", rvecs, tvecs, name
    
    # ODOMETRY

    def tick(self):
        getattr(self, self.state)()
        
    def amcl_cb(self, msg):
        self.pose = pose.pose.pose
        

    def searching(self):
        self.twist.angular.z = 0
        self.twist.linear.x = 0.1
        self.cmd_vel_pub.publish(self.twist)
    
    def turning(self):
        goal = self.pose
        
        self.move.send_goal(goal)
        self.move.wait_for_result()
        
    
    def locking(self, x1, y1, x2, y2):
        pass
        # takes template tracking position and lock the marker to the center of screen
        self.cmd_vel_pub.publish(self.twist)
    
    def docking(self, tvec, rvec):
        print tvec
        print rvec
        #self.twist.angular.z = - theta[0]  * 180 / 3.1415 / 10
        #self.twist.linear.x = (dist[-1]- 15) / 100
        z = 0
        if rvec[0] > 0.2:
            tvec[0] -= 6
        elif rvec[0] < -0.2:
            tvec[0] += 6

        if 0 > tvec[0]:
            z = 0.2
        elif 0 < tvec[0]:
            z = -0.2
        else:
            z = 0


        if tvec[-1] > 10:
            x = 0.2
        else:
            x = 0

        self.twist.angular.z = (3*self.twist.angular.z + z) / 4
        self.twist.linear.x = (3*self.twist.linear.x + x) / 4

        self.cmd_vel_pub.publish(self.twist)

        
    def sound_beep():
        
        soundhandle = SoundClient()  # blocking = False by default
        rospy.sleep(0.5)  # Ensure publisher connection is successful.
    
        sound_beep = soundhandle.say("beep")

        sleep(1)
        soundhandle.stopAll()

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.header.stamp = rospy.Time.now()
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose


if __name__ == "__main__":
    rospy.init_node('comp4')
    comp4 = Comp4()
    set_interval(comp4.tick, 0.1)
    rospy.spin()
