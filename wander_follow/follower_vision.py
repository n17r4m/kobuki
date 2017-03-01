#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Image_msg:
    def __init__ (self):
        self.bridge = CvBridge()
        self.Initialized = False
        self._tracking = False
        self.reattempt = False
        self.go = True
        self.prev_scan = None
        self.minrange = 0.2

        self.roi_hist = None

        self.msg = Twist()
        self.z = 0
        self.x = 0

        self.roi_center = None
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        self.ang = {0:0, 001:0.1, 10:.0, 11:.1, 100:-0.1, 101:-0.1, 110:-.3, 111:-.0, "stop": 0}
        self.fwd = {0:0, 001:0.1, 10:.3, 11:0.1, 100:0.1,101:0,110:0.1,111:-0.2, "stop": 0}

        self.laser_sect_1 = 0
        self.laser_sect_2 = 0
        self.laser_sect_3 = 0
        self.laser_ang = {0:0, 001:-1, 10:-3, 11:-3, 100:1, 101:3, 110:3, 111:3, "stop": 0}
        self.laser_fwd = {0:.3, 1:-0.1, 10:0, 11:-0.1, 100:-0.1,101:0,110:-0.1,111:0, "stop": 0}
        self.dbgmsg = {0: 'Move forward', 1: 'Veer right', 10: 'Move forward', 11: 'Veer right', 100: 'Veer left', 101: 'Confused', 110: 'Veer left', 111: 'Veer right', "stop": "Stopped"}

    def run(self):
        rospy.init_node('tracker')
        rate = rospy.Rate(10) # 10hz
        self.test_pub = rospy.Publisher('/testing', Image, queue_size = 10)
        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 10)

        self.img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.img_callback)
        self.sub2 = rospy.Subscriber('/joy', Joy, self.for_callback_joy)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        while not rospy.is_shutdown():
            rate.sleep()

    def reset_sect(self):
    	'''Resets the below variables before each new scan message is read'''
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        self.laser_sect_1 = 0
        self.laser_sect_2 = 0
        self.laser_sect_3 = 0

    def sort(self, laserscan):
    	entries = len(laserscan.ranges)
    	for entry in range(0,entries):
    		if 0.4 < laserscan.ranges[entry] < 0.8:
    			self.laser_sect_1 += 1 if (0 < entry < entries/3) else 0
    			self.laser_sect_2 += 1 if (entries/3 < entry < entries/2) else 0
    			self.laser_sect_3 += 1 if (entries/2 < entry < entries) else 0
    	rospy.loginfo("sort complete,sect_1: " + str(self.sect_1) + " sect_2: " + str(self.sect_2) + " sect_3: " + str(self.sect_3))

    def movement(self, vision = True):
        if vision:
            if self.go:
                sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3))
            else:
                sect = "stop"

        	rospy.loginfo("Sect = " + str(sect))
            z = ((6 * self.msg.angular.z) + self.ang[sect]) / 7
            x = ((14 * self.msg.linear.x) + self.fwd[sect]) / 15

            self.msg.angular.z = z
            self.msg.linear.x = x
            rospy.loginfo(self.dbgmsg[sect])
            print self.msg.linear.x, self.msg.angular.z
            self.vel_pub.publish(self.msg)

            self.reset_sect()
        else:
            if self.go:
                laser_sect = int(str(self.laser_sect_1) + str(self.laser_sect_2) + str(self.laser_sect_3))
            else:
                laser_sect = "stop"

        	rospy.loginfo("Sect = " + str(laser_sect))
            z = ((6 * self.msg.angular.z) + self.laser_ang[laser_sect]) / 7
            x = ((14 * self.msg.linear.x) + self.laser_fwd[laser_sect]) / 15

            self.msg.angular.z = z
            self.msg.linear.x = x

            rospy.loginfo(self.dbgmsg[laser_sect])
            print self.msg.linear.x, self.msg.angular.z
            self.vel_pub.publish(self.msg)

            self.reset_sect()

    def camshift(self,img):
        print img.shape
        r,h,c,w = 200,240,490,400
        track_window = (r,h,c,w)
        if not self.Initialized:
            roi = img
            hsv_roi =  cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            inrange = cv2.inRange(hsv_roi, np.array((0.,0.,0.)), np.array((250.,60.,60.)))
            mask = cv2.erode(inrange, np.ones((5, 5), np.uint8), iterations=1)
            mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m10']/M['m00'])
                cv2.circle(image), (cx,cy), 20, (0,0,255), -1)
            self.roi_hist = cv2.calcHist([hsv_roi],[0],mask,[180],[0,180])
            cv2.normalize(self.roi_hist,self.roi_hist,0,255,cv2.NORM_MINMAX)
            self.Initialized = True

        term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 5, 1 )

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv],[0],self.roi_hist,[0,180],1)
        # apply meanshift to get the new location
        ret, track_window = cv2.CamShift(dst, track_window, term_crit)
        # Draw it on image
        if ret[2] != 0:
            pts = cv2.boxPoints(ret)
            pts = np.int0(pts)
            cv2.rectangle(img, (r,h), (c,w), (0,255,0), 2)
            dst = cv2.polylines(img,[pts],True, 255,2)
            self.roi_center = ((pts[0][0] + pts[1][0])/2 + (pts[3][0] + pts[2][0])/2)/2
            print self.roi_center

            if self.minrange > 0.4:
                self.sect_1 += 1 if (self.roi_center < 640/3) else 0
                self.sect_2 += 1 if (640/3 < self.roi_center < 640/3 * 2) else 0
                self.sect_3 += 1 if (640/3 * 2 < self.roi_center) else 0
                rospy.loginfo("sort complete,sect_1: " + str(self.sect_1) + " sect_2: " + str(self.sect_2) + " sect_3: " + str(self.sect_3))
                self.movement()

                return dst
            else:
                rospy.loginfo("too close")
                self.sect_1 = self.sect_2 = self.sect_3 = 1
                self.movement()
                return dst


        else:
            rospy.loginfo("searching")
            self.Initialized = False
            self.movement(vision = False)

            return img


    def scan_callback(self, data):
        data_t = np.nan_to_num(np.array(data.ranges))
        self.minrange = min(data_t) + 0.2
        print "min range = ", self.minrange
        self.sort(data)

    def img_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        processed = self.camshift(img)
        #processed = self.HOG(img)
        img_to_ros = self.bridge.cv2_to_imgmsg(processed, 'rgb8')
        self.test_pub.publish(img_to_ros)

    def for_callback_joy(self,joystick):
    	print("joy buttons")
    	if joystick.buttons[0]:
    		self.go = not self.go

if __name__ == "__main__":
    hub = Image_msg()
    hub.run()
