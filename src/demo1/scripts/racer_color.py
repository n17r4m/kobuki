#!/usr/bin/env python

# author : Noni

# BEGIN ALL
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
import math
import time

from dynamic_reconfigure.server import Server
from demo1.cfg import lineColorConfig

class Follower:
    def __init__(self):
        self.srv = Server(lineColorConfig, self.cfg_callback)
        self.go = False
        self.stop = False
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=5)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.xtwists = []
        self.ztwists = []

        self.err_past = [0]
        self.current_error = 0

        self.lower_yellow = np.array([ 10,  10,  10])
        self.upper_yellow = np.array([255, 255, 250])

        self.r_lower_hue = 0
        self.r_lower_sat = 0
        self.r_lower_value = 247
        self.r_upper_hue = 255
        self.r_upper_sat = 255
        self.r_upper_value = 255

        self.l_lower_hue = 0
        self.l_lower_sat = 0
        self.l_lower_value = 230
        self.l_upper_hue = 123
        self.l_upper_sat = 255
        self.l_upper_value = 255

        self.canny_upper = 72
        self.canny_lower = 45

        #self.web_cap = cv2.VideoCapture(0)
        self.time_start = time.clock()

        self.debug_error = rospy.Publisher('debug/error', Float32, queue_size = 5)

        self.P_ = 1
        self.I_ = 0.1
        self.D_ = 0.1

    def cfg_callback(self, data, level):
        self.r_upper_value = data['r_upper_value']
        self.r_lower_value = data['r_lower_value']
        self.r_upper_sat = data['r_upper_sat']
        self.r_lower_sat = data['r_lower_sat']
        self.r_upper_hue = data['r_upper_hue']
        self.r_lower_hue = data['r_lower_hue']

        self.l_upper_value = data['l_upper_value']
        self.l_lower_value = data['l_lower_value']
        self.l_upper_sat = data['l_upper_sat']
        self.l_lower_sat = data['l_lower_sat']
        self.l_upper_hue = data['l_upper_hue']
        self.l_lower_hue = data['l_lower_hue']


        self.lowerbound_white = np.array([self.r_lower_hue, self.r_lower_sat, self.r_lower_value])
        self.upperbound_white = np.array([self.r_upper_hue, self.r_upper_sat, self.r_upper_value])

        self.lowerbound_yellow = np.array([self.l_lower_hue, self.l_lower_sat, self.l_lower_value])
        self.upperbound_yellow = np.array([self.l_upper_hue, self.l_upper_sat, self.l_upper_value])
        return data

    def joy_callback(self, data):
        if data.buttons[0]:
            self.go = not self.go
        #if not self.go:
            #self.stop = True
            self.xtwists = []
            self.ztwists = []

    def minpool(self, mask):
        h, w = mask.shape
        pool = np.zeros((h/2, w/2))
        for y in range(0, h, 2):
            for x in range(0, w, 2):
                pool[y/2, x/2] = np.amin(mask[y:y+1, x:x+1])
        return pool

    def gamma_correction(self, img, correction):
        img = img / 255.0
        img = cv2.pow(img, correction)
        return np.uint8(img * 255)

    def webcam(self):
        while(True):
            ret, frame = self.web_cap.read()
            if ret:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask1 = cv2.inRange(hsv, (0, 70, 50), (10, 255, 180))
                mask2 = cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))
                mask = mask1 | mask2
                mask = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=1)
                mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)
                img, cts, hir = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(cts) > 0:
                rect = list(cv2.boundingRect(cts[0]))
                #print rect
                '''
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                print box
                '''
                cv2.rectangle(frame, (rect[0], rect[1]),(rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)
                #print len(retval)
                #cv2.drawContours(frame, [box], 0, (0,0,255), 2)
                if rect[2] * rect[3] > 5:
                    self.stop = True
            else:
                self.stop = False
            cv2.imshow('webcam', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.web_cap.release()
        cv2.destroyAllWindows()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        
        # larger search area
        h, w, d = image.shape
        search_top = h / 5 * 3
        search_bot = h
        image = image[search_top:search_bot, :, :]
        h, w, d = image.shape
        
        
        image = self.gamma_correction(image, 0.5)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


        left = hsv.copy()
        right = hsv.copy()
        left[:, w/2:w] = 0
        right[:, 0:w/2] = 0


        left = cv2.inRange(left, (0,0,230), (123, 255, 255))
        right = cv2.inRange(right, self.lowerbound_white, self.upperbound_white)

        #left = self.minpool(left)
        #right = self.minpool(right)

        M_left = cv2.moments(left)
        M_right = cv2.moments(right)

        mask = left + right

        mask = cv2.bilateralFilter(mask, 5, 1, 10)

        if M_left['m00'] > 0 and M_right['m00'] > 0:
            cx_left = int(M_left['m10']/M_left['m00'])
            cy_left = int(M_left['m01']/M_left['m00'])
            cx_right = int(M_right['m10']/M_right['m00'])
            cy_right = int(M_right['m01']/M_right['m00'])
            cv2.circle(mask, (cx_left, cy_left), 20, (0,0,255), -1)
            cv2.circle(mask, (cx_right, cy_right), 20, (0,0,255),-1)
            # BEGIN CONTROL
            self.current_error = (cx_left + cx_right* 0.8)/2 - w/2

        elif M_left['m00'] > 0 and M_right['m00'] == 0:
            cx_left = int(M_left['m10']/M_left['m00'])
            cy_left = int(M_left['m01']/M_left['m00'])
            cv2.circle(mask, (cx_left, cy_left), 20, (0,0,255), -1)
            # BEGIN CONTROL
            self.current_error = - cx_left + w/2

        elif M_left['m00'] ==0 and M_right['m00'] > 0:
            cx_right = int(M_right['m10']/M_right['m00'])
            cy_right = int(M_right['m01']/M_right['m00'])
            cv2.circle(mask, (cx_right, cy_right), 20, (0,0,255),-1)
            # BEGIN CONTROL
            self.current_error = -cy_right + w/2


        if len(self.err_past) > 10:
            self.err_past.pop(0)
            #self.theta_past.pop(0)
            self.time_start = time.clock()
        else:
            self.err_past.append(self.current_error)
            #self.theta_past.append(theta)

        #err = np.mean(self.err_past)
        #print np.mean(np.diff(self.theta_past)/0.01 )
        current_time = time.clock()

        Kp = - float(self.current_error) / 150
        Ki = (max(self.err_past) - min(self.err_past)) * (current_time - self.time_start)
        Kd = np.mean(np.diff(self.err_past)/100)

        x = 0.3
        z = self.P_*Kp + self.I_*Ki + self.D_*Kd
        print z

        #if self.go and (not self.stop):
        if self.go:
            self.xtwists.append(x)
            self.ztwists.append(z)

        if len(self.xtwists) > 10:
            twist = Twist()
            twist.linear.x = np.mean(self.xtwists)
            #twist.angular.z = np.mean(self.ztwists)
            twist.angular.z = z
            self.cmd_vel_pub.publish(twist)
            self.xtwists.pop(0)
            self.ztwists.pop(0)
            # END CONTROL
        cv2.imshow("window", mask)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
#follower.webcam()
rospy.spin()
# END ALL
