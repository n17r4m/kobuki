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
        cv2.namedWindow("window", 1)
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

        self.xlist = []
        self.ylist = []

        self.r_lower_hue = 0
        self.r_lower_sat = 0
        self.r_lower_value = 217
        self.r_upper_hue = 255
        self.r_upper_sat = 255
        self.r_upper_value = 255

        self.l_lower_hue = 0
        self.l_lower_sat = 0
        self.l_lower_value = 218
        self.l_upper_hue = 255
        self.l_upper_sat = 255
        self.l_upper_value = 255

        self.canny_upper = 72
        self.canny_lower = 45

        self.web_cap = cv2.VideoCapture(0)
        self.time_start = time.clock()

        self.debug_error = rospy.Publisher('debug/error', Float32, queue_size = 5)

        self.P_ = 1
        self.I_ = 0.1
        self.D_ = 0.2

        self.turn = 0
        # 0->go forward 1->turn left 2->turn right

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
        if data.buttons[1]:
            self.go = not self.go
        if not self.go:
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
                #mask1 = cv2.inRange(hsv, (0, 70, 50), (10, 255, 180))
                mask = cv2.inRange(hsv, (170, 70, 50), (200, 255, 255))
                #mask = mask1 | mask2
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

                #print len(retval)
                #cv2.drawContours(frame, [box], 0, (0,0,255), 2)
                if rect[2] * rect[3] > 50:
                    self.stop = True
                    cv2.rectangle(frame, (rect[0], rect[1]),(rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)
            else:
                self.stop = False
            cv2.imshow('webcam', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.web_cap.release()
        cv2.destroyAllWindows()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #while True:
            #ret, image = self.web_cap.read()                cv2.rectangle(frame, (rect[0], rect[1]),(rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 2)
        image = self.gamma_correction(image, 0.5)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # larger search area
        h, w, d = image.shape
        search_top = h / 5 * 3
        search_bot = h
        hsv[0:search_top, 0:w] = 0
        hsv[search_bot:h, 0:w] = 0

        left = hsv.copy()
        right = hsv.copy()
        left[:, w/5 * 2:w] = 0
        right[:, 0:w/5 *3] = 0


        left = cv2.inRange(left, self.lowerbound_yellow, self.upperbound_yellow)
        right = cv2.inRange(right, self.lowerbound_white, self.upperbound_white)

        #left = self.minpool(left)
        #right = self.minpool(right)

        M_left = cv2.moments(left)
        M_right = cv2.moments(right)

        mask = left + right
        cy_left = 0
        cy_right = 0

        mask = cv2.bilateralFilter(mask, 5, 1, 10)

        if M_left['m00'] > 0 and M_right['m00'] > 0:
            cx_left = int(M_left['m10']/M_left['m00'])
            cy_left = int(M_left['m01']/M_left['m00'])
            cx_right = int(M_right['m10']/M_right['m00'])
            cy_right = int(M_right['m01']/M_right['m00'])
            cv2.circle(mask, (cx_left, cy_left), 20, (0,0,255), -1)
            cv2.circle(mask, (cx_right, cy_right), 20, (0,0,255),-1)
            # BEGIN CONTROL
            self.current_error = (cx_left + cx_right)/2 - w/2
            #self.turn = 0

        elif M_left['m00'] > 0 and M_right['m00'] == 0:
            cx_left = int(M_left['m10']/M_left['m00'])
            cy_left = int(M_left['m01']/M_left['m00'])
            cv2.circle(mask, (cx_left, cy_left), 20, (0,0,255), -1)
            # BEGIN CONTROL
            self.current_error = -cx_left + w/2
            #self.turn = 2

        elif M_left['m00'] ==0 and M_right['m00'] > 0:
            cx_right = int(M_right['m10']/M_right['m00'])
            cy_right = int(M_right['m01']/M_right['m00'])
            cv2.circle(mask, (cx_right, cy_right), 20, (0,0,255),-1)
            # BEGIN CONTROL
            self.current_error = cy_right - w/2
            #self.turn = 1

        else:
            #self.current_error = self.err_past[-1]
            z = self.ztwists[-1]

        print self.current_error


        # error correction
        tmp = self.err_past
        tmp.append(self.current_error)
        if np.std(tmp) > np.std(self.err_past) and len(self.err_past) > 5:

            if len(self.err_past) > 10:
                self.err_past.pop(0)
                #self.theta_past.pop(0)
                self.time_start = time.clock()
            else:
                self.err_past.append(self.current_error)
                #self.theta_past.append(theta)
        else:
            self.current_error = np.mean(self.err_past)

        left_angle = 0
        right_angle = 0
        points_angle = 0
        if M_left['m00'] != 0 and M_right['m00'] != 0:
            left_20 = M_left['m20'] / M_left['m00'] - cx_left*cx_left
            left_02 = M_left['m02'] / M_left['m00'] - cy_left*cy_left
            left_11 = M_left['m11'] / M_left['m00'] - cx_left*cy_left
            left_angle = 0.5 * math.atan2(2*left_11, (left_20-left_02))
            right_20 = M_right['m20'] / M_right['m00'] - cx_right*cx_right
            right_02 = M_right['m02'] / M_right['m00'] - cy_right*cy_right
            right_11 = M_right['m11'] / M_right['m00'] - cx_right*cy_right
            right_angle = 0.5 * math.atan2(2*right_11, (right_20-right_02))
            points_angle = math.atan2((cx_left-cx_right),(cy_left-cy_left))

        #err = np.mean(self.err_past)
        #print np.mean(np.diff(self.theta_past)/0.01 )
        current_time = time.clock()

        Kp = float(self.current_error) / 150
        Ki = (max(self.err_past) - min(self.err_past)) * (current_time - self.time_start)
        Kd = np.mean(np.diff(self.err_past)/100)

        x = 0.2
        z = self.P_*Kp # + self.I_*Ki + self.D_*Kd
        #z = 0

        if cy_left >= 420 and cy_right >= 420:
            z = 0.8
        elif cy_left >= 420 and cy_right < 420:
            self.turn = 1
            z = 0.8
        elif cy_left < 420 and cy_right >= 420 and right_angle <= 1.57:
            z = -0.8
            self.turn = 2
        if (left_angle == points_angle) | (right_angle == points_angle):
            z = 0.8

        if self.go and (not self.stop):
        #if self.go:
            self.xtwists.append(x)
            self.ztwists.append(z)

        if len(self.xtwists) > 5:
            twist = Twist()
            twist.linear.x = np.mean(self.xtwists)
            twist.angular.z = np.mean(self.ztwists)
            #twist.angular.z = z
            rospy.loginfo("====z value" + str(z))
            self.cmd_vel_pub.publish(twist)
            self.xtwists.pop(0)
            self.ztwists.pop(0)
            # END CONTROL
        cv2.imshow("window", mask)
        cv2.waitKey(3)

        #self.web_cap.release()
        #cv2.destroyAllWindows()

rospy.init_node('follower')
follower = Follower()
#follower.webcam()
rospy.spin()
# END ALL
