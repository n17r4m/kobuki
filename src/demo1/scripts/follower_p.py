#!/usr/bin/env python
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

    self.theta_past = []
    self.err_past = [0]

    self.lower_yellow = np.array([ 10,  10,  10])
    self.upper_yellow = np.array([255, 255, 250])

    self.lower_hue = 79
    self.lower_sat = 150
    self.lower_value = 0
    self.upper_hue = 225
    self.upper_sat = 255
    self.upper_value = 101

    self.lowerbound_blue = np.array([self.lower_hue, self.lower_sat, self.lower_value])
    self.upperbound_blue = np.array([self.upper_hue, self.upper_sat, self.upper_value])

    #self.web_cap = cv2.VideoCapture(0)
    self.time_start = time.clock()

    self.debug_error = rospy.Publisher('debug/error', Float32, queue_size = 5)

    self.P_ = 2
    self.I_ = 0.1
    self.D_ = 1.5

  def cfg_callback(self, data, level):
      self.upper_value = data['upper_value']
      self.lower_value = data['lower_value']
      self.upper_sat = data['upper_sat']
      self.lower_sat = data['lower_sat']
      self.upper_hue = data['upper_hue']
      self.lower_hue = data['lower_hue']

      self.lowerbound_blue = np.array([self.lower_hue, self.lower_sat, self.lower_value])
      self.upperbound_blue = np.array([self.upper_hue, self.upper_sat, self.upper_value])
      return data


  def joy_callback(self, data):
    if data.buttons[0]:
      self.go = not self.go
      #if not self.go:
       #self.stop = True
      self.xtwists = []
      self.ztwists = []

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
    #image = cv2.flip(image, flipCode = 0)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #print self.lowerbound_blue, self.upperbound_blue
    mask = cv2.inRange(hsv, self.lowerbound_blue, self.upperbound_blue)
    #mask = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=1)
    #mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=1)

    # larger search area
    h, w, d = image.shape
    search_top = h / 6 * 5
    search_bot = h
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(mask, (cx, cy), 20, (0,0,255), -1)

      M02_ = int(M['m02']) - pow(int(M['m01']),2) / int(M['m00'])
      M20_ = int(M['m20']) - pow(int(M['m10']),2) / int(M['m00'])
      M11_ = int(M['m11']) - int(M['m10']) * int(M['m01']) / int(M['m00'])

      theta = 45
      if not M20_ - M02_ == 0:
        tan2th = 2 * M11_ / (M20_ - M02_)
        theta = math.degrees(math.atan(tan2th)/2)
        rospy.loginfo("degree is "+ str(theta) )
      # BEGIN CONTROL
      err = cx - w/2
      e = Float32()
      e.data = err
      self.debug_error.publish(err)

      if len(self.err_past) > 10:
        self.err_past.pop(0)
        self.theta_past.pop(0)
        self.time_start = time.clock()
      else:
        self.err_past.append(err)
        self.theta_past.append(theta)

      #err = np.mean(self.err_past)
      #print np.mean(np.diff(self.theta_past)/0.01 )
      current_time = time.clock()

      Kp = - (float(err) / 150 + (theta - 45) /250 )
      Ki = (max(self.err_past) - min(self.err_past)) * (current_time - self.time_start)
      Kd = np.mean(np.diff(self.err_past)/100)

      x = 0.6 - abs(float(err)/500)
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
