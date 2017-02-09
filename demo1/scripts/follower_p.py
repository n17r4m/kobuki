#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math

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
    self.err_past = []

    self.lower_yellow = np.array([ 10,  10,  10])
    self.upper_yellow = np.array([255, 255, 250])

    self.lower_blue = 79
    self.lower_green = 150
    self.lower_red = 0
    self.upper_blue = 225
    self.upper_green = 255
    self.upper_red = 101

    self.lowerbound_blue = np.array([self.lower_blue, self.lower_green, self.lower_red])
    self.upperbound_blue = np.array([self.upper_blue, self.upper_green, self.upper_red])

    self.web_cap = cv2.VideoCapture(0)

  def cfg_callback(self, data, level):
      self.upper_red = data['upper_red']
      self.lower_red = data['lower_red']
      self.upper_green = data['upper_green']
      self.lower_green = data['lower_green']
      self.upper_blue = data['upper_blue']
      self.lower_blue = data['lower_blue']

      self.lowerbound_blue = np.array([self.lower_blue, self.lower_green, self.lower_red])
      self.upperbound_blue = np.array([self.upper_blue, self.upper_green, self.upper_red])
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
    search_top = h / 4 * 3
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
      if len(self.err_past) > 5:
        self.err_past.pop(0)
      else:
        self.err_past.append(err)

      if len(self.theta_past) > 5:
        self.theta_past.pop(0)
      else:
        self.theta_past.append(theta)
      #err = np.mean(self.err_past)
      print np.mean(np.diff(self.theta_past)/0.01 )

      x = 0.2
      z = - ( (float(err) / 150 + (theta - 45) /250 ))

      if self.go and (not self.stop):
        self.xtwists.append(x)
        self.ztwists.append(z)

        if len(self.xtwists) > 8:
          twist = Twist()
          twist.linear.x = np.mean(self.xtwists)
          twist.angular.z = np.mean(self.ztwists)
          self.cmd_vel_pub.publish(twist)
          self.xtwists.pop(0)
          self.ztwists.pop(0)
      # END CONTROL
    cv2.imshow("window", mask)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
follower.webcam()
rospy.spin()
# END ALL
