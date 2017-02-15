#!/usr/bin/env python
# BEGIN ALL
import rospy, roslib, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server

class Racer:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
    self.twist = Twist()
    
  def cfg_callback(self, data, level):
      self.upper_value = data['upper_value']
      self.lower_value = data['lower_value']
      self.upper_sat = data['upper_sat']
      self.lower_sat = data['lower_sat']
      self.upper_hue = data['upper_hue']
      self.lower_hue = data['lower_hue']
  
  
  def minpool(self, mask):
    h, w = mask.shape
    pool = numpy.zeros((h/2, w/2))
    for y in range(0, h, 2):
      for x in range(0, w, 2):
        pool[y/2, x/2] = numpy.amin(mask[y:y+1, x:x+1])
    return pool
  
  
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    h, w, d = image.shape
    
    top = 3*h/5
    bot = 3*h/5 + 80
    roi = image[top:bot, :, :]
    
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    solid = self.mask_yellow(hsv)
    pool = self.minpool(solid)
    
    cv2.imshow("hsv", pool)
    cv2.waitKey(3)
  
  def mask_yellow(self, hsv):
    lower = numpy.array([ 10,  10,  10])
    upper = numpy.array([255, 255, 250])
    return cv2.inRange(hsv, lower, upper)
  
  def mask_white(self, hsv):
    lower = numpy.array([ 10,  10,  10])
    upper = numpy.array([ 100, 100, 100])
    return cv2.inRange(hsv, lower, upper)



rospy.init_node('racer')
racer = Racer()
rospy.spin()
# END ALL
