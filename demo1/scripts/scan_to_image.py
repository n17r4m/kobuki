#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

global come_in_img
global come_in_scan

def distance_mapping():
    """
    maps distance to color difference
    The further the object is, more purple it is. Otherwise, the nearer the reder.
    """

def scan_callback(msg):
    global come_in_scan
    come_in_scan = msg.ranges
    print len(msg.rang)
    #depth_img_pub.publish(bridge.cv2_to_imgmsg(depth_img, "bgr8"))

def image_callback(data):
    global come_in_img
    come_in_img = bridge.imgmsg_to_cv2(data, "bgr8")

if __name__ == "__main__":
    global come_in_img
    global come_in_scan
    rospy.init_node('scanToimage')
    bridge = CvBridge()
    depth_img_pub = rospy.Publisher('/depth_with_line', Image, queue_size = 5)
    depth_img_sub = rospy.Subscriber('/camera/depth/image_raw', Image, image_callback)
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    rate = rospy.Rate(10)

    cv2.line(come_in_img, (0,320), (0, 640), (255,0,0))
    depth_img_pub.publish(bridge.cv2_to_imgmsg(come_in_img, "bgr8"))

    try:
        while not rospy.is_shutdown():
            #rospy.spin()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
