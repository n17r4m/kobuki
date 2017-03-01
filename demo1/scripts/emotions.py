#!/usr/bin/env python

import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
import cv2

cv2.namedWindow("emotions")
zero = cv2.imread('/home/ka/catkin_ws/src/demo1/scripts/imgs/zero.jpeg')
thinking = cv2.imread('/home/ka/catkin_ws/src/demo1/scripts/imgs/thinking.png')
happy = cv2.imread('/home/ka/catkin_ws/src/demo1/scripts/imgs/happy.jpg')
happy = cv2.resize(happy,(256, 256), interpolation = cv2.INTER_CUBIC)
emb = cv2.imread('/home/ka/catkin_ws/src/demo1/scripts/imgs/emb.jpg')
emb = cv2.resize(emb,(256, 256), interpolation = cv2.INTER_CUBIC)
flushed = cv2.imread('/home/ka/catkin_ws/src/demo1/scripts/imgs/flushed.png')

image_pub = rospy.Publisher('/emotions', Image, queue_size = 10)
bridge = CvBridge()

def callback(data):
    print data.velocity
    if -0.1 <= data.velocity[0] < 0.5 and -0.1 <= data.velocity[1] < 0.5:
        try:
            image_pub.publish(bridge.cv2_to_imgmsg(zero, "bgr8"))
        except CvBridgeError as e:
            print(e)
    elif data.velocity[0] > 2 and data.velocity[1] > 2:
        try:
            image_pub.publish(bridge.cv2_to_imgmsg(happy, "bgr8"))
        except CvBridgeError as e:
            print(e)
    elif data.velocity[0] < -2 and data.velocity[1] < -2:
        try:
            image_pub.publish(bridge.cv2_to_imgmsg(flushed, "bgr8"))
        except CvBridgeError as e:
            print(e)
    elif abs(data.velocity[0] - data.velocity[1]) > 2:
        try:
            image_pub.publish(bridge.cv2_to_imgmsg(thinking, "bgr8"))
        except CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    rospy.init_node('emotions')
    velocity = rospy.Subscriber('/joint_states', JointState, callback)
    rate = rospy.Rate(10) # 30hz
    try:
        while not rospy.is_shutdown():
            #rospy.spin()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
