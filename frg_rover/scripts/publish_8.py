#!/usr/bin/env python
PKG = 'frg_rover'
import roslib; roslib.load_manifest(PKG)
#Need this for the Msgs to work
roslib.load_manifest('frg_rover_msgs')

import rospy
import numpy
import math
from numpy import *

from std_msgs.msg import UInt16
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
    
class Moving_Average_v0:
    def __init__(self,buf_len = 10):
        self.buf = []
        self.buf_len = buf_len
        self.average = 0

    def update_average(self, val):
        if len(self.buf) >= self.buf_len:
            for i in range(self.buf_len - 1):
                self.buf[i] = self.buf[i+1]
            self.buf[self.buf_len - 1] = val
        else:
            self.buf.append(val)

        self.average = mean(self.buf)

        return self.average

scan_data = LaserScan()
new_scan_flag = 0

# ----------------------------------------------

def update_scan(data):
    global scan_data
    global new_scan_flag

    new_scan_flag = 1
    scan_data = data
#-----------------------------------------

def main_collision_avoidance():

    global scan_data
    global new_scan_flag

    pub_vel = rospy.Publisher('/cmd_vel',Twist, queue_size = 10)

    rospy.init_node('frg_move_8', anonymous=True)


    r = rospy.Rate(10)
    e_stop = 0

    cmd = Twist()

    dmin_MA = Moving_Average_v0(5)
    dmin_average = 1000


    i = 0

    while not rospy.is_shutdown():
        vr = 0.9

        wr = 0.60 * sin(2.0*math.pi* float(i) / float(150))

        wr = 0.1
        if i > 500:
            wr = -0.1

        if i > 1000:
            i = 0
        cmd.linear.x = vr 
        cmd.angular.z = wr

        pub_vel.publish(cmd)
#        print rospy.Time.now(), "e-stop engaged"

        r.sleep()
        i = i+1

if __name__ == "__main__": 
    main_collision_avoidance()












