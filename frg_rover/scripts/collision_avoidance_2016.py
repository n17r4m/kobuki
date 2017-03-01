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

    rospy.Subscriber('/scan_laser',LaserScan,update_scan, queue_size = 10)

    pub_vel = rospy.Publisher('/move_base/cmd_vel',Twist, queue_size = 10)

    rospy.init_node('frg_collision_avoidance', anonymous=True)


    r = rospy.Rate(10)
    e_stop = 0

    cmd = Twist()

    dmin_MA = Moving_Average_v0(5)
    dmin_average = 1000

    while not rospy.is_shutdown():

        #print scan_data.ranges[0]
        a_min = scan_data.angle_min
        a_inc = scan_data.angle_increment

        a_list = [a_min]
        a_max = a_min

        for i in range(len(scan_data.ranges)-1):
            a_max += a_inc
            a_list.append(a_max)

        # print a_list

        # for i in range(len(a_list)):
        #     a_list[i] = a_list[i] - math.pi/2

        if not len(scan_data.ranges) > 0:
            print 'No scan data...'

        dx_min = 1000000

        if len(scan_data.ranges) > 1:

            dx_list = []
            dy_list = []

            for i in range(len(scan_data.ranges)):
                d1 = scan_data.ranges[i]
                a1 = a_list[i]

                dx =  d1 * numpy.cos(a1)
                dy =  d1 * numpy.sin(a1)

                dx_list.append(dx)
                dy_list.append(dy)

                if dx_min > dx:
                    dx_min = dx

        

        dmin_average = dmin_MA.update_average(dx_min)
        print dx_min, dmin_average

        dist_limit = 1.0
        if (e_stop==1) and dmin_average > dist_limit+0.1:
            # get out of estop
            e_stop = 0

        if dmin_average < dist_limit:
            # engage estop
            e_stop = 1

        if e_stop:
            pub_vel.publish(cmd)
            print rospy.Time.now(), "e-stop engaged"

        r.sleep()

if __name__ == "__main__": 
    main_collision_avoidance()












