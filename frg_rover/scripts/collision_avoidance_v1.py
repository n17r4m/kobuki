#!/usr/bin/env python
PKG = 'frg_rover'
import roslib; roslib.load_manifest(PKG)
#Need this for the Msgs to work
roslib.load_manifest('frg_rover_msgs')

import rospy
import numpy
import math

from std_msgs.msg import UInt16
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from frg_rover_msgs.msg import sys_states_all
    
state0 = 0
state1 = 0
state2 = 0
state3 = 0
state4 = 0
state5 = 0
state6 = 0
state7 = 0
state8 = 0
state9 = 0

scan_data = LaserScan()


# ----------------------------------------------
def update_states(data):
    global state0
    global state1
    global state2
    global state3
    global state4
    global state5
    global state6
    global state7
    global state8
    global state9

    state0 = data.state0
    state1 = data.state1
    state2 = data.state2
    state3 = data.state3
    state4 = data.state4
    state5 = data.state5
    state6 = data.state6
    state7 = data.state7
    state8 = data.state8
    state9 = data.state9

def update_scan(data):
    global scan_data
    scan_data = data
#-----------------------------------------

def publish_state():

    global state0
    global state1
    global state2
    global state3
    global state4
    global state5
    global state6
    global state7
    global state8
    global state9

    global scan_data

    rospy.Subscriber('/sys_states/sys_states_all',sys_states_all,update_states)
    rospy.Subscriber('/scan_base2',LaserScan,update_scan)

    pub = rospy.Publisher('/sys_states/state2',UInt16)

    pub_vel2 = rospy.Publisher('/frg_rover_control2',Twist)


    rospy.init_node('frg_state_pub', anonymous=True)


    r = rospy.Rate(10)
    state = UInt16()
    e_stop = 0

    cmd = Twist()
    while not rospy.is_shutdown():

        #print scan_data.ranges[0]
        a_min = scan_data.angle_min
        a_inc = scan_data.angle_increment
        a_list = [a_min]
        a_max = a_min
        for i in range(len(scan_data.ranges)-1):
            a_max += a_inc
            a_list.append(a_max)

        # for i in range(len(a_list)):
        #     a_list[i] = a_list[i] - math.pi/2

        #print len(scan_data.ranges), len(a_list)

        d1 = 100000
        a1 = 100000
        dx = 100000
        dy = 100000

        dx_min = 1000000

        if len(scan_data.ranges) > 1:

            point_valid = 0
            for i in range(len(scan_data.ranges)):
                d1 = scan_data.ranges[i]
                a1 = a_list[i]

                dx =  d1 * numpy.cos(a1)
                dy =  d1 * numpy.sin(a1)

                if dx_min > dx:
                    dx_min = dx

        #print dx_min

        if e_stop and dx_min > 0.55:
            e_stop = 0

        if dx_min < 0.5:
            e_stop = 1

        if e_stop:
            state.data = 1
           
        else:
            state.data = 2


        if not(state2 == state):
            pub.publish(state) 

        pub_vel2.publish(cmd)

        r.sleep()

if __name__ == "__main__": 
    publish_state()












