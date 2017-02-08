#!/usr/bin/env python
PKG = 'frg_rover'
import roslib; roslib.load_manifest(PKG)
#Need this for the Msgs to work
roslib.load_manifest('frg_rover_msgs')

import rospy
import numpy

from std_msgs.msg import UInt16
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

    rospy.Subscriber('/sys_states/sys_states_all',sys_states_all,update_states)

    pub = rospy.Publisher('/sys_states/state0',UInt16)
    rospy.init_node('frg_state_pub', anonymous=True)


    r = rospy.Rate(1)
    state = UInt16()

    while not rospy.is_shutdown():

        if not(state0 == state):
            pub.publish(state) 

        r.sleep()
        print state

if __name__ == "__main__": 
    publish_state()

