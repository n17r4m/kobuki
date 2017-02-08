#!/usr/bin/env python
PKG = 'frg_rover'
import roslib; roslib.load_manifest(PKG)
#Need this for the Msgs to work
roslib.load_manifest('frg_rover_msgs')

import rospy
import numpy

from std_msgs.msg import UInt16
from frg_rover_msgs.msg import sys_states_all

state0 = UInt16()
state1 = UInt16()
state2 = UInt16()
state3 = UInt16()
state4 = UInt16()
state5 = UInt16()
state6 = UInt16()
state7 = UInt16()
state8 = UInt16()
state9 = UInt16()

#state1.data = 1
#state3.data = 1

# ----------------------------------------------

def update_state0(data):
    global state0
    state0 = data

def update_state1(data):
    global state1
    state1 = data

def update_state2(data):
    global state2
    state2 = data

def update_state3(data):
    global state3
    state3 = data

def update_state4(data):
    global state4
    state4 = data

def update_state5(data):
    global state5
    state5 = data

def update_state6(data):
    global state6
    state6 = data

def update_state7(data):
    global state7
    state7 = data

def update_state8(data):
    global state8
    state8 = data

def update_state9(data):
    global state9
    state9 = data

#-----------------------------------------

def publish_states():

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

    rospy.Subscriber('/sys_states/state0',UInt16, update_state0)
    rospy.Subscriber('/sys_states/state1',UInt16, update_state1)
    rospy.Subscriber('/sys_states/state2',UInt16, update_state2)
    rospy.Subscriber('/sys_states/state3',UInt16, update_state3)
    rospy.Subscriber('/sys_states/state4',UInt16, update_state4)
    rospy.Subscriber('/sys_states/state5',UInt16, update_state5)
    rospy.Subscriber('/sys_states/state6',UInt16, update_state6)
    rospy.Subscriber('/sys_states/state7',UInt16, update_state7)
    rospy.Subscriber('/sys_states/state8',UInt16, update_state8)
    rospy.Subscriber('/sys_states/state9',UInt16, update_state9)

    pub = rospy.Publisher('/sys_states/sys_states_all',sys_states_all)

    rospy.init_node('frg_sys_states_pub', anonymous=True)

    r = rospy.Rate(50)
    states = sys_states_all()

    while not rospy.is_shutdown():

        
        
        states.state0 = state0.data
        states.state1 = state1.data
        states.state2 = state2.data
        states.state3 = state3.data
        states.state4 = state4.data
        states.state5 = state5.data
        states.state6 = state6.data
        states.state7 = state7.data
        states.state8 = state8.data
        states.state9 = state9.data

        pub.publish(states)
        r.sleep()
        #print state0, state1, state2, state3, state4, state5, state6, state7, state8, state9

if __name__ == "__main__": 
    publish_states()

