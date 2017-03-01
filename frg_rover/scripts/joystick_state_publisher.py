#!/usr/bin/env python
PKG = 'frg_rover'
import roslib; roslib.load_manifest(PKG)
#Need this for the Msgs to work
roslib.load_manifest('frg_rover_msgs')

import rospy
import numpy

from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy
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

joy_data = Joy()


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

def update_joy(data):
    global joy_data
    joy_data = data
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

    global joy_data

    rospy.Subscriber('/sys_states/sys_states_all',sys_states_all,update_states)
    rospy.Subscriber('/joy',Joy,update_joy)

    pub = rospy.Publisher('/sys_states/state1',UInt16)
    pub_state3 = rospy.Publisher('/sys_states/state3',UInt16)

    rospy.init_node('frg_state_pub', anonymous=True)


    r = rospy.Rate(50)
    state1a = UInt16()
    state3a = UInt16()

    button_B = 0
    button_Y = 0


    while not rospy.is_shutdown():


        if not(len(joy_data.buttons)>3): pass
        else:
            button_X = joy_data.buttons[0]
            button_A = joy_data.buttons[1]
            button_B = joy_data.buttons[2]
            button_Y = joy_data.buttons[3]
            button_LB = joy_data.buttons[4]

            axes_0 = joy_data.axes[0]
            axes_1 = joy_data.axes[1]
            axes_2 = joy_data.axes[2]
            axes_3 = joy_data.axes[3]

            axes_0 = int(axes_0*100)
            axes_1 = int(axes_1*100)
            axes_2 = int(axes_2*100)
            axes_3 = int(axes_3*100)



            if button_X or button_A:
                state1a = 1
            if axes_0 or axes_1 or axes_2 or axes_3:
                state1a = 1
            if button_B:
                state1a = 2
                state3a = 1
            if button_LB:
                state1a = 2
                state3a = 2
            if button_Y:
                state1a = 2
                state3a = 3

        if not(state1 == state1a):
            pub.publish(state1a)
        if not(state3 == state3a):
            pub_state3.publish(state3a)
        r.sleep()

if __name__ == "__main__": 
    publish_state()

