#!/usr/bin/env python
PKG = 'frg_rover'
import roslib; roslib.load_manifest(PKG)
#Need this for the Msgs to work
roslib.load_manifest('frg_rover_msgs')

import rospy
import numpy

from geometry_msgs.msg import Twist
from frg_rover_msgs.msg import sys_states_all

vel1 = Twist()
vel2 = Twist()
vel3 = Twist()
vel4 = Twist()
vel5 = Twist()

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

def update_vel1(data):
    global vel1
    vel1 = data

def update_vel2(data):
    global vel2
    vel2 = data

def update_vel3(data):
    global vel3
    vel3 = data

def update_vel4(data):
    global vel4
    vel4 = data

def update_vel5(data):
    global vel5
    vel5 = data

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

# ----------------------------------------------


def sendmsg():
    global vel1
    global vel2
    global vel3
    global vel4
    global vel5

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

    rospy.Subscriber('/frg_rover_control1',Twist, update_vel1)
    rospy.Subscriber('/frg_rover_control2',Twist, update_vel2)
    rospy.Subscriber('/frg_rover_control3',Twist, update_vel3)
    rospy.Subscriber('/frg_rover_control4',Twist, update_vel4)
    rospy.Subscriber('/frg_rover_control5',Twist, update_vel5)

    pub = rospy.Publisher('/husky/cmd_vel',Twist)

    #pub_teleop_joystick = rospy.Publisher('/sys_states/teleop_joystick',sys_state)
    #pub_teleop_rc       = rospy.Publisher('/sys_states/teleop_rc',sys_state)


    rospy.init_node('frg_rover_motion_control', anonymous=True)

    r = rospy.Rate(50) # 50hz because this is the max freq that HA200 can receive

    #temp_state = sys_state()


    i = 0
    imax = 50

    while not rospy.is_shutdown():

        #-----------------------------------------------
        # MOTION CONTROL
        # always publish 0, unless somebody is in control
        cmd = Twist()

        # In this order, the bottom one will have priority

        # Autonomous in control
        if state3 == 1:
            cmd = vel3
        elif state3 == 2:
            cmd = vel4
        elif state3 == 3:
            cmd = vel5

        # Obstacle avoidance in control
        if state2 == 1:
            cmd = vel2

        # Highest Priority: Joystick in control
        if state1 == 1 or state1 == 0:
            cmd = vel1

        pub.publish(cmd)

        
        #print state0, state1, state2, state3, state4, state5, state6, state7, state8, state9
        r.sleep()


if __name__ == "__main__": 
    sendmsg()

