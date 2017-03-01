#!/usr/bin/env python
PKG = 'frg_rover'
import roslib; roslib.load_manifest(PKG)
#Need this for the Msgs to work
roslib.load_manifest('frg_rover_msgs')

import rospy
import numpy

from std_msgs.msg import UInt16
from frg_rover_msgs.msg import sys_states_all


# This is needed to get keys pressed. 
import termios, fcntl, sys, os
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)


# ----------------------------------------------


state_array= []
for i in range(10):
    state_array.append(UInt16())

state_array_r = []
for i in range(10):
    state_array_r.append(0)

pub_list = []

state_mod_flag = 1
state_mod = 0
print '\nSelect state number to modify: (0-9):' 

def useC(c):
    global state_array
    global pub_list

    global state_mod_flag
    global state_mod

    if c == 's':
        rospy.loginfo('Select state number to modify: (0-9).')
        state_mod_flag = 1

    if c == '0' or c == '1' or c == '2' or c == '3' or c == '4' or c == '5' or c == '6' or c == '7' or c == '8' or c == '9':
        if state_mod_flag:
            state_mod = int(c)
            state_mod_flag = 0
            rospy.loginfo('Selected state ' + str(state_mod)+ '.')
            rospy.loginfo('Select state value to assign to state '+ str(state_mod)+' (0-9):')
        else:            
            state_array[state_mod].data = int(c)
            pub_list[state_mod].publish(state_array[state_mod])
            rospy.loginfo('Selected state value ' + c +' for state '+ str(state_mod) + '.')
            rospy.loginfo('Select state value to assign to state '+ str(state_mod)+' (0-9) or press s to select a different state:' )



# ----------------------------------------------
def update_states(data):
    global state_array_r

    state_array_r[0] = data.state0
    state_array_r[1] = data.state1
    state_array_r[2] = data.state2
    state_array_r[3] = data.state3
    state_array_r[4] = data.state4
    state_array_r[5] = data.state5
    state_array_r[6] = data.state6
    state_array_r[7] = data.state7
    state_array_r[8] = data.state8
    state_array_r[9] = data.state9

#-----------------------------------------

def publish_state():

    global state_array_r
    global pub_list

    rospy.Subscriber('/sys_states/sys_states_all',sys_states_all,update_states)

    pub0 = rospy.Publisher('/sys_states/state0',UInt16)
    pub1 = rospy.Publisher('/sys_states/state1',UInt16)
    pub2 = rospy.Publisher('/sys_states/state2',UInt16)
    pub3 = rospy.Publisher('/sys_states/state3',UInt16)
    pub4 = rospy.Publisher('/sys_states/state4',UInt16)
    pub5 = rospy.Publisher('/sys_states/state5',UInt16)
    pub6 = rospy.Publisher('/sys_states/state6',UInt16)
    pub7 = rospy.Publisher('/sys_states/state7',UInt16)
    pub8 = rospy.Publisher('/sys_states/state8',UInt16)
    pub9 = rospy.Publisher('/sys_states/state9',UInt16)

    pub_list = [pub0, pub1, pub2, pub3, pub4, pub5, pub6, pub7, pub8, pub9]

    rospy.init_node('frg_state_pub', anonymous=True)


    r = rospy.Rate(1)
    
    while not rospy.is_shutdown():

        ii = 0
        while ii<10:
            try:
                c = sys.stdin.read(1)
                useC(c)
            except IOError: pass
            ii = ii+1

        r.sleep()

    #this is needed to get the terminal back:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)    
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

if __name__ == "__main__": 
    publish_state()

