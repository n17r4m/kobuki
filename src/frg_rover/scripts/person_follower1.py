#!/usr/bin/env python
PKG = 'rover1'
import roslib
roslib.load_manifest(PKG)

import rospy
import math
import tf
import geometry_msgs.msg
import numpy

from std_msgs.msg import Int16
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist



time_last_heart_beat = 1.0
time_prev_heart_beat = 0.0

#---------------------------------------

def update_time_since_last_heart_beat(data):
    global time_last_heart_beat
    global time_prev_heart_beat
    time_prev_heart_beat = time_last_heart_beat
    time_last_heart_beat = rospy.get_time()

#---------------------------------------

def ckeck_comm_wifi():

    global time_last_heart_beat
    global time_prev_heart_beat


    pub                 = rospy.Publisher('/frg_rover_control3',Twist)


    rospy.Subscriber('/tf',TFMessage, update_time_since_last_heart_beat)
    rospy.init_node('kinect_control', anonymous=True)

    r = rospy.Rate(20) # hz
    cmd = Twist()

    listener = tf.TransformListener()

    global xk
    global yk
    global zk
    global vr
    global wr

    global KP

    global yk_goal

    xk, yk, zk, vr, wr = 0, 0, 0, 0, 0

    tf_good = 0
    e = 0
    e_old = 0

    while not rospy.is_shutdown():


        c_go = 1

        if time_last_heart_beat > time_prev_heart_beat:
            # /tf publishing
            tf_good = 1
        else:
            # /tf lost
            tf_good = 0
        time_prev_heart_beat = time_last_heart_beat

        try:
            (trans,rot) = listener.lookupTransform('openni_depth_frame','torso_1',  rospy.Time(0))
            (xk, yk, zk) = trans
            #print yk#, yk, zk

            # CHECK FOR LIMITS
            
            if xk < 0.8:
                c_go = 0
            if xk > 2.5:
                c_go = 0
            if yk < -0.5:
                c_go = 0
            if yk > 0.5:
                c_go = 0



            # Linear Velocity:
            KP = 2.0 / xk  # so its a smaller KP when it is further away
            KD = 0.00 # not very good because de is very noisy - so this makes the control signal noisy
            yk_goal = 0.0

            e = float(yk_goal - yk)
            de = (e - e_old)/float(0.05)
            e_old = e

            if abs(e) > 0.005:
                vr = KP * e + KD * de
                #print de, KP * e ,  KD * de
            else:
                vr = 0

            if vr > 1.0:
                vr = 1.0
            if vr < -1.0:
                vr = -1.0


            # Angular Velocity

            wr = 0.0
            yk_goal2 = 0.0

            xk_mid = 1.2
            yk_mid = 0

            wr_dir = 0.0

            # Positive dir
            if xk >  1.2 and yk < yk_mid:
                wr_dir = 1.0
            if xk <  1.2 and yk > yk_mid:
                wr_dir = 1.0     

            # Negative dir
            if xk >  1.2 and yk > yk_mid:
                wr_dir = - 1.0
            if xk <  1.2 and yk < yk_mid:
                wr_dir = - 1.0  


            if xk > 1.22 or xk < 1.18:

                KPw = 0.5
                xk_goal = 1.2
                ew = float(xk_goal - xk)
                wr = KPw * ew
                wr = wr_dir * abs(wr)
            else:
                wr = 0

            if  abs(e) < 0.01:
                wr = 0

            #wr = 0
            #print yk_goal2

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

            c_go = 0
            print 'except'
            #continue



        # SEND STUFF

        if not c_go:
            vr = 0.0
            wr = 0.0
            print 'c_go = 0'

        # if not tf_good:
        #     # works but is not imediate
        #     vr = 0.0
        #     wr = 0.0
        #     print 'tf_good = 0'

        print vr, wr, xk
        cmd.linear.x = vr
        cmd.angular.z = wr 
        pub.publish(cmd)
        r.sleep()

if __name__ == "__main__": 
    ckeck_comm_wifi()

'''
Notes:
Kinect mounted on the left side of the robot, pointing out



'''