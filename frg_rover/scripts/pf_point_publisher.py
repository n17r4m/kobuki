#!/usr/bin/env python
PKG = 'frg_rover'
import roslib; roslib.load_manifest(PKG)
#Need this for the Msgs to work
roslib.load_manifest('frg_rover_msgs')

import rospy
import numpy
from random import random, gauss
from copy import deepcopy
from math import *

from std_msgs.msg import UInt16
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from frg_rover_msgs.msg import sys_states_all
from geometry_msgs.msg import Point
    
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


class particle:
    # Particle Class
    # based on the code from the robot() class in Unit 3.

    def __init__(self):
        self.x = 1.0 #random()
        self.y = 0.0 #random()
        self.xdot = 0.0#random() * initial_vel_max
        self.ydot = 0.0#random() * initial_vel_max

        self.a = 0.0
        self.move_stdev = [0.0,0.0,0.0,0.0]
        self.sense_stdev = [0.0]

    def set_pos(self, new_x, new_y, new_xdot, new_ydot):

        self.x = new_x
        self.y = new_y
        self.xdot = new_xdot
        self.ydot = new_ydot

    def set_stdev(self, new_move_stdev, new_sense_stdev):
        self.move_stdev = new_move_stdev
        self.sense_stdev = new_sense_stdev

    def sense(self, x_list, y_list):

        err = 100000
        i_a_close = 0

        for i in range(len(x_list)):

            xerr = self.x - x_list[i]
            yerr = self.y - y_list[i]
            err_tmp = numpy.sqrt(xerr ** 2 + yerr **2)

            if err_tmp < err:
                err = err_tmp
                i_a_close = i

        #print 'error', err
        return err

    def move(self):      

        x_stdev = self.move_stdev[0]
        y_stdev = self.move_stdev[1]
        xdot_stdev = self.move_stdev[2]
        ydot_stdev = self.move_stdev[3]

        self.x = self.x + gauss(0.0,x_stdev) + self.xdot
        self.y = self.y + gauss(0.0,y_stdev) + self.ydot
        #self.xdot = self.xdot + gauss(0.0,xdot_stdev)
        #self.ydot = self.ydot + gauss(0.0,ydot_stdev)

        return self

    def Gaussian(self, mu, sigma, x):
        # from code in Unit 3  
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return numpy.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / numpy.sqrt(2.0 * numpy.pi * (sigma ** 2))

    def calc_likelihood(self, x_list, y_list):

        d = self.sense(x_list, y_list)

        w = self.Gaussian(0,self.sense_stdev[0],d)
        #print 'weight', w
        return w


    def __repr__(self):
        return '[x=%.6s y=%.6s]' % (str(self.x), str(self.y))


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
    rospy.Subscriber('/scan_base',LaserScan,update_scan)

    pub = rospy.Publisher('/frg_point_follower/point2',Point)

    rospy.init_node('frg_pf_laser_point_pub', anonymous=True)

    r = rospy.Rate(20)
    point = Point()


    # Particle Filter settings:
    move_noise = [0.015,0.015,0.001,0.001]
    sense_noise = [0.01]
    wmin = 0.0

    # Generate particles:
    N = 10
    p = []
    for i in range(N):
        x = particle()
        x.set_stdev(move_noise,sense_noise)
        p.append(x)

    print 'Particles created'

    xdot = 0.0
    ydot = 0.0
    xdot_old = 0.0
    ydot_old = 0.0


    while not rospy.is_shutdown():

        #print scan_data.ranges[0]
        a_min = scan_data.angle_min
        a_inc = scan_data.angle_increment
        a_list = [a_min]
        a_max = a_min
        for i in range(len(scan_data.ranges)-1):
            a_max += a_inc
            a_list.append(a_max)

        # Find min and corresponding angle

        d1 = 100000
        a1 = 100000
        dx = 100000
        dy = 100000

        point_valid = 0

        for i in range(len(scan_data.ranges)):
            if d1 > scan_data.ranges[i]:
                d1 = scan_data.ranges[i]
                a1 = a_list[i]
                point_valid = 1

        if point_valid:

            dx3 =  d1 * numpy.cos(a1)
            dy3 =  d1 * numpy.sin(a1)


            #print dx3, dy3

            x_list_d = []
            y_list_d = []
            for i in range(len(a_list)):
                x_list_d.append(scan_data.ranges[i] * numpy.cos(a_list[i]))
                y_list_d.append(scan_data.ranges[i] * numpy.sin(a_list[i]))

            # Here do particle filter stuff
            for i in range(len(p)):
                p[i].set_pos(p[i].x, p[i].y, xdot, ydot)
            for i in range(len(p)):
                p[i].move()
            w = []
            for i in range(len(p)):
                w_tmp = p[i].calc_likelihood(x_list_d,y_list_d)
                w.append(w_tmp)
            ps =[]
            index = int(random() * len(p))
            beta = 0.0
            mw = max(w)
            N = len(p)
            for i in range(N):
                beta += random() * 2.0 * mw
                while beta > w[index]:
                    beta -= w[index]
                    index = (index + 1) % N
                p_sel = p[index]

                # Here deep copy is required to make new particles with the same values
                newp = deepcopy(p_sel)
                ps.append(newp)

            p = ps

            if max(w) < wmin:
                print 'reinitialize half particles'
                # method to pick half the particles at random and reinitialize them
                pnew = []
                for i in range(N/2):
                    rand_index = int(random() * N)
                    rand_selected_particle = deepcopy(p[rand_index])
                    pnew.append(rand_selected_particle)
                while len(pnew)<N:
                    x = particle()
                    x.set_stdev(move_noise,sense_noise)
                    pnew.append(x)
                p = pnew

            if i>30:
                #p = reinitialize_half_particles(p1,w)
                i = 0

            i+=1


            # Update velocities
            x_list_p = []
            y_list_p = []
            for parti in p:
                x_list_p.append(parti.x)
                y_list_p.append(parti.y)

            # xdot = mode(x_list_p)[0][0] - xdot_old
            # ydot = mode(y_list_p)[0][0] - ydot_old
            xdot = numpy.mean(x_list_p) - xdot_old
            ydot = numpy.mean(y_list_p) - ydot_old

            if abs(xdot)>0.15:
                xdot = 0.0
            if abs(ydot)>0.15:
                ydot = 0.0
            xdot_old = xdot
            ydot_old = ydot


            dx3 = numpy.mean(x_list_p)
            dy3 = numpy.mean(y_list_p)
            print dx3, dy3

            dx = 1.0 - (dy3 - 0.2)
            dy = dx3 - 1.0

            x_offset = 0.0
            y_offset = 0.15 #0.19

            point.x = dx + x_offset
            point.y = dy + y_offset
            pub.publish(point)

        r.sleep()

if __name__ == "__main__": 
    publish_state()












