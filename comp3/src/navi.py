#!/usr/bin/env python

# author: Noni
# date: Mar 14th
# reference:

# navi on a map with four distinguished goals, and loop three times

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
import math

goals = [
    [(-0.45, -12.6, 0.0), (0.0, 0.0,  0.00, 1.00)], # top left corner
    [( 2.00, -12.2, 0.0), (0.0, 0.0,  0.66, 0.75)], # bottom left
    [( 0.64, -6.00, 0.0), (0.0, 0.0, -1.00, 0.00)], # bottom right
    [(-1.93, -6.73, 0.0), (0.0, 0.0, -0.68, 0.74)]  # top right
    ]


def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.header.stamp = rospy.Time.now()
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

position = [0,0]
def amcl_cb(pose):
    global position
    position[0] = pose.pose.pose.position.x
    position[1] = pose.pose.pose.position.y

def close_to_goal(goal):
    x, y = goal.target_pose.pose.position.x, goal.target_pose.pose.position.y
    d = math.sqrt((x - position[0])**2 + (y - position[1])**2)
    print "Distance to goal:", d
    return d < 0.5

go_g = False
def joy_cb(msg):
    global go_g
    if msg.buttons[1]:
        go_g = not go_g

if __name__ == '__main__':
    rospy.init_node('patrol')
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_cb)
    rospy.Subscriber('/joy', Joy, joy_cb)
    rate = rospy.Rate(10)

    try:

        #if go_g:
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        loop = 0
        while loop < 3:
            for pose in goals:
                print "New goal:", pose
                notNear = True
                while notNear:
                    rospy.spin()
                    #if not rospy.is_shutdown():
                    goal = goal_pose(pose)
                    client.send_goal(goal)
                    client.wait_for_result(rospy.Duration.from_sec(0.25))
                    if close_to_goal(goal):
                        notNear = False

            loop += 1


    except rospy.ROSInterruptException:
        pass
