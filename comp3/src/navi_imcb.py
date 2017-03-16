#!/usr/bin/env python

# author: Noni
# date: Mar 14th
# reference:

# navi on a map with four distinguished goals, and loop three times

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Joy, Image
import math

goals = [
    [(-0.45, -12.6, 0.0), (0.0, 0.0,  0.00, 1.00)], # top left corner
    [( 2.00, -12.2, 0.0), (0.0, 0.0,  0.66, 0.75)], # bottom left
    [( 0.64, -6.00, 0.0), (0.0, 0.0, -1.00, 0.00)], # bottom right
    [(-1.93, -6.50, 0.0), (0.0, 0.0, -0.68, 0.74)]  # top right
    ]
    
cur_goal = 0
cur_loop = 0
position = [0,0]
go_g = False


def next_goal():
    global cur_goal, cur_loop
    cur_goal = (cur_goal + 1) % 4
    if cur_goal is 0:
        cur_loop += 1

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

def tick_cb(msg):
    global client, goals, cur_goal, cur_loop, position, go_g
    pose = goal_pose(goals[cur_goal])
    if close_to_goal(pose):
        next_goal()
        pose = goal_pose(goals[cur_goal])
    if cur_loop <= 3:
        client.send_goal(pose)
        client.wait_for_result(rospy.Duration.from_sec(2))
        

def amcl_cb(pose):
    global position
    position[0] = pose.pose.pose.position.x
    position[1] = pose.pose.pose.position.y

def close_to_goal(pose):
    x, y = pose.target_pose.pose.position.x, pose.target_pose.pose.position.y
    d = math.sqrt((x - position[0])**2 + (y - position[1])**2)
    print "Distance to goal:", d
    return d < 0.2


def joy_cb(msg):
    global go_g
    if msg.buttons[1]:
        go_g = not go_g

if __name__ == '__main__':
    rospy.init_node('patrol')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_cb)
    rospy.Subscriber('/joy', Joy, joy_cb)
    rospy.Subscriber('/camera/rgb/image_raw', Image, tick_cb) #is there a better "ticker?"
    rospy.spin()
