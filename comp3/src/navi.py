#!/usr/bin/env python

# author: Noni
# date: Mar 14th
# reference: 

# navi on a map with four distinguished goals, and loop three times

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
import math

goals = [
    [(-0.95, -9.67, 0.0), (0.0, 0.0, -0.61, 0.79)], # top left corner
    [( 0.22, -9.67, 0.0), (0.0, 0.0,  0.01, 1.00)], # bottom left
    [( 0.14, -5.86, 0.0), (0.0, 0.0, -0.99, 0.14)], # bottom right
    [(-1.57, -6.52, 0.0), (0.0, 0.0, -0.99, 0.19)]  # top right
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
    
position = (0,0)
def amcl_cb(pose):
    global position
    position[0] = pose.position.x
    position[1] = pose.position.y

def close_to_goal(goal):
    x, y = goal.target_pose.pose.position.x, goal.target_pose.pose.position.y
    return math.sqrt((x - position[0])**2 + (y - position[1])**2) < 0.05
    
if __name__ == '__main__':
    rospy.init_node('patrol')
    rospy.Subscriber('/amcl_pose', Pose, amcl_cb)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()
    
    loop = 0
    while loop < 3:
        for pose in goals: 
            notNear = True
            while notNear:
                goal = goal_pose(pose)
                client.send_goal(goal)
                client.wait_for_result(rospy.Duration.from_sec(0.5))
                if close_to_goal(goal):
                    notNear = False
        
        loop += 1