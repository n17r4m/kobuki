#!/usr/bin/python

import rospy
import roslib
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from geometry_msgs.msg import Point, PointStamped
from tf import TransformListener
import sys


# This single person trackers requires the multi-person tracker to also be running


class SinglePersonTracker:
    MAX_LOCK_ON_DIST = 2.0
    EXPECTED_STARTING_POSITION_X = 2.0
    EXPECTED_STARTING_POSITION_Y = 0.0

    def __init__(self):        
        self.tf = TransformListener()


        self.person_position = Point()
        self.person_position.x = self.EXPECTED_STARTING_POSITION_X
        self.person_position.y = self.EXPECTED_STARTING_POSITION_Y
        
        self.locked_on = False
        self.person_id = -1
        
        self.single_person_tracked_pub = rospy.Publisher('/frg_point_follower/point2', Point)
        self.people_tracked_sub = rospy.Subscriber('people_tracked', PoseArray, self.tracked_people_callback)
        

        self.FIXED_FRAME = None
        if len(sys.argv) == 4:
            self.FIXED_FRAME = sys.argv[1]
        else:
            print "ERRORS: incorrect number of arguments passed to kalman_confidence_person_tracker"
            
            
        

    def tracked_people_callback(self, msg):
        rospy.loginfo(self.person_position.x)
        if self.locked_on:
            # If we're locked on to one person, just use their ID to find their position
            person_found = False
            for pose in msg.poses:
                if pose.position.z == self.person_id:
                    person_found = True

                    new_pose = PoseStamped()
                    new_pose.pose.position.x = pose.position.x
                    new_pose.pose.position.y = pose.position.y
                    new_pose.header.frame_id = msg.header.frame_id
                    new_pose.header.stamp = self.tf.getLatestCommonTime("base_footprint", "odom")
                    pose_tf = self.tf.transformPose("base_footprint", new_pose)

                    self.person_position.x = pose_tf.pose.position.x
                    self.person_position.y = pose_tf.pose.position.y  
                    break                  
                     
            if not person_found:
                self.locked_on = False
                rospy.loginfo("Lost our person")
                rospy.loginfo(self.locked_on)
                
        else:
            # If we haven't locked onto to someone, find the closest person to last locked on position
            closest_dist = self.MAX_LOCK_ON_DIST
            closest_x = None
            closest_y = None
            closest_id = None

            for pose in msg.poses:
                new_pose = PoseStamped()
                new_pose.pose.position.x = pose.position.x
                new_pose.pose.position.y = pose.position.y
                new_pose.header.frame_id = msg.header.frame_id
                new_pose.header.stamp = self.tf.getLatestCommonTime("base_footprint", "odom")


                pose_tf = self.tf.transformPose("base_footprint", new_pose)

                rel_x = pose_tf.pose.position.x - self.person_position.x
                rel_y = pose_tf.pose.position.y - self.person_position.y
                dist = (rel_x**2 + rel_y**2)**(1./2.)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_x = pose_tf.pose.position.x
                    closest_y = pose_tf.pose.position.y
                    closest_id = pose.position.z # Note the z position is actually the ID num
                    self.locked_on = True
                    
            if self.locked_on:
                self.person_position.x = closest_x
                self.person_position.y = closest_y    
                self.person_id = closest_id        
                rospy.loginfo("Locked on to a person")
            
                    
                
        




    def publish_person(self):
        if self.locked_on:         
            #rospy.logwarn('hello2')
            person_position_offset =self.person_position
            person_position_offset.y -= 0.15

            self.single_person_tracked_pub.publish(person_position_offset)
#        else:
#            # Publish a point at (0,0,0) if we're not locked onto a person
#            dummy_point = Point()
#            dummy_point.x = 0.0
#            dummy_point.y = 0.0
#            dummy_point.z = 0.0
#            self.single_person_tracked_pub.publish(dummy_point)
        


    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_person()
            rate.sleep()




if __name__ == '__main__':
    rospy.init_node('single_person_tracker', anonymous=True)
    spt = SinglePersonTracker()
    spt.spin()




