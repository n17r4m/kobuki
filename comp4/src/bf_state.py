#!/usr/bin/env python

# experimenting for fun

import roslib
import rospy
import smach
import smach_ros
from bf_alg import *

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# global turning points
turning_goals = [
      [(-0.18, 0.24, 0.0), (0.0, 0.0, 0.73, 0.67)],
      [(-0.22, 4.17, 0.0), (0.0, 0.0, -0.98, 0.14)],
      [(-10.89, 3.73, 0.0), (0.0, 0.0, -0.73, 0.68)],
      [(-10.42, -0.35, 0.0), (0.0, 0.0, 0.15, 0.98)]
]


class templateMatcher(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['template_matched', 'not_matching'])
        self.sound = SoundClient()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.Subscriber('/joy', Joy, self.joy_cb)
        self.can_go = False

        #rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)

    def joy_cb(self, msg):
        if msg.buttons[1]:
            self.can_go = not self.can_go

    def execute(self, userdata):
        rospy.loginfo('Executing template matcher on the side camera')
        if template1.status == 'ready2dock' or template2.status == 'ready2dock':
            self.sound.say('found one marker')
            return 'template_matched'
        else:
            '''
            for pose in turning_goals:
                goal = goal_pose(pose)
                self.client.send_goal(goal)
                self.client.wait_for_result()
            '''
            return 'not_matching'

class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['confirmed'])
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
        self.twist = Twist()

    def execute(self, userdata):
        self.twist.angular.z = 0
        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(4)
        return 'confirmed'

class turning90(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['turned'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)
        self.pose = None

    def amcl_cb(self, msg):
        self.pose = msg.pose.pose

    def execute(self, userdata):
        rospy.loginfo('Turning 90 degrees')
        qo = np.array([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
        qz = tf.transformations.quaternion_about_axis(-3.14159/2.0, (0,0,1))
        q = tf.transformations.quaternion_multiply(qo, qz)
        goal = self.pose
        goal.orientation.x = q[0]
        goal.orientation.y = q[1]
        goal.orientation.z = q[2]
        goal.orientation.w = q[3]
        goal = goal_pose(goal)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return 'turned'

class orbMatcher(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['docked', 'not_docked'])

    def execute(self, userdata):
        rospy.loginfo('Executing orb matching on front camera')
        if orb1.status == 'docked' or orb2.status == 'docked':
            return 'docked'
        else:
            return 'not_docked'


class back2Searching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['returned'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        rospy.loginfo('Returning back to the track')
        goal = returning_points.pop()
        goal = goal_pose(goal)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return 'returned'


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

def main():
    rospy.init_node('simple_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['not_matching'])

    # Open the container
    with sm:
        # Add states to the container
        """
        smach.StateMachine.add('searching', templateMatcher(),
                               transitions={'template_matched':'turning90', 'not_matching':'searching'})
        smach.StateMachine.add('turning90', turning90(),
                               transitions={'turned': 'locking'})
        smach.StateMachine.add('locking', orbMatcher(),
                               transitions={'docked':'returning', 'not_docked':'locking'})
        smach.StateMachine.add('returning', back2Searching(),
                               transitions={'returned':'searching'})
        """
        smach.StateMachine.add('searching', templateMatcher(),
                               transitions = {'template_matched':'Stop', 'not_matching':'searching'})
        smach.StateMachine.add('Stop', Stop(),
                               transitions = {'confirmed': 'searching'})

    sis = smach_ros.IntrospectionServer('SM', sm, '/IMPRROC_BOTTOM')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    template1 = TemplateMatcher('ua_small.png', threshold = 0.23)
    template2 = TemplateMatcher('ar_small.png', threshold = 0.34)
    #orb1 = OrbTracker('ua.png')
    #orb2 = OrbTracker('ar.png')
    #can_go = False
    main()
