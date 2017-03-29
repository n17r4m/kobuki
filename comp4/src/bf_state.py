#!/usr/bin/env python

# experimenting for fun

import roslib
import rospy
import smach
import smach_ros
from comp4_bf import *

class templateMatcher(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['template_matched'], input_keys = ['start_matching'], output_keys=['tm_done'])
        
    def execute(self, usrdata):
        rospy.loginfo('Executing template matcher on the side camera')
        if usrdata.start_matching:
            # do matching
        if usrdata.tm_done:
            return 'template_matched'

class orbMatcher(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['pose_found'])
                                   
    def execute(self, usrdata):
        rospy.loginfo('Executing orb matching on front camera')
        # do stuff
        return 'pose_found'
        
class Docking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['docked'])
    
    def execute(self, usrdata):
        rospy.loginfo('Executing docking.')
        # do stuff
        return 'docked'
            
class back2Searching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['returned'])
    def execute(self, usrdata):
        rospy.loginfo('Returning back to the track')
        # do stuff
        return 'returned'
            
def main():
    rospy.init_node('simple_state_machine')
    comp4 = Comp4()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['searching'])    
    
    sis = smach_ros.IntrospectionServer('simple_viewer', sm, '/SM_ROOT')
    sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('searching', templateMatcher(), 
                               transitions={'template_matched':'locking'})
        smach.StateMachine.add('locking', orbMatcher(), 
                               transitions={'pose_found':'docking'})
        smach.StateMachine.add('docking', Docking(),
                               transitions={'docked':'returning'})
        smach.StateMachine.add('returning', back2Searching(),
                               transitions={'returned':'searching'})


    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()