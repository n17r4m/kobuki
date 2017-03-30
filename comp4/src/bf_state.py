#!/usr/bin/env python

# experimenting for fun

import roslib
import rospy
import smach
import smach_ros
from bf_alg import *


class templateMatcher(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['template_matched'])
        
    def execute(self, usrdata):
        rospy.loginfo('Executing template matcher on the side camera')
        try:
            template = TemplateMatcher()
            if template.status == 'searching':
                pass
            elif template.status == 'ready2dock':
                return 'template_matched'

class orbMatcher(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['docked'])
                                   
    def execute(self, usrdata):
        rospy.loginfo('Executing orb matching on front camera')
        try:
            orb = OrbTracker()
            if orb.status == 'notyet':
                pass
            elif orb.status == 'docked':
                return 'docked'
        except rospy.ROSInterruptException:
            pass

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
                               transitions={'docked':'returning'})
        smach.StateMachine.add('returning', back2Searching(),
                               transitions={'returned':'searching'})


    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()