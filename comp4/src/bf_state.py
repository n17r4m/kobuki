#!/usr/bin/env python

# experimenting for fun

import roslib
import rospy
import smach
import smach_ros

class templateMatcher(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['template_matched'])
        
    def execute(self, usrdata):
        rospy.loginfo('Executing template matcher on the side camera')
        # do stuff
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
    sis = smach_ros.IntrospectionServer('simple_viewer', sm, '/SM_ROOT')
    sis.start()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['searching'])
    sm.userdata.sm_counter = 0

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