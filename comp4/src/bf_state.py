#!/usr/bin/env python

# experimenting for fun

import roslib; roslib.load_manifest('smach_exp')
import rospy
import smach
import smach_ros

class templateMatcher(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['locking'],
                                   input_keys = ['matching'],
                                   output_keys = ['template_matched'])
        
    def execute(self, usrdata):
        rospy.loginfo('Executing template matcher on the side camera')
        if usrdata.matching:
            usrdata.template_matched = True
            return 'locking'
        else:
            return 'searching'


class orbMatcher(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['docking'],
                                   input_keys = ['template_matched'],
                                   output_keys = ['locked'])
                                   
    def execute(self, usrdata):
        rospy.loginfo('Executing orb matching on front camera')
        if usrdata.template_matched:
            # do stuff
            usrdata.locked = True
            return 'docking'
            
class Docking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['returning'],
                                   input_keys = ['locked'], 
                                   output_keys = ['docked'])
    
    def execute(self, usrdata):
        rospy.loginfo('Executing docking.')
        if usrdata.locked:
            # do stuff
            usrdata.docked = True
            return 'returning'
            
class back2Searching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['searching'],
                                   input_keys = ['docked'],
                                   output_keys = ['back_to_seach'])
    def execute(self, usrdata):
        if usrdata.docked:
            # do stuff
            usrdata.back_to_seach = True
            return 'searching'
            
def main():
    rospy.init_node('simple_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['searching'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('templateMatching', templateMatcher(), 
                               transitions={'searching':'locking'})
        smach.StateMachine.add('orbMatching', orbMatcher(), 
                               transitions={'locking':'docking'})
        smach.StateMachine.add('docking', Docking(),
                               transitions={'docking':'returning'})
        smach.StateMachine.add('returning', back2Searching(),
                               transitions={'returning':'searching'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()