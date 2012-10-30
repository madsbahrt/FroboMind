#!/usr/bin/env python

#rosrun smach_viewer smach_viewer.py

import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

import behaviours

import actionlib

import smach
import smach_ros

# behaviours used in this statemachine
from fmExecutors.msg import navigate_in_rowAction,navigate_in_rowGoal
from behaviours.frobit import build_find_line_behaviour


if __name__ == "__main__":
    # 
    #
    rospy.init_node("MissionMaster")


    master = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    with master:
        smach.StateMachine.add("FIND_LINE",
                               build_find_line_behaviour(2.5,0.1),
                               transitions={'succeeded':'FOLLOW_LINE'})
        smach.StateMachine.add("FOLLOW_LINE",
                               smach_ros.SimpleActionState('/fmExecutors/follow_line',navigate_in_rowAction,goal=navigate_in_rowGoal(P=1.0)),
                               transitions={'succeeded':'succeeded','aborted':'FIND_LINE','preempted':'preempted'})


    intro_server = smach_ros.IntrospectionServer('field_mission',master,'/FIELDMISSION')
    intro_server.start()    
    
    outcome =  master.execute()
    
    
    
    rospy.signal_shutdown('All done.')

