#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

import behaviours

import actionlib
import tf

import threading
from sensor_msgs.msg import Joy

import smach
import smach_ros

# behaviours used in this statemachine
from fmDecisionMakers.msg import navigate_in_rowAction, navigate_in_rowGoal
from fmDecisionMakers.msg import find_rowAction, find_rowGoal

import behaviours.wii_states.wii_auto_manuel


from fmTools.srv import switch_muxRequest,switch_muxResponse,switch_mux

def build_sm():
    """
        Construct the state machine executing the selected behaviours
    """
    row_navigator = smach.StateMachine(outcomes=["succeeded","aborted","preempted"])
    
    find_goal = find_rowGoal()
    find_goal.quality = 200
    
    nav_goal = navigate_in_rowGoal()

    nav_goal.desired_offset_from_row = 0.5
    nav_goal.replan_heading_threshold = 0.1
    nav_goal.replan_offset_threshold = 10
    nav_goal.required_quality = 200

    
    with row_navigator:
        smach.StateMachine.add("FIND_ROW", 
                               smach_ros.SimpleActionState("/fmDecisionMakers/find_row", find_rowAction,find_goal),
                               transitions={'succeeded':'NAVIGATE_IN_ROW'}
                               )
        
        smach.StateMachine.add("NAVIGATE_IN_ROW", 
                               smach_ros.SimpleActionState("/fmDecisionMakers/navigate_in_row", navigate_in_rowAction,nav_goal),
                               transitions={'succeeded':'FIND_ROW'}
                               )
        
    return behaviours.wii_states.wii_auto_manuel.create(row_navigator, "/fmHMI/joy", 3)
    
    
    
if __name__ == "__main__":
    
    rospy.init_node("field_mission")
    
    sm = build_sm()
    
    intro_server = smach_ros.IntrospectionServer('field_mission',sm,'/FIELDMISSION')
    intro_server.start()    
    
    smach_thread = threading.Thread(target = sm.execute)
    smach_thread.start()
    
    rospy.spin();

    sm.request_preempt()
    intro_server.stop()
