#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

import behaviours

import actionlib
import math
import tf

from sensor_msgs.msg import Joy
import threading

import smach
import smach_ros

# behaviours used in this statemachine
from fmExecutors.msg  import navigate_in_row_simpleAction, navigate_in_row_simpleGoal
from fmExecutors.msg  import make_turnAction, make_turnGoal, drive_forwardAction, drive_forwardGoal
from fmMsgs.msg import row, lidar_safety_zone
import behaviours.wii_states.wii_auto_manuel


from fmTools.srv import switch_muxRequest,switch_muxResponse,switch_mux

def on_rows(ud,msg):
    if msg.leftvalid and msg.rightvalid:
        return False
    else:
        return True
    
def on_safety_yellow(ud,msg):
    if msg.yellow_activated:
        return False
    else:
        return True

def on_safety_yellow_neg(ud,msg):
    if msg.yellow_activated:
        return True
    else:
        return False
    
def on_safety_red(ud,msg):
    if msg.red_activated:
        return False
    else:
        return True

def on_safety_red_neg(ud,msg):
    if msg.red_activated:
        return True
    else:
        return False
    
def force_preempt(a):
    return True

def build_sm():
    """
        Construct the state machine executing the selected behaviours
    """
    row_navigator = smach.StateMachine(outcomes=["succeeded","aborted","preempted"])
    
    row_goal = navigate_in_row_simpleGoal()
    
    row_goal.desired_offset_from_row = 1.1
    row_goal.distance_scale = -0.3
    row_goal.forward_velcoity = 0.8
    row_goal.headland_timeout = 0
    row_goal.P = 1.2
    
    turn_goal = make_turnGoal()
    turn_goal.amount = math.pi
    turn_goal.vel = 0.6
    
    wiggle_goal = make_turnGoal()
    wiggle_goal.amount = 0.1
    wiggle_goal.vel = 0.5
    
    fw_goal = drive_forwardGoal()
    fw_goal.amount = 2
    fw_goal.vel = 0.6
    
    find_row_timeout_sm = smach.Concurrence(outcomes=['succeeded','aborted','preempted'], 
                           default_outcome='aborted',
                           outcome_map={
                                        "aborted":{'TIMEOUT':'succeeded','FIND_ROW':'preempted'}, 
                                        "succeeded":{'FIND_ROW':'invalid'}},
                           child_termination_cb=force_preempt)
    
    with find_row_timeout_sm:
        smach.Concurrence.add("FIND_ROW", smach_ros.MonitorState("/fmExtractors/rows", row, on_rows, -1))
        smach.Concurrence.add("TIMEOUT" , behaviours.WaitState(2))
        
    with row_navigator:
        smach.StateMachine.add("FIND_ROW_WITH_TIMEOUT", 
                               find_row_timeout_sm,
                               transitions={'succeeded':'NAVIGATE_IN_ROW','aborted':'NAVIGATE_IN_ROW'}
                               )
        
        smach.StateMachine.add("NAVIGATE_IN_ROW", 
                               smach_ros.SimpleActionState("/navigate_in_row_simple", navigate_in_row_simpleAction,row_goal),
                               transitions={'succeeded':'aborted'}
                               )
    
    laser_state = smach.Concurrence(outcomes=['yellow_active','red_active'], 
                                    outcome_map={'yellow_active':{'WAIT_FOR_YELLOW':'invalid','WAIT_FOR_RED':'preempted'},
                                                 'red_active'   :{'WAIT_FOR_RED':'invalid'}}
                                    ,default_outcome='red_active',child_termination_cb=force_preempt)
    
    red_active = smach_ros.MonitorState("/fmExtractors/safety_zone",lidar_safety_zone,on_safety_red, max_checks=-1)
    red_not_active = smach_ros.MonitorState("/fmExtractors/safety_zone",lidar_safety_zone,on_safety_red_neg, max_checks=-1)
    
    yellow_active = smach_ros.MonitorState("/fmExtractors/safety_zone",lidar_safety_zone,on_safety_yellow, max_checks=-1)
    yellow_not_active = smach_ros.MonitorState("/fmExtractors/safety_zone",lidar_safety_zone,on_safety_yellow_neg, max_checks=-1)
    with laser_state:
        smach.Concurrence.add("WAIT_FOR_YELLOW",
                               yellow_active
                               )
        smach.Concurrence.add("WAIT_FOR_RED",
                              red_active
                              )

    
    control_state = smach.Concurrence(outcomes=['stop','retract','aborted'],
                                      outcome_map={'stop':{'MONITOR_LASER':'yellow_active'},
                                                   'retract':{'MONITOR_LASER':'red_active'},
                                                   'aborted':{'NAVIGATE_IN_ROW':'aborted'}}, 
                                      default_outcome='stop',
                                      child_termination_cb=force_preempt)

    with control_state:
        smach.Concurrence.add("MONITOR_LASER", laser_state)
        smach.Concurrence.add("NAVIGATE_IN_ROW", row_navigator)
       
       
        
    stop_state = smach.Concurrence(outcomes=['continue','retreat'],
                                      outcome_map={'continue':{'MONITOR_YELLOW':'invalid','MONITOR_RED':'preempted'},
                                                   'retreat':{'MONITOR_RED':'invalid'}
                                                   },
                                      default_outcome='continue',
                                      child_termination_cb=force_preempt)
    with stop_state:
        smach.Concurrence.add("MONITOR_RED", red_active)
        smach.Concurrence.add("MONITOR_YELLOW",yellow_not_active)
    
    
    retreat_state = smach.StateMachine(outcomes=['clear','aborted','preempted'])
    
    with retreat_state:
        smach.StateMachine.add("BACK_UP",
                               smach_ros.SimpleActionState("/drive_forward",drive_forwardAction,drive_forwardGoal(amount=-0.3,vel=0.3)),
                               transitions ={'succeeded':'MONITOR_RED','aborted':'aborted','preempted':'preempted'})
        smach.StateMachine.add("MONITOR_RED",
                               red_not_active,
                               transitions={'valid':'BACK_UP','invalid':'clear'})
        
    
    master = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
        
    with master:
        smach.StateMachine.add("DRIVE_IN_ROW",
                               control_state,
                               transitions={'stop':'STOP_STATE','retract':'RETREAT_STATE','aborted':'TURN_180'})
        smach.StateMachine.add("STOP_STATE",
                               stop_state,
                               transitions={'continue':'DRIVE_IN_ROW','retreat':'RETREAT_STATE'})
        smach.StateMachine.add("RETREAT_STATE",
                               retreat_state,
                               transitions={'clear':'TURN_180','aborted':'aborted'})
        smach.StateMachine.add("TURN_180",
                               smach_ros.SimpleActionState("/make_turn", make_turnAction, goal=make_turnGoal(amount=math.pi-0.01,vel=0.6)),
                               transitions={'succeeded':'DRIVE_IN_ROW'})
        
    return master
    
    
    
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
