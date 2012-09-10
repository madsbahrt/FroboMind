#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

import behaviours

import actionlib
import math
import tf

from sensor_msgs.msg import Joy

import smach
import smach_ros

# behaviours used in this statemachine
from fmDecisionMakers.msg import navigate_in_row_simpleAction, navigate_in_row_simpleGoal
from fmDecisionMakers.msg import make_turnAction, make_turnGoal, drive_forwardAction, drive_forwardGoal
from fmImplements.msg import move_tool_simpleAction, move_tool_simpleGoal
from fmMsgs.msg import row
import behaviours.wii_states.wii_auto_manuel


def on_rows(ud,msg):
    if msg.leftvalid and msg.rightvalid:
        return False
    else:
        return True
    
def force_preempt(a):
    return True

def up_pressed(ud,msg):
    if msg.buttons[0]:
        return False
    else:
        return True

def down_pressed(ud,msg):
    if msg.buttons[1]:
        return False
    else:
        return True
    
@smach.cb_interface(input_keys=['last_action'], output_keys=['new_action'],outcomes=['left','right'])
def turn_left_or_right(ud):
    if ud.last_action == "left":
        ud.new_action = "right"
        return "right"
    else:
        ud.new_action = "left"
        return "left"    
    
    
def build_fish_tail(turn_vel,drive_vel):
    fish_tail = smach.Sequence(outcomes=['succeeded','aborted','preempted'], connector_outcome='succeeded')
    
    with fish_tail:
        smach.Sequence.add("TURN1", 
                           smach_ros.SimpleActionState("/fmDecisionMakers/make_turn",
                                        make_turnAction,
                                        goal=make_turnGoal(amount=0.5025,vel=turn_vel))
                           )
        smach.Sequence.add("DRIVE1",
                           smach_ros.SimpleActionState("/fmDecisionMakers/drive_forward",
                                        drive_forwardAction,
                                        goal=drive_forwardGoal(amount=1,vel=drive_vel))
                           )
        smach.Sequence.add("TURN2", 
                           smach_ros.SimpleActionState("/fmDecisionMakers/make_turn",
                                        make_turnAction,
                                        goal=make_turnGoal(amount=1.05,vel=turn_vel))
                           )
        smach.Sequence.add("DRIVE_REV1",
                           smach_ros.SimpleActionState("/fmDecisionMakers/drive_forward",
                                        drive_forwardAction,
                                        goal=drive_forwardGoal(amount=-1,vel=drive_vel))
                           )
        smach.Sequence.add("TURN_REV1", 
                           smach_ros.SimpleActionState("/fmDecisionMakers/make_turn",
                                        make_turnAction,
                                        goal=make_turnGoal(amount=1.05,vel=turn_vel))
                           )
        smach.Sequence.add("DRIVE_REV2",
                           smach_ros.SimpleActionState("/fmDecisionMakers/drive_forward",
                                        drive_forwardAction,
                                        goal=drive_forwardGoal(amount=1,vel=drive_vel))
                           )
        smach.Sequence.add("TURN_REV2", 
                           smach_ros.SimpleActionState("/fmDecisionMakers/make_turn",
                                        make_turnAction,
                                        goal=make_turnGoal(amount=0.5025,vel=turn_vel))
                           )
        
    return fish_tail

def build_raise_lower_boom():
    btn_monitors = smach.Concurrence(outcomes=['raise','lower','succeeded'],
                                     default_outcome='succeeded',
                           outcome_map={
                                        "raise":{'MOVE_UP_BTN':'invalid','MOVE_DOWN_BTN':'preempted'}, 
                                        "lower":{'MOVE_UP_BTN':'preempted','MOVE_DOWN_BTN':'invalid'}
                                        },
                           child_termination_cb=force_preempt)
    
    with btn_monitors:
        smach.Concurrence.add("MOVE_UP_BTN", smach_ros.MonitorState("/fmHMI/joy", Joy, up_pressed, -1))
        smach.Concurrence.add("MOVE_DOWN_BTN", smach_ros.MonitorState("/fmHMI/joy", Joy, down_pressed, -1))
        
    sm = smach.StateMachine(outcomes=["succeeded","aborted","preempted"])
    
    with sm:
        smach.StateMachine.add("MONITOR_BUTTONS", 
                               btn_monitors, 
                               transitions={"raise":"MOVE_UP","lower":"MOVE_DOWN","succeeded":"MONITOR_BUTTONS"})
        smach.StateMachine.add("MOVE_UP",
                               smach_ros.SimpleActionState("/fmImplements/RowClean/move_tool",move_tool_simpleAction,goal=move_tool_simpleGoal(direction=1,timeout=10)),
                               transitions = {"succeeded":"MONITOR_BUTTONS","aborted":"aborted","preempted":"preempted"}
                               )
        smach.StateMachine.add("MOVE_DOWN",
                               smach_ros.SimpleActionState("/fmImplements/RowClean/move_tool",move_tool_simpleAction,goal=move_tool_simpleGoal(direction=0,timeout=10)),
                               transitions = {"succeeded":"MONITOR_BUTTONS","aborted":"aborted","preempted":"preempted"}
                               )
        
    return sm

def build_turn_sm(forward_vel,turn_vel,turn_left=True):
    
    turn_sm = smach.Sequence(outcomes=["succeeded","aborted","preempted"],connector_outcome='succeeded')
    with turn_sm:
        smach.Sequence.add(
            "DRIVE_OUT_OF_ROW", 
            smach_ros.SimpleActionState("/fmDecisionMakers/drive_forward",
                                        drive_forwardAction,
                                        goal=drive_forwardGoal(amount=2.5,vel=forward_vel))
        )
        if turn_left:
            smach.Sequence.add(
                "TURN_LEFT", 
                smach_ros.SimpleActionState("/fmDecisionMakers/make_turn",
                                            make_turnAction,
                                            goal=make_turnGoal(amount=math.pi/2,vel=turn_vel))
            )
        else:
            smach.Sequence.add(
                "TURN_RIGHT", 
                smach_ros.SimpleActionState("/fmDecisionMakers/make_turn",
                                            make_turnAction,
                                            goal=make_turnGoal(amount=-math.pi/2,vel=turn_vel))
            )
        smach.Sequence.add(
            "DRIVE_TO_NEXT_ROW", 
            smach_ros.SimpleActionState("/fmDecisionMakers/drive_forward",
                                        drive_forwardAction,
                                        goal=drive_forwardGoal(amount=0.5,vel=forward_vel))
        )
        if turn_left:
            smach.Sequence.add(
                "TURN_LEFT_INTO_ROW", 
                smach_ros.SimpleActionState("/fmDecisionMakers/make_turn",
                                            make_turnAction,
                                            goal=make_turnGoal(amount=math.pi/2,vel=turn_vel)) 
            )
        else:
            smach.Sequence.add(
                "TURN_RIGHT_INTO_ROW", 
                smach_ros.SimpleActionState("/fmDecisionMakers/make_turn",
                                            make_turnAction,
                                            goal=make_turnGoal(amount=-math.pi/2,vel=turn_vel)) 
            )
        smach.Sequence.add(
            "DRIVE_INTO_ROW", 
            smach_ros.SimpleActionState("/fmDecisionMakers/drive_forward",
                                        drive_forwardAction,
                                        goal=drive_forwardGoal(amount=1.5,vel=forward_vel))
        )
    return turn_sm

def build_sm():
    """
        Construct the state machine executing the selected behaviours
    """
    
    
    row_goal = navigate_in_row_simpleGoal()
    row_goal.desired_offset_from_row = -0.2
    row_goal.distance_scale = -0.8
    row_goal.forward_velcoity = 0.5
    row_goal.headland_timeout = 20
    row_goal.P = 2
    
    find_row_timeout_sm = smach.Concurrence(outcomes=['succeeded','aborted','preempted'], 
                           default_outcome='aborted',
                           outcome_map={
                                        "aborted":{'TIMEOUT':'succeeded','FIND_ROW':'preempted'}, 
                                        "succeeded":{'FIND_ROW':'invalid'}},
                           child_termination_cb=force_preempt)
    
    with find_row_timeout_sm:
        smach.Concurrence.add("FIND_ROW", smach_ros.MonitorState("/fmExtractors/rows", row, on_rows, -1))
        smach.Concurrence.add("TIMEOUT" , behaviours.WaitState(2))
        
    row_navigator = smach.StateMachine(outcomes=["succeeded","aborted","preempted"])
    row_navigator.userdata.distance_driven_in_row = 0
    
    with row_navigator:
        smach.StateMachine.add("FIND_ROW_WITH_TIMEOUT", 
                               find_row_timeout_sm,
                               transitions={'succeeded':'NAVIGATE_IN_ROW','aborted':'aborted','preempted':'preempted'}
                               )
        
        smach.StateMachine.add("NAVIGATE_IN_ROW", 
                               smach_ros.SimpleActionState("/fmDecisionMakers/navigate_in_row_simple", navigate_in_row_simpleAction,row_goal,result_slots=['distance_traveled']),
                               transitions={'succeeded':'succeeded','aborted':'aborted','preempted':'preempted'},
                               remapping={'distance_traveled':'distance_driven_in_row'}
                               )
    
    #complete_row_turn = build_turn_sm(0.5,0.5)
    left_turn = build_turn_sm(0.5,0.5,True)
    right_turn = build_turn_sm(0.5,0.5,False)
    
    master = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    master.userdata.last_action = "right"
    master.userdata.distance_driven_in_row = 0
        
    with master:
        smach.StateMachine.add("LOWER_IMPLEMENT",
                                smach_ros.SimpleActionState("/fmImplements/RowClean/move_tool",move_tool_simpleAction,goal=move_tool_simpleGoal(direction=0,timeout=7)),
                                transitions = {"succeeded":"DRIVE_IN_ROW","aborted":"aborted","preempted":"preempted"}
                                )
        smach.StateMachine.add("DRIVE_IN_ROW",
                               row_navigator,
                               transitions={'succeeded':'RAISE_IMPLEMENT','aborted':'aborted','preempted':'preempted'},
                               remapping = {'distance_driven_in_row':'distance_driven_in_row'}
                               )
        smach.StateMachine.add("RAISE_IMPLEMENT",
                                smach_ros.SimpleActionState("/fmImplements/RowClean/move_tool",move_tool_simpleAction,goal=move_tool_simpleGoal(direction=1,timeout=7)),
                                transitions = {"succeeded":"LEFT_OR_RIGHT","aborted":"aborted","preempted":"preempted"}
                                )
        smach.StateMachine.add("LEFT_OR_RIGHT", 
                               smach.CBState(turn_left_or_right), 
                               transitions={"right":"TURN_OUTROW_RIGHT","left":"TURN_OUTROW_LEFT"}, 
                               remapping = {"last_action":"last_action","new_action":"last_action"})
        smach.StateMachine.add("TURN_OUTROW_LEFT",
                               left_turn,
                               transitions = {"succeeded":"LOWER_IMPLEMENT","aborted":"aborted","preempted":"preempted"})
        smach.StateMachine.add("TURN_OUTROW_RIGHT",
                               right_turn,
                               transitions = {"succeeded":"LOWER_IMPLEMENT","aborted":"aborted","preempted":"preempted"})

    m2 = behaviours.wii_states.wii_auto_manuel.create(master, "/fmHMI/joy", 2)
    
    m3 = smach.Concurrence(outcomes=['succeeded','aborted','preempted'], 
                           default_outcome='aborted',
                           outcome_map={},
                           child_termination_cb=force_preempt)
    with m3:
        smach.Concurrence.add("MASTER",m2)
        smach.Concurrence.add("MOVE_IMPLEMENT_MANUAL",build_raise_lower_boom())
    
    return m3
    
    
    
if __name__ == "__main__":
    
    rospy.init_node("field_mission")
    
    sm = build_sm()
    
    intro_server = smach_ros.IntrospectionServer('field_mission',sm,'/FIELDMISSION')
    intro_server.start()
    
    
    try:
        outcome = sm.execute()
    except:
        rospy.logerr("Statemachine aborted")
        
    intro_server.stop()
    
    rospy.signal_shutdown('All done.')