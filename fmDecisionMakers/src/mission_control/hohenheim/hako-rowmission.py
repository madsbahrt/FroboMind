#!/usr/bin/env python

# Import generic python libraries
import threading

# Import generic ros libraries
import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

# import predefined FroboMind behaviours
# e.g. wiimote auto manuel wrapper, follow_plan, stop_and_go etc...


import actionlib
import tf

import smach
import smach_ros

# behaviours used in this statemachine
import behaviours
import behaviours.wii_states.wii_auto_manuel


# Actions used in this statemachine
from fmExecutors.msg import navigate_in_row_simpleAction, navigate_in_row_simpleGoal

from behaviours.turn_behaviours import build_u_turn_sm
from behaviours.row_behaviours import build_row_nav_sm

# messages used 
from sensor_msgs.msg import Joy
    
def force_preempt(a):
    return True

@smach.cb_interface(input_keys=["next_turn_in"],output_keys=["next_turn_out"],outcomes=['left','right'])
def on_turn_selection(ud):
    if ud.next_turn_in == "left":
        ud.next_turn_out = "right"
        return 'left'
    else:
        ud.next_turn_out = "left"
        return 'right'

def build_main_sm():
    """
        Construct the state machine executing the selected behaviours and actions
    """
    
    #
    # Create the inrow behaviour
    # 
    row_goal = navigate_in_row_simpleGoal()
    row_goal.desired_offset_from_row = 0
    row_goal.distance_scale = -0.5
    row_goal.forward_velcoity = 0.5
    row_goal.headland_timeout = 20
    row_goal.P = 1.5
    
    row_nav = build_row_nav_sm(row_goal,2)


    #length_in, length_out, width, turn_radius , direction_l,vel_fw,vel_turn):
    uturn_right = build_u_turn_sm(2,1.5, 1.7, 0.5, False, 0.5, 0.4)
    uturn_left = build_u_turn_sm(2,1.5, 1.7, 0.5, True, 0.5, 0.4)
    
    main_sm = smach.StateMachine(["succeeded","aborted","preempted"])
    main_sm.userdata.next_turn = "right"
    
    with main_sm:
        smach.StateMachine.add("NAVIGATE_IN_ROW",
                                row_nav,
                                transitions={"succeeded":"TURN_SELECTOR"},
                                )
        smach.StateMachine.add("TURN_SELECTOR",
                               smach.CBState(on_turn_selection, outcomes=["left","right"], input_keys=["next_turn_in"], output_keys=["next_turn_out"]),
                               transitions={"left":"MAKE_TURN_LEFT","right":"MAKE_TURN_RIGHT"},
                               remapping = {"next_turn_in":"next_turn","next_turn_out":"next_turn"}
                               
                               )
        smach.StateMachine.add("MAKE_TURN_RIGHT",
                               uturn_right,
                               transitions={"succeeded":"NAVIGATE_IN_ROW"}
                               )
        smach.StateMachine.add("MAKE_TURN_LEFT",
                               uturn_left,
                               transitions={"succeeded":"NAVIGATE_IN_ROW"}
                               )

    return main_sm
    
    
if __name__ == "__main__":
    
    rospy.init_node("field_mission")
    
    sm = build_main_sm()
    
    intro_server = smach_ros.IntrospectionServer('field_mission',sm,'/FIELDMISSION')
    intro_server.start()    
    
    smach_thread = threading.Thread(target = sm.execute)
    smach_thread.start()
    
    rospy.spin();

    sm.request_preempt()
    intro_server.stop()