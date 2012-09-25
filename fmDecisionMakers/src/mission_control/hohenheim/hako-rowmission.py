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



def build_main_sm():
    """
        Construct the state machine executing the selected behaviours and actions
    """
    
    #
    # Create the inrow behaviour
    # 
    row_goal = navigate_in_row_simpleGoal()
    row_goal.desired_offset_from_row = -0.2
    row_goal.distance_scale = -0.8
    row_goal.forward_velcoity = 0.5
    row_goal.headland_timeout = 20
    row_goal.P = 2
    
    row_nav = build_row_nav_sm(row_goal,2)

    uturn = build_u_turn_sm(2, 1.5, 0.5, False, 0.5, 0.4)
    
    
    

    return uturn
    
    
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