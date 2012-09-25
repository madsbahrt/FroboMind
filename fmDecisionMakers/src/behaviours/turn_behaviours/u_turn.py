#!/usr/bin/env python

# Import generic ros libraries
import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

import math

import actionlib

import smach
import smach_ros

# Actions used in this statemachine
from fmExecutors.msg import drive_forwardGoal, drive_forwardAction,make_turnGoal,make_turnAction


def build_u_turn_sm(length_in, length_out, width, turn_radius , direction_l,vel_fw,vel_turn):
    """
    """
    # vel is in m/s turn radius is in meter
    turn =  vel_turn/turn_radius;
    
    if direction_l:
        lr_amount = math.pi/2 - 10*math.pi/180.
    else:
        lr_amount = -math.pi/2 + 10*math.pi/180.
     
    uturn_sm = smach.StateMachine(outcomes=["succeeded","aborted","preempted"])
    
    with uturn_sm:
        smach.StateMachine.add("DRIVE_FW_IN", 
                               smach_ros.SimpleActionState("/fmExecutors/drive_forward", 
                                                           drive_forwardAction, 
                                                           goal = drive_forwardGoal(amount=(length_in-turn_radius), vel=vel_fw ) ), 
                               transitions={"succeeded":"TURN_1"}, 
                               )
        smach.StateMachine.add("TURN_1",
                               smach_ros.SimpleActionState("/fmExecutors/make_turn",
                                                           make_turnAction,
                                                           goal= make_turnGoal(amount = lr_amount, vel=turn, forward_vel=vel_turn)),
                               transitions={"succeeded":"DRIVE_FW_CROSS"}
                               )
        smach.StateMachine.add("DRIVE_FW_CROSS", 
                               smach_ros.SimpleActionState("/fmExecutors/drive_forward", 
                                                           drive_forwardAction, 
                                                           goal = drive_forwardGoal(amount=(width-turn_radius*2), vel=vel_fw ) ), 
                               transitions={"succeeded":"TURN_2"}, 
                               )
        smach.StateMachine.add("TURN_2",
                               smach_ros.SimpleActionState("/fmExecutors/make_turn",
                                                           make_turnAction,
                                                           goal= make_turnGoal(amount = lr_amount, vel=turn, forward_vel=vel_turn)),
                               transitions={"succeeded":"DRIVE_FW_OUT"}
                               )
        smach.StateMachine.add("DRIVE_FW_OUT",
                               smach_ros.SimpleActionState("/fmExecutors/drive_forward", 
                                                           drive_forwardAction, 
                                                           goal = drive_forwardGoal(amount=(length_out-turn_radius), vel=vel_fw ) ), 
                               transitions={"succeeded":"succeeded"}, 
                               )
    
    return uturn_sm

