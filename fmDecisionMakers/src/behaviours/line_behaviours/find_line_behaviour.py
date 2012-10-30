#!/usr/bin/env python
import roslib; 
roslib.load_manifest('fmDecisionMakers')
import rospy
import smach
import smach_ros
import actionlib
import math
import tf
from fmExecutors.msg import timed_turnAction,timed_turnGoal 
from fmMsgs.msg import adc
    

def force_preempt(a):
    return True

def line_found(ud,msg):
    if msg.value[0] > ud.threshold:
        return False
    elif msg.value[1] > ud.threshold:
        return False
    else:       
        return True
        

def build_wiggle_sm(threshold, width):
    wiggle_sm = smach.StateMachine(outcomes = ['succeeded','aborted','preempted'])
    with wiggle_sm:
        smach.StateMachine.add('WIGGLE_LEFT',
                               smach_ros.SimpleActionState('/fmExecutors/timed_turn',timed_turnAction,goal=timed_turnGoal(time=2.0, vel=0.1)),
                               transitions = {'succeeded':'WIGGLE_RIGHT','aborted':'aborted','preempted':'preempted'}
                               )
        smach.StateMachine.add('WIGGLE_RIGHT',
                               smach_ros.SimpleActionState('/fmExecutors/timed_turn',timed_turnAction,goal=timed_turnGoal(time=2.0, vel=-0.1)),
                               transitions = {'succeeded':'WIGGLE_LEFT','aborted':'aborted','preempted':'preempted'}
                               )
    return wiggle_sm
        
def build_find_line_behaviour(threshold,vel):
    find_line_sm = smach.Concurrence(outcomes=['succeeded','preempted','aborted'],
                                     default_outcome = 'aborted',
                                     outcome_map = {
                                                    'succeeded':{'WIGGLE':'preempted','FIND_LINE':'invalid'},
                                                    'preempted':{'WIGGLE':'preempted','FIND_LINE':'preempted'}
                                                    },
                                     child_termination_cb=force_preempt)
    find_line_sm.userdata.threshold = threshold
    with find_line_sm:
        smach.Concurrence.add('WIGGLE',build_wiggle_sm(threshold,vel))
        smach.Concurrence.add('FIND_LINE',smach_ros.MonitorState("/fmSensors/adc",adc,line_found,-1),
                              remapping = {'threshold':'threshold'})
        
    return find_line_sm

