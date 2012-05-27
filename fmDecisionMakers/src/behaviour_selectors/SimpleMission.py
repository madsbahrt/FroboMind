#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

import behaviours
import actionlib
import tf

#from sensor_msgs.msg import Joy
from joy.msg import Joy

import smach
import smach_ros

# behaviours used in this statemachine
from fmExecutors.msg import follow_pathAction, follow_pathGoal

from fmTools.srv import switch_muxRequest,switch_muxResponse,switch_mux


def btn_1_pressed(ud,msg):
    if msg.buttons[0]:
        return False
    else:
        return True 

def btn_2_pressed(ud,msg):
    if msg.buttons[1]:
        return False
    else:
        return True 
    
def btn_3_pressed(ud,msg):
    if msg.buttons[2]:
        return False
    else:
        return True 
    
def btn_5_pressed(ud,msg):
    if msg.buttons[4]:
        return False
    else:
        return True 

def set_speed(userdata,speedtoggle):
    rospy.set_param("/fmControllers/rabbit_follower/max_linear_vel", speedtoggle.toggle_speed())
    return "done"

def stop_behaviour(keys):
    return False

def should_preempt(a):
    print "Forcing shutdown of other childs in concurrence"
    return True

class SpeedToggle():
    def __init__(self,speed1,speed2):
        self.speed1 = speed1
        self.speed2 = speed2
        self.selected = 0
    
    def toggle_speed(self):
        if self.selected == 1:
            self.selected = 0
            return self.speed1
        else:
            self.selected = 1
            return self.speed2
        

if __name__ == "__main__":
    
    rospy.init_node("MissionMaster")
    #
    # Manuel statemachine
    #
    manuel_mode = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
    
    manuel_req = switch_muxRequest();
    manuel_req.mode = switch_muxRequest.MANUEL
    
    auto_req = switch_muxRequest();
    auto_req.mode = switch_muxRequest.AUTO
    
    with manuel_mode:
        smach.StateMachine.add("SET_MANUEL_MODE", 
                               smach_ros.ServiceState("/fmTools/cmd_vel_mux/cmd_vel_mux", switch_mux, request=manuel_req), 
                               transitions={'succeeded':'MONITOR_BTN_A'} 
                               )
        smach.StateMachine.add("MONITOR_BTN_A",
                               smach_ros.MonitorState("/fmHMI/joy", Joy, btn_3_pressed, max_checks=-1),
                               transitions={'invalid':'SET_AUTO_MODE','valid':'aborted'}
                               )
        smach.StateMachine.add("SET_AUTO_MODE", 
                               smach_ros.ServiceState("/fmTools/cmd_vel_mux/cmd_vel_mux", switch_mux, request=auto_req), 
                               transitions={'succeeded':'WAIT'}
                               ) 
        smach.StateMachine.add("WAIT",
                               behaviours.WaitState(0.5),transitions={'succeeded':'preempted'})
    
    #
    # Drive slow mode
    #
    speed_selector = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    speed_toggle = SpeedToggle(0.5,1.0)
    with speed_selector:
        smach.StateMachine.add("CHANGE_REQUESTED", 
                               smach_ros.MonitorState("/fmHMI/joy", Joy, btn_5_pressed, max_checks=-1),
                               transitions={'invalid':'TOGGLE_SPEED','valid':'aborted'}
                               )
        smach.StateMachine.add("TOGGLE_SPEED",smach.CBState(set_speed, cb_args=[speed_toggle], outcomes=['done']),
                               transitions={'done':'WAIT'})
        smach.StateMachine.add("WAIT",behaviours.wait_state.WaitState(rospy.Duration(0.5)),
                               transitions={'succeeded':'CHANGE_REQUESTED'})
    
    #
    # STOP and GO statemachine
    #
    point_and_shoot = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    with point_and_shoot:
        smach.StateMachine.add("DRIVE_FORWARD", 
                               behaviours.point_and_shoot.PointAndShoot("/fmExecutors/follow_path",10,"odom_combined","base_footprint"),
                               )

    auto_mode_2 = smach.Concurrence(outcomes=['succeeded','aborted','preempted'], 
                           default_outcome='aborted',
                           outcome_map={"preempted":{'MONITOR_BTN_A':'invalid'}, 
                                        "succeeded":{'AUTO_MODE':'succeeded'}},
                           child_termination_cb=should_preempt)

    with auto_mode_2:
        smach.Concurrence.add("MONITOR_BTN_A",smach_ros.MonitorState("/fmHMI/joy", Joy, btn_3_pressed, max_checks=-1))
        smach.Concurrence.add("AUTO_MODE",point_and_shoot)
        smach.Concurrence.add("SPEED_SELECTOR",speed_selector)
        
    master = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    with master:
        smach.StateMachine.add("MANUEL_MODE",
                               manuel_mode,
                               transitions={'preempted':'AUTO_MODE'})
        smach.StateMachine.add("AUTO_MODE",
                               auto_mode_2,
                               transitions={'succeeded':'MANUEL_MODE','preempted':'MANUEL_MODE'})


    intro_server = smach_ros.IntrospectionServer('field_mission',master,'/FIELDMISSION')
    intro_server.start()    
    
    outcome =  master.execute()
    
    rospy.signal_shutdown('All done.')

