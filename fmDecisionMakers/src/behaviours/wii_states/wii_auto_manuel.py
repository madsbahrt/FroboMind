import rospy
import smach
import smach_ros
import behaviours
from fmTools.srv import switch_muxRequest,switch_muxResponse,switch_mux

from joy.msg import Joy


def should_preempt(arg):
    return True

def btn_pressed(ud,msg):
    return not msg.buttons[ud.btn_id]

def create(auto_state,joy_topic,btn_index):
    """
    Creates a master state by wrapping the auto state with a manuel mode, 
    where the mode can be changed by pressing the selected wii btn.
    If the auto state succeeds the state returns to manuel mode.
    """    
    manuel_req = switch_muxRequest();
    manuel_req.mode = switch_muxRequest.MANUEL
    
    auto_req = switch_muxRequest();
    auto_req.mode = switch_muxRequest.AUTO
        
    auto_wrapper =  smach.Concurrence(outcomes=['succeeded','aborted','preempted'], 
                           default_outcome='aborted',
                           outcome_map={"preempted":{'MONITOR_BTN':'invalid'}, 
                                        "succeeded":{'AUTO_STATE':'succeeded'}},
                           child_termination_cb=should_preempt)
    
    auto_wrapper.userdata.btn_id = btn_index;
    
    with auto_wrapper:
        smach.Concurrence.add("MONITOR_BTN",smach_ros.MonitorState(joy_topic, Joy, btn_pressed, max_checks=-1),remapping={"btn_id":"btn_id"})
        smach.Concurrence.add("AUTO_STATE",auto_state)
    
    
    master = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    master.userdata.btn_id = btn_index;
    with master:
        smach.StateMachine.add("SET_MANUEL_MODE", 
                               smach_ros.ServiceState("/fmTools/cmd_vel_mux/cmd_vel_mux", switch_mux, request=manuel_req), 
                               transitions={'succeeded':'WAIT'} 
                               )
        smach.StateMachine.add("WAIT",
                               behaviours.wait_state.WaitState(rospy.Duration(0.5)),
                               transitions={"succeeded":"MONITOR_BTN"})
        smach.StateMachine.add("MONITOR_BTN",
                               smach_ros.MonitorState(joy_topic, Joy, btn_pressed, max_checks=-1),
                               transitions={'invalid':'SET_AUTO_MODE','valid':'aborted'},
                               remapping={"btn_id":"btn_id"}
                               )
        smach.StateMachine.add("SET_AUTO_MODE", 
                               smach_ros.ServiceState("/fmTools/cmd_vel_mux/cmd_vel_mux", switch_mux, request=auto_req), 
                               transitions={'succeeded':'AUTO_MODE','aborted':'SET_MANUEL_MODE'}
                               ) 
        smach.StateMachine.add("AUTO_MODE",
                               auto_wrapper,
                               transitions={'succeeded':'SET_MANUEL_MODE','preempted':'SET_MANUEL_MODE'},
                               remapping={"btn_id":"btn_id"}
                               )
    

    return master