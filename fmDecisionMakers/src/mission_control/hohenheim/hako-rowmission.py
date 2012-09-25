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
from fmExecutors.msg import follow_pathGoal, follow_pathAction
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# messages used 
from sensor_msgs.msg import Joy
    
def force_preempt(a):
    return True

def generate_local_path():
    
    p = follow_pathGoal()
    
    ps = PoseStamped()
    
    ps.pose.position.x = 5
    ps.pose.position.y = 1
    ps.pose.orientation.x = 1
    ps.header.frame_id="base_footprint"
    ps.header.stamp = rospy.Time.now()
    
    p.path.poses.append(ps)
    
    ps = PoseStamped()
    
    ps.pose.position.x = 10
    ps.pose.position.y = 10
    ps.pose.orientation.x = 1
    ps.header.frame_id="base_footprint"
    ps.header.stamp = rospy.Time.now()
    
    p.path.poses.append(ps)
    
    ps = PoseStamped()
    
    ps.pose.position.x = 30
    ps.pose.position.y = 10
    ps.pose.orientation.x = 1
    ps.header.frame_id="base_footprint"
    ps.header.stamp = rospy.Time.now()
    
    p.path.poses.append(ps)
    
    ps = PoseStamped()
    
    ps.pose.position.x = 30
    ps.pose.position.y = 30
    ps.pose.orientation.x = 1
    ps.header.frame_id="base_footprint"
    ps.header.stamp = rospy.Time.now()
    
    p.path.poses.append(ps)
    
    return p


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
    
    ##row_nav = build_row_nav_sm(row_goal,2)

    local_path = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    local_path_goal = generate_local_path()
    with local_path:
        smach.StateMachine.add("FOLLOW_PATH_LOCAL",
                               smach_ros.SimpleActionState("/fmExecutors/follow_path",follow_pathAction,local_path_goal),
                               transitions={"succeeded":"FOLLOW_PATH_LOCAL"})

    #return behaviours.wii_states.wii_auto_manuel.create(local_path, "/fmHMI/joy", 3)
    return local_path
    
    
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