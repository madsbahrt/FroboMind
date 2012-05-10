#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

import actionlib
import tf

import smach
import smach_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# behaviours used in this statemachine
from fmDecisionMakers.msg import navigate_in_rowAction
from fmDecisionMakers.msg import find_rowAction
from fmExecutors.msg import follow_pathAction, follow_pathGoal

@smach.cb_interface(outcomes=['succeeded'])
def notify(arg):
    print "Something went wrong"
    return "succeeded"


def load_path(filename):
    path = Path()
    
    with open(filename) as file:
        file.readline()
            
        for line in file:
            p = PoseStamped()
            x,y=line.split()
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            path.poses.append(p)
    
    for p in path.poses:
        print p
    
    return path



if __name__ == "__main__":
    
    rospy.init_node("MissionMaster")
    

    infile = rospy.get_param("~field_path_filename", "path.txt")
    outfile = infile
    
    drive_out_goal = follow_pathGoal(path=load_path(infile))
    
    
    p_home = load_path(outfile)
    
    p_home.poses.reverse()
    
    drive_home_goal = follow_pathGoal(path=p_home)
   
    
    
    sm = smach.StateMachine(['succeeded','aborted','preempted'])
    with sm:
        smach.StateMachine.add('FIND_ROW',
                         smach_ros.SimpleActionState("find_row", find_rowAction),
                         transitions={'succeeded':'NAVIGATE_IN_ROW','aborted':'NOTIFY_ABORT','preempted':'NOTIFY_ABORT'})
        
        smach.StateMachine.add('NAVIGATE_IN_ROW',
                         smach_ros.SimpleActionState("navigate_in_row", navigate_in_rowAction,server_wait_timeout=1),
                         transitions={'aborted':'FIND_ROW'})
        smach.StateMachine.add('NOTIFY_ABORT',
                               smach.CBState(notify),
                               transitions={'succeeded':'aborted'}
                               )
        
    
    to_from_field = smach.Sequence(['succeeded','aborted','preempted'],connector_outcome='succeeded')
    
    with to_from_field:
        smach.Sequence.add('DRIVE_TO_FIELD', smach_ros.SimpleActionState("/fmExecutors/follow_path",follow_pathAction,goal=drive_out_goal))
        #smach.Sequence.add('PERFORM_FIELD_TASK', sm)
        smach.Sequence.add('DRIVE_FROM_FIELD',smach_ros.SimpleActionState("/fmExecutors/follow_path",follow_pathAction,goal=drive_home_goal))
    
    
    intro_server = smach_ros.IntrospectionServer('field_mission',to_from_field,'/FIELDMISSION')
    intro_server.start()    
    
    outcome =  to_from_field.execute()
    print outcome
    
    rospy.signal_shutdown('All done.')

