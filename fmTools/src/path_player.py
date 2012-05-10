#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmTools")
import rospy
import actionlib
from nav_msgs.msg import Odometry
from fmExecutors.msg import *
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path



filename = "/home/morl/work/ASuBot/waypoints.txt"

def on_timer(event):

    ac = actionlib.SimpleActionClient("/follow_path",follow_pathAction)

        
    rospy.loginfo("opening file")
    # transmit path from file
    with open(filename) as infile:
        infile.readline()
        path = Path()
        pose = PoseStamped()
        rospy.loginfo("processing lines in file")
        for line in infile:

            (x,y) = line.split()
           
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            
            path.poses.append(pose)
            
        rospy.loginfo("waiting for server")
        
        ac.wait_for_server(rospy.Duration(2));
            
        rospy.loginfo("sending path")
        
        goal = follow_pathGoal()
        
        goal.path = path
        
        ac.send_goal_and_wait(goal)
                
    rospy.loginfo("path_player is done");
            
if __name__ == "__main__":

    rospy.init_node("path_player_node")
    
    on_timer(1)

    #rospy.get_param("~path_filename",filename,"waypoints.txt")
    
    
    