#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

import actionlib
import math
from tf import TransformListener

from fmDecisionMakers.msg import *
from fmExecutors.msg import * 
from fmMsgs.msg import claas_row_cam

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class FindRow():
    """
        Currently we have no idea how to find a row 
    """
    
    __path = Path()
    
    
    def __init__(self,name,odom_frame):
        
        self._action_name = name
        self._server = actionlib.SimpleActionServer(self._action_name,find_rowAction,execute_cb=self.execute_cb)
        self._action_client = actionlib.SimpleActionClient("/rabbitplanner/follow_path",follow_pathAction)
        self._online = False
        self.__listen = TransformListener()
        self._cur_msg  =None
        
        self._server.start()
    
    def execute_cb(self,goal):
        #
        found = False
        while(not found):
            if self._server.is_preempt_requested():
                self._server.set_preempted(None, "premepted was requested")
                break
            else:
                if self._cur_msg is not None and ((self._cur_msg.header.stamp - rospy.Time.now()) < 1):
                    if self._cur_msg.quality > 200:
                        found = True
        
        if found:
            self._server.set_succeeded(None, "crap")
        pass
    
    def on_row(self,msg):
        if self._online:
            # generate a short plan and follow it
            # in this case we drive forward for 2m
            if msg.quality > 200:
                self._server.set_succeeded(1, "row found")
                self._online = False
        
    def __generate_plan(self):
        self.__path.poses.clear()

        try:
            odom_pose = self.__listen.transformPose(self.odom_frame, pose);
            pose = PoseStamped()
        
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            self.__path.poses.append(odom_pose)
            pose.pose.position.x = 1;
            pose.pose.position.y = 0;
            self.__path.poses.append(odom_pose)
            
            goal = follow_pathGoal()
            goal.path = self.__path
            self._action_client.wait_for_server(rospy.Duration(1))
            self._action_client.send_goal(goal)
            
        except (tf.LookupException, tf.ConnectivityException),err:
            rospy.logerr("Could not transform from local frame to navigation frame")
            rospy.logerr(str(err))
             
        

if __name__ == "__main__":
    rospy.init_node("find_rows")
    
    FindRow("find_row","odom_combined")
    
    rospy.spin()
    
    