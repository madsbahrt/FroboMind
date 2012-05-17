import rospy

import actionlib
import math
from tf import TransformListener,LookupException,ConnectivityException

from fmDecisionMakers.msg import *
from fmExecutors.msg import *

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from smach import State

class DriveRel(State):
    """
    Behaviour to execute a series of relative poses 
    (relative is based from the pose at which this action was first requested)
    """
    
    
    def __init__(self,name,odom_frame,cmd_vel_producer,velocity):
        """
            initialises the behaviour.
            starts the action server
            connects to the plan action
        """
        State.__init__(self, outcomes=["succeeded","aborted","preempted"],input_keys=['goal'])
        
        self.__listen = TransformListener()
        self.__path_msg = Path()
        self.r = rospy.Rate(30)
        self.odom_frame = odom_frame
        self.desired_vel = velocity
        self.cmd_vel_node = cmd_vel_producer
        
        self._action_client = actionlib.SimpleActionClient("/fmExecutors/follow_path",follow_pathAction)
                
    def execute(self,userdata):
        """
            When this action is requested the received relative poses are transform immediately to the odometry frame
            and the sent to the followplan action, if the followplan action fails (abort,preempt) then we abort
        """
        #Reset signals
        done = False
        self.__path_msg.poses = []
        self.goal = userdata.goal
        outcome = "aborted"
        
        rospy.set_param(self.cmd_vel_node, self.desired_vel)
        
        if not self._action_client.wait_for_server(rospy.Duration(0)):
            done = True
            outcome = "aborted"
        else:
            if self.__calculate_new_plan():
                goal = follow_pathGoal()
                goal.path = self.__path_msg
                self._action_client.send_goal(goal)
            else:
                done = True
                outcome = "aborted"


        while not done:
            if self.preempt_requested():
                self._action_client.cancel_goal()
                done = True;
                outcome = "preempted"
                
            else:
                # monitor action client
                status = self._action_client.get_state() 
                if status in [actionlib.GoalStatus.ABORTED,actionlib.GoalStatus.LOST,actionlib.GoalStatus.PREEMPTED]:
                    outcome = "aborted"
                    done = True
                elif status == actionlib.GoalStatus.SUCCEEDED:
                    done = True
                    outcome = "succeeded"
                else:
                    self.r.sleep()
            
        return outcome
        
            
    
    def __calculate_new_plan(self):
        """
            Each of the relative poses in the goal path is transformed into a set of poses in the odometry frame
            and then sent to the action server follow_path
        """
        
        ret = False
        
        try:
            for g_rel in self.goal.poses:
                p = self.__listen.transformPose(self.odom_frame, g_rel)
                self.__path_msg.poses.append(p)
            
            ret= True
            
        except (LookupException, ConnectivityException),err:
              rospy.logerr("Could not transform relative pose into odometry frame")
              rospy.logerr(str(err))
              
        return ret
        
    