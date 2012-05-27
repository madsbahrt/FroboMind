import rospy
import numpy as np
import actionlib
from fmExecutors.msg import * 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from smach import State,StateMachine
from tf import TransformListener,LookupException,ConnectivityException




class PointAndShoot(State):
    """
       Will drive \"distance\" meters in the current direction. 
       Adheres to the SAP (succeeded, aborted, preempted) state outcomes.
    """
    def __init__(self,name,distance,odom_frame,vehicle_frame):
        """
            initialises the behaviour.
            starts the action server
            connects to the plan action
        """
        State.__init__(self, outcomes=["succeeded","aborted","preempted"])
        
        self.target_distance = target_distance
        self.__path_msg = Path()
        self.r = rospy.Rate(30)
        self.odom_frame = odom_frame
        self.vehicle_frame = vehicle_frame
        self.desired_dist = distance
        self.__listen = TransformListener()
        
        self._action_client = actionlib.SimpleActionClient("/fmExecutors/follow_path",follow_pathAction)
                
    def execute(self,userdata):
        """
        """
        #Reset signals
        done = False
        self.__path_msg.poses = []
        outcome = "aborted"
        
        if not self._action_client.wait_for_server(rospy.Duration(30)):
            done = True
            outcome = "aborted"
        else:
            if self.__calculate_new_plan_no_AB():
                goal = follow_pathGoal()
                goal.path = self.__path_msg
                self._action_client.send_goal(goal)
            else:
                done = True
                outcome = "aborted"


        while not done:
            if self.preempt_requested():
                self._action_client.cancel_goal()
                self.service_preempt()
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
    
    def __calculate_new_plan_no_AB(self):
        """
         Places a point \"x\" meters in the basefootprint frame and converts it into the odometry frame
         and returns true if the point could be converted
        """
        
        ret = False
        
        try:
            trans,rot = self.__listen.lookupTransform(self.odom_frame,self.vehicle_frame,rospy.Time(0))
            
            Ap = PoseStamped()
            Ap.pose.position.x = trans[0]
            Ap.pose.position.y = trans[1]
           
            
            poseB = PoseStamped()
            poseB.header.frame_id = self.vehicle_frame
            poseB.pose.position.x = self.target_distance
            poseB.pose.position.y = 0
            Bp = self.__listen.transformPose(self.odom_frame, poseB)
            
            self.__path_msg.poses.append(Ap)
            self.__path_msg.poses.append(Bp)
            ret = True
            print "PoseA x: %f y: %f" % (Ap.pose.position.x,Ap.pose.position.y)
            print "PoseB x: %f y: %f" % (Bp.pose.position.x,Bp.pose.position.y)
            
        except (LookupException, ConnectivityException),err:
              rospy.logerr("Could not transform relative pose into odometry frame")
              rospy.logerr(str(err))
        
        return ret