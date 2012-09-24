import rospy
import numpy as np
import actionlib
from fmExecutors.msg import * 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from smach import State,StateMachine
from tf import TransformListener,LookupException,ConnectivityException




class StopAndGO(State):
    """
        Given an AB line it will drive \'distance\' meters in the direction of the line A->B
        Can be used in situations were a task is needed to be performed every X meters on A->B
        So instead of manually breaking up the large line this state can be used. 
        Requires the use of the AB lines executor and a valid AB line given as userdata.
    """
    def __init__(self,name,distance,odom_frame,vehicle_frame):
        """
            initialises the behaviour.
            starts the action server
            connects to the plan action
        """
        State.__init__(self, outcomes=["succeeded","aborted","preempted"],input_keys=['AB_path'])
        
        
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
        self.goal = userdata.AB_path
        outcome = "aborted"
        
        if not self._action_client.wait_for_server(rospy.Duration(30)):
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
    
    def __calculate_new_plan(self):
        """
        """
        
        ret = False
        
        try:
            
            trans,rot = self.__listen.lookupTransform(self.odom_frame,self.vehicle_frame,rospy.Time(0))
            A = np.array([self.goal[0].pose.position.x,self.goal[0].pose.position.y])
            B = np.array([self.goal[1].pose.position.x,self.goal[1].pose.position.y])
            base = np.array([trans[0],trans[1]])
            # project our position onto the AB line
            AB = B - A
            ABSquared = AB[1]*AB[1] + AB[0]*AB[0]
            
            if not ABSquared == 0:
                Abase = B - base
                
                distance = (Abase[0]*AB[0]+Abase[1]*AB[1])/ABSquared

                local_A = B - (distance * AB)
                local_B = local_A + self.desired_dist*AB/np.sqrt(ABSquared)
                
                Ap = PoseStamped()
                Bp = PoseStamped()
                
                Ap.pose.position.x = local_A[0]
                Ap.pose.position.y = local_A[1]
                
                Bp.pose.position.x = local_B[0]
                Bp.pose.position.y = local_B[1]
                
                self.__path_msg.poses.append(Ap)
                self.__path_msg.poses.append(Bp)
                
                ret = True
                
        except (LookupException, ConnectivityException),err:
              rospy.logerr("Could not transform relative pose into odometry frame")
              rospy.logerr(str(err))
              
        return ret