import rospy

import actionlib
from fmExecutors.msg import * 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from smach import State


class PlanFollow(State):
    """
        This state follows a given plan, if the state is 
        preempted the current step in the plan
        is recorded and when the state is activated again 
        it continues from the stored point in the plan.
    """
    def __init__(self,plan,action_name):
        """
            @param plan: the plan to follow 
            @param action_name: the action to call in order to execute the plan
        """
        State.__init__(self, outcomes=["succeeded","aborted","preempted"],output_keys=['currentAB'])
        self.goal = follow_pathGoal()
        self.cur_index = 0
        self.action_name = action_name
        self.goal.path = plan
        self.r = rospy.Rate(10)
        self.done = False;
        self.outcome = "aborted"
        pass


    def execute(self,userdata):
        """
            The path is sent to the action_client 
            and each time a pose in the plan has been executed the pose is removed from the plan
            in order to be able to be preempted and then continue the plan at a later time.
        """
        
        self._action_client = actionlib.SimpleActionClient(self.action_name,follow_pathAction)
        self.done = False
        # first modify list of poses according to cur_index
        self.goal.path.poses = self.goal.path.poses[self.cur_index:]
        # reset cur_index as we are now at step zero in our new plan
        self.cur_index = 0
       
        if not self._action_client.wait_for_server(rospy.Duration(30)):
            self.done = True
            self.outcome = "aborted"
        else:
            self._action_client.send_goal(self.goal,feedback_cb=self.on_feedback_from_action)
            
        while not self.done:
            if self.preempt_requested():
                print "preemption requested"
                # inform action_client by aborting
                self._action_client.cancel_goal()
                self.service_preempt()
                self.outcome = "preempted"
                self.done = True
            else:
                status = self._action_client.get_state() 
                if status in [actionlib.GoalStatus.ABORTED,actionlib.GoalStatus.LOST,actionlib.GoalStatus.PREEMPTED]:
                    self.outcome = "aborted"
                    print status
                    self.done = True
                elif status == actionlib.GoalStatus.SUCCEEDED:
                    self.outcome = "succeeded"
                    print status
                    self.done = True
                else:
                    self.r.sleep()
                    
        
        userdata.currentAB =[ self.goal.path.poses[self.cur_index],self.goal.path.poses[self.cur_index+1] ] 
        return self.outcome
            
        
    def on_feedback_from_action(self,feedback):
        self.cur_index = feedback.reached_pose_nr - 1

    