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


class NavigateInRow():
    """
    Behaviour to navigate in a row given an estimate of the offset 
    and heading of the row from a sensor.
    This behviour generates a plan and sends it to the rabbitplanner 
    as two waypoints which corresponds to a straight line parallel 
    to the detected row at the specified offset.
    """
    __feedback_msg = navigate_in_rowFeedback()
    __path_msg = Path()
    
    def __init__(self,name,rowtopic,replan_heading_threshold,replan_offset_threshold,desired_offset,plan_ahead):
        """
            initialises the behaviour.
            starts the action server, connects to the row_topic
            connects to the plan service
        """
        self.replan_heading_threshold = replan_heading_threshold
        self.replan_offset_threshold = replan_offset_threshold
        self.desired_offset = desired_offset
        self.planAhead = plan_ahead
        
        self.__listen = TransformListener()
        
        self._action_name = name
        self._server = actionlib.SimpleActionServer(self._action_name,navigate_in_rowAction,auto_start=False)
        self._server.register_goal_callback(self.goal_cb)
        self._server.register_preempt_callback(self.preempt_cb);
        
        self._subscriber = rospy.Subscriber(rowtopic, claas_row_cam, callback=NavigateInRow.row_msg_callback,callback_args=self)
        self._action_client = actionlib.SimpleActionClient("/rabbitplanner/follow_path",follow_pathAction)
        self._last_row_msg = None
        self._server.start()
        
    def preempt_cb(self):
        rospy.loginfo("preempt received")
        self._server.set_preempted()
        pass
    
    def goal_cb(self):
        #Reset signals
        rospy.loginfo("goal received")
        self._last_row_msg = None
        
        self._server.accept_new_goal()
        
    def row_msg_callback(self,msg):
        """
            new row callback function
        """
        if self._server.is_active():
            if msg.quality > 150:
                if self._last_row_msg:
                    if self.__should_replan(msg, self._last_row_msg):
                        self.__calculate_new_plan(msg)
                        self._last_row_msg = msg
                else:
                    self.__calculate_new_plan(msg)
                    self._last_row_msg = msg
                self.__feedback_msg.error = msg.quality
                self._server.publish_feedback(self.__feedback_msg)
            else:
                # we have lost track
                self._server.set_aborted(None, "quality too low")
            
        
    def __should_replan(self,row,last_row):
        replan = False
        # calculate the change from last row to current row
        offset_change = row.offset - last_row.offset 
        heading_change = row.heading - last_row.heading
        
        if offset_change > self.replan_offset_threshold:
            replan = True
        if heading_change > self.replan_heading_threshold:
            replan = True
        
        # if the current goal has been reached we should plan a new one
        # furthermore if it has been aborted lost or preempted
        if self._action_client.get_result() in [actionlib.GoalStatus.SUCCEEDED,
                                                actionlib.GoalStatus.ABORTED,
                                                actionlib.GoalStatus.LOST,
                                                actionlib.GoalStatus.PREEMPTED]:
            replan = True
            
        return replan
    
    def __calculate_new_plan(self,row):
        """
            The plan is calculated as two poses, one right in front of the vehicle and a second one
            further away, the distance to the second point is a parameter planAhead. 
            The two points are placed so the line formed is parallel to the detected row.
            The distance between the planned line and the row is a desired_offset parameter.
            The calculated points are transformed into the global navigation frame using tf.
            The header.frame_id from the row message is used as source frame, and the target frame
            is determined by parameter odom_frame.
        """
        A = None
        B = None
        
        row_point_start = PoseStamped()
        row_point_end   = PoseStamped()
        
        row_point_start.header.stamp = row_point_end.header.stamp = rospy.Time.now() 
        
        row_point_start.header.frame_id = row.header.frame_id
        row_point_start.pose.position.y = -((row.offset-self.desired_offset)/100.0)
        row_point_start.pose.position.x = 0
        
        row_point_end.header.frame_id = row.header.frame_id
        row_point_end.pose.position.x = self.planAhead
        row_point_end.pose.position.y = self.planAhead * math.tan(row.heading/360.0 * 2* math.pi) - ((row.offset-self.desired_offset)/100.0);
        
        
        try:
            A = self.__listen.transformPose(odom_frame,row_point_start)
            B = self.__listen.transformPose(odom_frame,row_point_end)
            self.__path.poses.clear()
        
            self.__path.poses.append(A)
            self.__path.poses.append(B)
            
            g = follow_pathGoal()
            g.path = self.__path;
            
            self._action_client.send_goal(g)
            
        except (tf.LookupException, tf.ConnectivityException),err:
              rospy.logerr("Could not transform row cam message into odometry frame")
              rospy.logerr(str(err))
              

        

if __name__ == "__main__":
    
    rospy.init_node("navigate_in_row")
    
    NavigateInRow(rospy.get_name(), "/fmSensors/row", 0.1, 0.1, 0.6,3)
    
    rospy.spin()
    