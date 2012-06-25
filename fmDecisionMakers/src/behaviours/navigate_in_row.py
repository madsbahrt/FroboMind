#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

import actionlib
import math
import tf
from tf import TransformListener, TransformBroadcaster

from fmDecisionMakers.msg import *
from fmExecutors.msg import *

from fmMsgs.msg import row, steering_angle_cmd

from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path


class NavigateInRowSimple():
    """
    Behaviour to navigate in a row given an estimate of the offset 
    and heading of the row from a sensor.
    This Behaviour does not require that the robot knows where it is, i.e the planner is not used 
    to drive in the row, instead a rabbit is produced locally to the robot.
    """
    __feedback_msg = navigate_in_rowFeedback()
    
    def __init__(self,name,rowtopic,plan_ahead,odom_frame):
        """
            initialises the behaviour.
            starts the action server, connects to the row_topic
        """
        self.desired_offset = 1.5
        
        self.planAhead = plan_ahead
        self.odom_frame = odom_frame
        
        self.imax = 2
        self.i = 0.0
        self.igain = 0.0
        self.pgain = 0.0
        self.dgain = 0.0
        self.error = 0.0
        self.angle_contribution = 0.0
        self.heading_error = 0.0
        self.prev_error = 0.0
        self.outofheadlandcount = 0
        self.max_out_of_headland_count = 0
        
        self.__listen = TransformListener()
        self.__broadcast = TransformBroadcaster()
        self.cur_row = None
        
        self.steering_pub = rospy.Publisher("/fmKinematics/steering_angle_cmd",steering_angle_cmd)
        
        self._action_name = name
        self._server = actionlib.SimpleActionServer(self._action_name,navigate_in_rowAction,auto_start=False)
        self._server.register_goal_callback(self.goal_cb)
        self._server.register_preempt_callback(self.preempt_cb);
        
        self._subscriber = rospy.Subscriber(rowtopic, row, callback=self.row_msg_callback)
        self._last_row_msg = None
        self.last_time = rospy.Time.now()
        self._server.start()
        
    def preempt_cb(self):
        """
            Called when the action client whishes to preempt us.
        """
        rospy.loginfo("preempt received")
        self._server.set_preempted()
        
    
    def goal_cb(self):
        """
            Called when we receive a new goal.
            We could inspect the goal before accepting it,
            however for start we only accept it.
        """
        rospy.loginfo("goal received")
        self._last_row_msg = None
        self.last_time = rospy.Time.now()
        
        g = self._server.accept_new_goal()
        
        self.desired_offset = g.desired_offset_from_row
        self.max_out_of_headland_count = g.headland_timeout
        
        self.pgain = g.P
        self.igain = g.I
        self.dgain = g.D
        self.imax = g.I_max
        self.angle_contribution = g.angle_contribution
        
        
        rospy.loginfo("running navigate in row with as: %f ds: %f desired_offset: %f" % (self.anglescale,self.distancescale,self.desired_offset))
        
    def row_msg_callback(self,msg):
        """
            This function is called every time a new row message is
            received. 
        """
        self.cur_row = msg
        if  (rospy.Time.now() - self.last_time) > rospy.Duration(0.05):
            self.on_timer(None)  
            self.last_time = rospy.Time.now()
            
    def on_timer(self,e):
        if self._server.is_active():
            if self.cur_row:
                if self.outofheadlandcount > self.max_out_of_headland_count:
                    self.__perform_turn()
                    rospy.loginfo("turning due to headland...")
                elif self.cur_row.headland:
                    self.outofheadlandcount += 1
                    rospy.loginfo("Row out of sight %d" % (self.outofheadlandcount))
                
                if self.cur_row.headland == False:
                    self.outofheadlandcount = 0
                        
                if self.cur_row.rightvalid and self.outofheadlandcount < self.max_out_of_headland_count:
                    self.__calculate_steering_angle(self.cur_row)


                        
            
            
    def __perform_turn(self):
        
        rospy.loginfo("performing turn")
        # turn right
        steering_angle = steering_angle_cmd()
        steering_angle.steering_angle = -0.7;
        steering_angle.header.stamp = rospy.Time.now()
        
        self.steering_pub.publish(steering_angle)
        
    
    def __calculate_steering_angle(self,row):
        """
            Tries to regulate the vehicle so it follow the row.
        """
        self.heading_error =  -( row.rightdistance - self.desired_offset)
        self.error = row.rightangle
        
        self.i += self.error
        
        if self.i > self.imax:
            self.i = self.imax
        elif self.i < -self.imax:
            self.i = -self.imax
        
        steering_angle = steering_angle_cmd()
        steering_angle.steering_angle = self.error*self.pgain + (self.error - self.prev_error)*self.dgain + self.igain*self.i + self.angle_contribution * self.heading_error
        steering_angle.header.stamp = rospy.Time.now()    
        
        self.steering_pub.publish(steering_angle)
        
        self.__feedback_msg.error = self.error
        self.__feedback_msg.integral = self.i
        self.__feedback_msg.differential = (self.error - self.prev_error)
        self._server.publish_feedback(self.__feedback_msg)
        
        self.prev_error = self.error

if __name__ == "__main__":
    
    rospy.init_node("navigate_in_row")
    
    NavigateInRowSimple(rospy.get_name(),"/rows",3,"odom_combined")
    
    rospy.spin()
    