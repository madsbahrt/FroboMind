#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

import actionlib
import math
import tf

from tf import TransformListener, TransformBroadcaster
from geometry_msgs.msg import PointStamped

from fmDecisionMakers.msg import *
from fmExecutors.msg import *



class TurnAction():
    """
        Performs a 180 turn either to the left or right 
        depending on the given goal.
    """
    def __init__(self,name,odom_frame,base_frame):
        """
        
        @param name: the name of the action
        """
        self._action_name = name
        self.__odom_frame = odom_frame
        self.__base_frame = base_frame
        self.__server =  actionlib.SimpleActionServer(self._action_name,make_turnAction,auto_start=False)
        self.__server.register_preempt_callback(self.preempt_cb)
        self.__server.register_goal_callback(self.goal_cb)
        
        self.__listen = TransformListener()
        self.__broadcast = TransformBroadcaster()
        
        self.__turn_timeout = 50
        
        self.__desired_radius = 3
        self.__success_radius = 0.1
        self.__turn_left = True
        self.new_goal = False
        
        self.__server.start()

    def preempt_cb(self):
        rospy.loginfo("Preempt requested")
        self.__server.set_preempted()
    
    def goal_cb(self):
        
        g = self.__server.accept_new_goal()
        self.__desired_radius= g.desired_radius
        self.__success_radius = g.success_radius
        self.__turn_left = bool(g.left)
        self.new_goal = True
    
    def on_timer(self,e):
        
        if self.__server.is_active():
            if self.new_goal:
                self.new_goal = False
                if self.__get_current_position():
                    self.__start_time = rospy.Time.now()
                else:
                    self.__server.set_aborted(text="could not find vehicle")
                if self.__place_rabbit():
                    self.__broadcast_rabbit()
                else:
                    self.__server.set_aborted(text="could not find vehicle")
            else:
                if rospy.Time.now() - self.__start_time > rospy.Duration(self.__turn_timeout):
                    self.__server.set_aborted(text="timeout on action")
                if not self.__broadcast_rabbit():
                    self.__server.set_aborted(text="could not broadcast rabbit")
                if self.__get_distance() < self.__success_radius:
                    self.__server.set_succeeded()
        pass
    
    def __get_current_position(self):
        ret = False
        try:
            self.__start_pos = self.__listen.lookupTransform(self.__odom_frame,self.__base_frame,rospy.Time(0))
            ret = True
        except (tf.LookupException, tf.ConnectivityException),err:
            rospy.loginfo("could not locate vehicle")
        
        return ret
    
    def __place_rabbit(self):
        """
            place the rabbit to either the right or left of a circle with desired radius.
        """
        ret = False
        if self.__turn_left:
            p = PointStamped()
            p.point.x =0
            p.point.y = self.__desired_radius
            p.header.frame_id = self.__base_frame
        else:
            p = PointStamped()
            p.header.frame_id = self.__base_frame
            p.point.x = 0
            p.point.y = - self.__desired_radius
            
        try:
            self.__rabbit = self.__listen.transformPoint(self.__odom_frame, p)
            ret = True
        except (tf.LookupException,tf.ConnectivityException),err:
            rospy.loginfo("could not locate vehicle")
        
        return ret
    
    def __broadcast_rabbit(self):
        ret = False
        try:
            self.__broadcast.sendTransform([self.__rabbit.point.x, self.__rabbit.point.y,0], [0, 0, 0, 1], rospy.Time.now(), "rabbit", self.__odom_frame)
            ret = True
        except (tf.LookupException,tf.ConnectivityException),err:
            rospy.loginfo("could not locate vehicle")
            
        return ret
    
    def __get_distance(self):
        dist = 0
        try:
            loc = self.__listen.lookupTransform("rabbit",self.__base_frame,rospy.Time(0))
            dist = math.sqrt(loc[0][0] * loc[0][0] + loc[0][1]*loc[0][1])
        except (tf.LookupException,tf.ConnectivityException),err:
            rospy.loginfo("could not calculate distance")
        
        return dist
            

if __name__ == "__main__":
    
    rospy.init_node("turn_action")
    
    action_server = TurnAction("perform_turn","odom","base_link")
    
    t = rospy.Timer(rospy.Duration(0.05),action_server.on_timer)
    
    rospy.spin()
    