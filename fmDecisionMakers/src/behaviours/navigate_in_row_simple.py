#!/usr/bin/env python
import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

import actionlib
import math
from tf import TransformListener, LookupException, ConnectivityException

from fmDecisionMakers.msg import navigate_in_row_simpleAction, navigate_in_row_simpleFeedback,navigate_in_row_simpleGoal,navigate_in_row_simpleResult

from fmMsgs.msg import row

from geometry_msgs.msg import Twist


class NavigateInRowSimple():
    """
    Behaviour to navigate in a row given an fmMsgs/row message
    
    The row is navigated based on the desired offset from row,
    which is interpreted as the desired offset from the center of the row.
    """
    __feedback_msg = navigate_in_row_simpleFeedback()
    __goal_msg = navigate_in_row_simpleResult()
    
    def __init__(self,name,rowtopic,odom_frame,vehicle_frame):
        """
            initialises the behaviour.
            starts the action server, connects to the row_topic
        """
        
        ## Setup some default values even though they are received via the goal
        self.desired_offset = 0.3
        self.max_out_of_headland_count = 10
        self.pgain = 0.2
        self.distance_scale = 0.1
        self.forward_velocity= 0.5
        
        
        # store the odometry frame id for use in distance calculation
        self.odom_frame = odom_frame
        self.vehicle_frame = vehicle_frame
        
        # reset counters used for detecting loss of rows and headland
        self.outofheadlandcount = 0
        self.left_valid_count = 0
        self.right_valid_count = 0
    
        self.__listen = TransformListener()
 
        self.cur_row = None
        
        self.vel_pub = rospy.Publisher("/fmControllers/cmd_vel_auto",Twist)
        
        self._action_name = name
        self._server = actionlib.SimpleActionServer(self._action_name,navigate_in_row_simpleAction,auto_start=False)
        self._server.register_goal_callback(self.goal_cb)
        self._server.register_preempt_callback(self.preempt_cb);
        
        self._subscriber = rospy.Subscriber(rowtopic, row, callback=self.row_msg_callback)
        self._last_row_msg = None
        
        self._server.start()
        
        
        
        self._timer = rospy.Timer(rospy.Duration(0.1),self.on_timer)
        
    def preempt_cb(self):
        """
            Called when the action client wishes to preempt us.
        """
        rospy.loginfo("preempt received")
        self.__send_safe_velocity()
        self._server.set_preempted()
        
    
    def goal_cb(self):
        """
            Called when we receive a new goal.
            We could inspect the goal before accepting it,
            however for start we only accept it.
        """
        rospy.loginfo("goal received")
        
        g = self._server.accept_new_goal()
        
        self.desired_offset = g.desired_offset_from_row
        self.max_out_of_headland_count = g.headland_timeout
        self.pgain = g.P
        self.distance_scale = g.distance_scale
        self.forward_velocity= g.forward_velcoity
        self.outofheadlandcount = 0
        
        # reset start and end pose
        self.__start_pose = self.get_pose()
        self.__end_pose = None
        
        self.left_valid_count = self.right_valid_count = 0
        
        self.start_time = rospy.Time.now()
        
    def row_msg_callback(self,row):
        """
            Stores the row received
        """
        self.cur_row = row

        
    def get_pose(self):
        """
            returns the pose of the vehicle in the odometry frame
            returns none if the vehicle could not be located
        """
        cur_pos = None
        
        try:
            cur_pos = self.__listen.lookupTransform(self.odom_frame,self.vehicle_frame,rospy.Time(0))
        except (LookupException, ConnectivityException),err:
            rospy.loginfo("could not locate vehicle")
            
        return cur_pos
    
        
    def get_distance(self):
        """
            returns the distance between the starting pose and ending pose
            recorded when a new goal is received and when the goal is successfully completed
            if the start or the end pose is not stored then 0 is returned
        """
        dist = 0
        if self.__start_pose and self.__end_pose:
            dist = math.sqrt(math.pow(self.__start_pose[0][0] - self.__end_pose[0][0],2) + pow(self.__start_pose[0][1] - self.__end_pose[0][1],2))
        return dist
        
            
    def on_timer(self,e):
        if self._server.is_active():
            if self.cur_row:
                if (rospy.Time.now() - self.cur_row.header.stamp) > rospy.Duration(1):
                    self.__end_pose = self.get_pose()
                    self.__goal_msg.distance_traveled = self.get_distance()
                    self._server.set_aborted(self.__goal_msg, "Row message is more than 1 second old aborting...") 
                    

                if self.outofheadlandcount > self.max_out_of_headland_count:
                    self.__end_pose = self.get_pose()
                    self.__goal_msg.distance_traveled = self.get_distance()
                    self._server.set_succeeded(self.__goal_msg, "Headland detected...")
                    rospy.loginfo("succeeded due to headland...")
                elif self.cur_row.headland:
                    self.outofheadlandcount += 1
                    rospy.loginfo("Row out of sight %d" % (self.outofheadlandcount))
                
                if self.cur_row.headland == False:
                    self.outofheadlandcount = 0
                        
                if self.outofheadlandcount < self.max_out_of_headland_count:
                    self.__calculate_velocity()
                else:
                    self.__send_safe_velocity()
            else:
                self.__send_safe_velocity()
                if rospy.Time.now() - self.start_time > rospy.Duration(5):
                    self._server.set_aborted(None, "No row message received within 5s") 
                
    def __send_safe_velocity(self):
        vel  = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        self.vel_pub.publish(vel)
    
    def __calculate_velocity(self):
        """
            Tries to regulate the vehicle so it follows the row.
            currently the angle error and the distance error are used 
            to calculate a total error based on the scalings provided by the goal
        """

        if self.cur_row.leftvalid and self.cur_row.rightvalid:
            # calculate the mean angle error so we can 
            # maintain a somewhat straight line in a crooked row
            angle_error = (self.cur_row.leftangle + self.cur_row.rightangle) / 2
            dist_error  = self.cur_row.rightdistance - self.cur_row.leftdistance 
            dist_error  = dist_error - self.desired_offset 
            self.left_valid_count = 0
            self.right_valid_count = 0
        else:
            angle_error = 0.0
            dist_error  = 0.0
            self.left_valid_count += 1
            self.right_valid_count += 1
            
        if self.right_valid_count > 30 or self.left_valid_count > 30:
            self._server.set_aborted(None, "Aborted due to one row not being valid")
        
        vel = Twist()
        vel.angular.z = (angle_error+dist_error*self.distance_scale) * self.pgain
        vel.linear.x = self.forward_velocity
        
        self.vel_pub.publish(vel)
        
        self.__feedback_msg.error_angle = angle_error
        self.__feedback_msg.error_distance = dist_error
        self.__feedback_msg.velocity_z = vel.angular.z
        
        self._server.publish_feedback(self.__feedback_msg)

        

if __name__ == "__main__":
    
    rospy.init_node("navigate_in_row_simple")
    
    NavigateInRowSimple(rospy.get_name(),"/fmExtractors/rows","odom_combined","base_footprint")
    
    rospy.spin()
    