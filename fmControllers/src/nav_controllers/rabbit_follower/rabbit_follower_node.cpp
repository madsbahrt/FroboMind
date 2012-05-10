/*
 * rabbit_follower_node.cpp
 *
 *  Created on: Apr 27, 2012
 *      Author: morl
 */


#include <ros/ros.h>
#include "RabbitFollow.h"




int main(int argc, char **argv) {

	//Initialize ros usage
	ros::init(argc, argv, "follow_the_rabbit");

	std::string r_frame,o_frame;
	std::string topic_id;

	//Create Nodehandlers
	ros::NodeHandle nh("~");
	ros::NodeHandle n();

	nh.param<std::string>("rabbit_frame_id",r_frame,"/rabbit");
	nh.param<std::string>("vehicle_frame_id",o_frame,"/base_footprint");
	nh.param<std::string>("cmd_vel_topic_id",topic_id,"/fmControllers/cmd_vel");

	RabbitFollow alice(r_frame,o_frame);

	nh.param<double>("field_of_view_rad",alice.fov,2*M_PI);
	nh.param<double>("max_angular_vel",alice.max_ang_vel,1.5);
	nh.param<double>("max_linear_vel",alice.max_lin_vel,1.0);
	nh.param<double>("oscilation_bounds",alice.oscilation_bound,0.05);
	nh.param<double>("target_acquired_tolerance",alice.target_acquired_tolerance,0.1);
	alice.nh = &nh;

	ros::Timer t = nh.createTimer(ros::Duration(0.05),&RabbitFollow::spin,&alice);

	alice.cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(topic_id.c_str(),10);

	t.start();
	//Handle callbacks
	ros::spin();

}
