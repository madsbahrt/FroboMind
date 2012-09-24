/*
 * EXSInterface_node.cpp
 *
 *  Created on: Sep 23, 2012
 *      Author: morl
 */

#include <ros/ros.h>
#include <fmMsgs/can.h>
#include <fmMsgs/encoder.h>
#include <fmMsgs/steering_angle_cmd.h>
#include "EXSInterface.h"


int main(int argc, char** argv)
{
	ros::init(argc,argv,"EXSInterface_node");
	ros::NodeHandle nh("~");
	ros::NodeHandle n;
	ros::Subscriber s1,s2,s3;
	std::string can_pub,can_sub,encoder_pub,cmd_vel_sub,steering_sub,angle_pub;
	double rr;

	nh.param<std::string>("can_tx_publisher_topic",can_pub,"/fmCSP/can0_tx");
	nh.param<std::string>("can_rx_subscriber_topic",can_sub,"/fmCSP/can0_rx");
	nh.param<std::string>("encoder_publisher_topic",encoder_pub,"/fmSensors/encoder_lr");
	nh.param<std::string>("angle_publisher_topic",angle_pub,"/fmSensors/encoder_angle");
	nh.param<std::string>("cmd_vel_subscriber_topic",cmd_vel_sub,"/fmKinematics/cmd_vel");
	nh.param<std::string>("steering_angle_subscriber_topic",steering_sub,"fmKinematics/steering_angle_cmd");
	nh.param<double> ("output_rate",rr,20);

	EXSInterface esx;

	esx.encoder_pub = nh.advertise<fmMsgs::encoder>(encoder_pub,10);
	esx.angle_pub = nh.advertise<fmMsgs::encoder>(angle_pub,10);
	esx.can_tx_pub = nh.advertise<fmMsgs::can>(can_pub,10);

	s1 = nh.subscribe<fmMsgs::can>(can_sub.c_str(),10,&EXSInterface::onCANMsg,&esx);
	s2 = nh.subscribe<fmMsgs::steering_angle_cmd> (steering_sub,10,&EXSInterface::onSteeringAngle,&esx);
	s3 = nh.subscribe<geometry_msgs::Twist> (cmd_vel_sub,10,&EXSInterface::onCmdVel,&esx);

	ros::Timer t = nh.createTimer(ros::Rate(rr),&EXSInterface::onTimer,&esx);

	ros::spin();

	return 0;
}



