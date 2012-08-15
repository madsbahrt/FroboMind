/*
 * kongkilde_rowcleaner.cpp
 *
 *  Created on: Aug 8, 2012
 *      Author: morl
 */

#include <ros/ros.h>
#include <fmMsgs/serial.h>
#include "kongskilderowcleaner.h"

int main(int argc, char **argv)
{

	//Initialize ros usage
	ros::init(argc, argv, "kongskilde_row_cleaner");

	//Create Nodehandlers
	ros::NodeHandle nn("");
	ros::NodeHandle nh("~");


	std::string serial_pub,serial_sub;


	nh.param<std::string>("serial_rx_sub",serial_sub,"/fmCSP/serial2_rx");
	nh.param<std::string>("serial_tx_pub",serial_pub,"/fmCSP/serial2_tx");

	kongskilde_rowcleaner node(nh,ros::Duration(7));

	ros::Subscriber s = nn.subscribe(serial_sub, 10, &kongskilde_rowcleaner::on_serial_rx, &node);
	node.serial_pub = nn.advertise<fmMsgs::serial>(serial_pub,5);

	ros::Timer t = nh.createTimer(ros::Duration(0.1),&kongskilde_rowcleaner::on_timer,&node);


	t.start();
	//Handle callbacks
	ros::spin();

}
