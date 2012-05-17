/*
 * roboteq_encoder_node.cpp
 *
 *  Created on: May 15, 2012
 *      Author: morl
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <fmMsgs/encoder.h>
#include <fmMsgs/serial.h>

class RobotTeqEncoder
{
private:
	int total_ticks;
	fmMsgs::serial serial_msg_out;
	fmMsgs::encoder encoder_msg_out;



	bool reply_received;
public:

	ros::Publisher p_serial,p_encoder;

	ros::Time last_query_time;


	RobotTeqEncoder()
	{
		total_ticks = 0;
		// force first transmission of CBR command
		reply_received = true;
	}

	void onSerialMsgReceived(const fmMsgs::serial::ConstPtr& msg)
	{
		int cbr;
		if(sscanf(msg->data.c_str(),"CBR=%d",&cbr))
		{
			total_ticks += cbr;
			reply_received = true;
		}
	}


	void spin(const ros::TimerEvent& e)
	{
		if(reply_received)
		{
			serial_msg_out.header.stamp = ros::Time::now();
			serial_msg_out.data = "?CBR\r";

			p_serial.publish(serial_msg_out);

			last_query_time = ros::Time::now();
			reply_received = false;

		}
		else
		{
			if(ros::Time::now() - last_query_time > ros::Duration(1))
			{
				ROS_DEBUG("have not heard from robotteq controller in 1 second, retrying");
				reply_received = true;
			}
		}

		encoder_msg_out.header.stamp = ros::Time::now();
		encoder_msg_out.encoderticks = total_ticks;

		p_encoder.publish(encoder_msg_out);

	}
};




int main(int argc,char** argv)
{
	ros::init(argc,argv,"roboteq_encoder_node");

	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	ros::Subscriber s1;

	std::string serial_rx_topic,serial_tx_topic,encoder_topic;

	nh.param<std::string>("serial_rx_topic",serial_rx_topic,"/fmCSP/S0_rx");
	nh.param<std::string>("serial_tx_topic",serial_tx_topic,"/fmCSP/S0_tx");
	nh.param<std::string>("encoder_topic",encoder_topic,"/fmSensors/encoder_ticks");

	RobotTeqEncoder roboteq;

	s1 = nh.subscribe<fmMsgs::serial>(serial_rx_topic,10,&RobotTeqEncoder::onSerialMsgReceived,&roboteq);
	roboteq.p_serial = nh.advertise<fmMsgs::serial>(serial_tx_topic,10);
	roboteq.p_encoder =nh.advertise<fmMsgs::encoder>(encoder_topic,10);

	ros::Rate r(5);

	ROS_INFO("Waiting for serial to subscribe");
	while(roboteq.p_serial.getNumSubscribers() == 0)
	{
		r.sleep();
	}

	ros::Timer t = nh.createTimer(ros::Duration(0.5),&RobotTeqEncoder::spin,&roboteq);

	ros::spin();

	return 0;
}
