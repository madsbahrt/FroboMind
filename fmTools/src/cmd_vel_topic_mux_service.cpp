/*
 * cmd_vel_topic_mux_action.cpp
 *
 *  Created on: May 14, 2012
 *      Author: morl
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <fmTools/switch_mux.h>

class CmdVelMux
{

public:

	bool topic1_active,topic2_active;
	ros::Time last_t1,last_t2;
	ros::Duration max_t1,max_t2;
	geometry_msgs::TwistStamped t1_msg;
	geometry_msgs::Twist t2_msg;
	geometry_msgs::TwistStamped msg_out;
	bool topic2_override;

	ros::Publisher p;

	CmdVelMux(ros::Duration max_delta_topic1,ros::Duration max_delta_topic2)
	{
		max_t1 = max_delta_topic1;
		max_t2 = max_delta_topic2;
		topic1_active = topic2_active = false;
		topic2_override = false;
	}

	void onTopic1(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		if(topic1_active == false)
		{

			topic1_active = true;
			t1_msg = *msg;
		}
		else
		{
			if(last_t1 - ros::Time::now()>max_t1)
			{
				topic1_active = false;
			}
			else
			{
				t1_msg = *msg;
			}
		}
		last_t1 = ros::Time::now();
	}

	void onTopic2(const geometry_msgs::Twist::ConstPtr& msg)
	{
		if(topic2_active == false)
		{

			topic2_active = true;
			t2_msg = *msg;
		}
		else
		{
			if(last_t2 - ros::Time::now()>max_t1)
			{
				topic1_active = false;
			}
			else
			{
				t2_msg = *msg;
			}
		}
		last_t2 = ros::Time::now();

	}

	bool on_mux_server_request(fmTools::switch_mux::Request& req,fmTools::switch_mux::Response& resp)
	{
		if(req.mode == fmTools::switch_muxRequest::AUTO)
		{
			ROS_INFO("Overriding cmd_vel");
			topic2_override = true;
			resp.result = fmTools::switch_muxResponse::RESULT_OK;
		}
		else if(req.mode == fmTools::switch_muxRequest::MANUEL)
		{
			ROS_INFO("Defaulting to cmd_vel priority");
			topic2_override = false;
			resp.result = fmTools::switch_muxResponse::RESULT_OK;
		}
		else
		{
			ROS_ERROR("Received incorrect mux request");
			resp.result = fmTools::switch_muxResponse::RESULT_ERR;
		}
		return true;
	}

	void spin(const ros::TimerEvent& e)
	{

		if(topic1_active && not topic2_override)
		{
			msg_out.header.stamp = ros::Time::now();
			msg_out.twist.linear.x = t1_msg.twist.linear.x;
			msg_out.twist.angular.z = t1_msg.twist.angular.z;
		}
		else if(topic2_active)
		{
			//msg_out = t2_msg;
			msg_out.header.stamp = ros::Time::now();
			msg_out.twist.linear.x = t2_msg.linear.x;
			msg_out.twist.angular.z = t2_msg.angular.z;
		}
		else
		{
			msg_out.header.stamp = ros::Time::now();
			msg_out.twist.linear.x = 0;
			msg_out.twist.angular.z = 0;
		}

		p.publish(msg_out);

	}
};



int main(int argc, char** argv)
{
	ros::init(argc,argv,"cmd_vel_mux");

	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	std::string topic1_id,topic2_id,outid;
	double t1,t2;

	ros::Subscriber s1,s2;

	nh.param<double>("topic_1_timeout",t1,1);
	nh.param<double>("topic_2_timeout",t2,1);

	nh.param<std::string>("topic_1_id",topic1_id,"/fmTools/cmd_vel_1");
	nh.param<std::string>("topic_2_id",topic2_id,"/fmTools/cmd_vel_2");
	nh.param<std::string>("cmd_vel_out_id",outid,"/fmTools/cmd_vel");

	CmdVelMux* mux;

	mux = new CmdVelMux(ros::Duration(t1),ros::Duration(t2));

	s1 = nh.subscribe<geometry_msgs::TwistStamped,CmdVelMux>(topic1_id,10,&CmdVelMux::onTopic1,mux);
	s2 = nh.subscribe<geometry_msgs::Twist,CmdVelMux>(topic2_id,10,&CmdVelMux::onTopic2,mux);

	mux->p = nh.advertise<geometry_msgs::TwistStamped>(outid,10);

	ros::Timer t = nh.createTimer(ros::Duration(0.05),&CmdVelMux::spin,mux);

	ros::ServiceServer service = nh.advertiseService<CmdVelMux,fmTools::switch_mux::Request,fmTools::switch_mux::Response>("cmd_vel_mux",&CmdVelMux::on_mux_server_request,mux);

	ros::spin();


	return 0;
}
