/*
 * simple_odom.cpp
 *
 *  Created on: Apr 25, 2012
 *      Author: morl
 */
#include <ros/ros.h>
#include <ros/console.h>

#include <fmMsgs/encoder.h>
#include <nav_msgs/Odometry.h>
#include <ros/subscriber.h>

#include <string>

using namespace std;

class SimpleOdom
{
public:
	SimpleOdom(double conv_l,double conv_r,double vl,double max_diff)
	{
		c_r = conv_r;
		c_l = conv_l;
		v_l = vl;
		this->max_diff = max_diff;

		delta_l = delta_r = 0;

		l_updated = r_updated = l_ready = r_ready = false;

	}

	~SimpleOdom()
	{

	}

	void processLeftEncoder(const fmMsgs::encoder::ConstPtr& msg)
	{
		if(!l_ready)
		{
			prev_l = *msg;
			l_ready = true;
		}
		else
		{
			delta_l += (msg->encoderticks - prev_l.encoderticks) * c_l;
			l_up_time = ros::Time::now();
			l_updated = true;
			prev_l = *msg;
		}

	}

	void processRightEncoder(const fmMsgs::encoder::ConstPtr& msg)
	{
		if(!r_ready)
		{
			prev_r = *msg;
			r_ready = true;
		}
		else
		{
			delta_r += (msg->encoderticks - prev_r.encoderticks) * c_r;
			r_up_time = ros::Time::now();
			r_updated = true;
			prev_r = *msg;
		}

	}

	void publishOdometry(const ros::TimerEvent& e)
	{
		if(l_updated && r_updated)
		{
			// check update times
			if((l_up_time - r_up_time).toSec() > max_diff)
			{
				ROS_WARN("Encoder stamp (left - right) differs %.4f",(l_up_time - r_up_time).toSec());
			}

			r_updated = l_updated = false;

			// calculate distance vector
			double dx = (delta_l + delta_r)/2;

			// calculate change in orientation based on distance between wheels
			double dtheta = (-delta_l / (2*v_l)) + (delta_r/(2 * v_l));

			odom.header.stamp = ros::Time::now();
			odom.header.frame_id = "local_odom";
			odom.twist.twist.linear.x  = dx;
			odom.twist.twist.angular.z = dtheta;

			odom_pub.publish(odom);

			delta_l = delta_r = 0;

		}

	}

	ros::Publisher odom_pub;

private:
	double c_l,c_r,v_l;

	double delta_l,delta_r;

	bool l_updated,r_updated;

	ros::Time l_up_time,r_up_time;

	bool l_ready,r_ready;

	double max_diff;

	fmMsgs::encoder prev_l,prev_r;
	nav_msgs::Odometry odom;
};

int main(int argc, char** argv) {

	ros::init(argc, argv, "odometry_publisher");

	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	string publish_topic;
	string subscribe_enc_l;
	string subscribe_enc_r;


	double v_l,r_t_m,l_t_m,max_diff;
	ros::Subscriber s1,s2;




	nh.param<string>("publisher_topic", publish_topic, "/fmExtractors/odom");
	nh.param<string>("enc_r_subscriber_topic", subscribe_enc_r,
			"/fmSensors/encoder_right");
	nh.param<string>("enc_l_subscriber_topic", subscribe_enc_l,
			"/fmSensors/encoder_left");

	nh.param<double>("conv_ticks_to_meter_left", l_t_m, 2*(M_PI)*0.24/8192);
	nh.param<double>("conv_ticks_to_meter_right", r_t_m, 2*(M_PI)*0.24/8192);
	nh.param<double>("max_time_diff", max_diff,1);

	nh.param<double>("distance_between_wheels_in_meter", v_l, 0.59);

	SimpleOdom p(l_t_m,r_t_m,v_l,max_diff);

	s1 = nh.subscribe(subscribe_enc_l,15,&SimpleOdom::processLeftEncoder,&p);
	s2 = nh.subscribe(subscribe_enc_r,15,&SimpleOdom::processRightEncoder,&p);

	p.odom_pub = n.advertise<nav_msgs::Odometry>(publish_topic.c_str(), 25);
	ros::Timer t;
	t = nh.createTimer(ros::Duration(1/30.0), &SimpleOdom::publishOdometry,&p);

	ros::spin();

	return 0;
}




