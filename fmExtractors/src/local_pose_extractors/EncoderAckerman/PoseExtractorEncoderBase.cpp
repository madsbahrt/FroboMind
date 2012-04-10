/*
 * PoseExtractorEncoderBase.cpp
 *
 *  Created on: Feb 20, 2012
 *      Author: molar
 */

#include "PoseExtractorEncoderBase.h"

PoseExtractorEncoderBase::PoseExtractorEncoderBase() {
	// TODO Auto-generated constructor stub
	resetAll();

}

PoseExtractorEncoderBase::PoseExtractorEncoderBase(ros::Publisher p, double rate,double max_diff)
{
	resetAll();
	this->odom_pub = p;
	this->pub_rate = rate;
	this->max_time_diff = max_diff;
}

PoseExtractorEncoderBase::~PoseExtractorEncoderBase() {
	// TODO Auto-generated destructor stub
}

void PoseExtractorEncoderBase::processEncoderLeft(const fmMsgs::encoder::ConstPtr & msg)
{
	// check time
	if(msg->header.stamp <= current_left.header.stamp)
	{
		ROS_WARN("Ignoring old timestamp");
		return;
	}
	else
	{
		if((current_left.header.stamp - previous_left.header.stamp).toSec() < max_time_diff)
		{
			l_updated = true;
		}

		previous_left = current_left;
		current_left = *msg;
	}
}



void PoseExtractorEncoderBase::processEncoderRight(const fmMsgs::encoder::ConstPtr & msg)
{
	// check time
	if(msg->header.stamp <= current_right.header.stamp)
	{
		ROS_WARN("Ignoring old timestamp");
		return;
	}
	else
	{
		if((current_right.header.stamp - previous_right.header.stamp).toSec() < max_time_diff)
		{
			r_updated = true;
		}
		previous_right = current_right;
		current_right = *msg;
	}

}



void PoseExtractorEncoderBase::setPublishTopic(ros::Publisher p)
{
	this->odom_pub = p;
}



void PoseExtractorEncoderBase::setPublishRate(double rate)
{
	this->pub_rate = rate;
}



void PoseExtractorEncoderBase::setEncoderLeftConversionConst(double ticks_to_m)
{
	this->l_ticks_to_m = ticks_to_m;
}



void PoseExtractorEncoderBase::setEncoderRightConversionConst(double ticks_to_m)
{
	this->r_ticks_to_m = ticks_to_m;
}



void PoseExtractorEncoderBase::calculatePose()
{
	double dt_l = (current_left.header.stamp
				- previous_left.header.stamp).toSec();
	double dt_r = (current_right.header.stamp
			- previous_right.header.stamp).toSec();

	// the old encoder message was ticks changed since last time, for now we convert the absolute encoder input to the old format
	int cl_ticks,cr_ticks;

	cl_ticks = current_left.encoderticks - previous_left.encoderticks;
	cr_ticks = current_right.encoderticks - previous_right.encoderticks;

	double dt = (dt_l + dt_r) / 2;

	double velocity_ = (((2 * M_PI * (0.24)) * (-cl_ticks / (2) + cr_ticks / (2)) / 8192)) / dt_l;

	double omega_ = ((2 * M_PI * 0.24) * (((1/(0.47))*cl_ticks +(1/(0.4713))*cr_ticks)/8192)/dt_l)/5*M_PI;

	ROS_DEBUG("Velocity: %f \t Omega: %f \t dt_l %f", velocity_, omega_, dt_l);

	//compute odometry in a typical way given the velocities of the robot
	double dx = (velocity_ * cos(theta)) * dt;
	double dy = (velocity_ * sin(theta)) * dt;
	double dtheta = omega_ * dt_l;

	x += dx;
	y += dy;
	theta += dtheta;

	// this may not be correct, but is what was found on the ASuBot
	vx = velocity_;
	vy = 0.0;
	vtheta = omega_;

}

void PoseExtractorEncoderBase::resetAll()
{
	this->l_ticks_to_m = 1.0;
	this->r_ticks_to_m = 1.0;

	this->l_updated = false;
	this->r_updated = false;

	this->pub_rate = 1.0;

	this->x = this->y = this->theta = 0.0;
	this->vx = this->vy = this->vtheta = 0.0;

}

bool PoseExtractorEncoderBase::checkUpdated()
{
	return l_updated & r_updated;
}

void PoseExtractorEncoderBase::clearUpdated()
{
	l_updated = r_updated = false;
}




void PoseExtractorEncoderBase::spin(const ros::TimerEvent& e)
{
	// get the time this loop got called
	ros::Time current_time = ros::Time::now();

	//ROS_INFO("Spinng Odometry at %f",current_time.toSec());

	// check for updated encoder input
	if(checkUpdated())
	{
		clearUpdated();
		calculatePose();

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

		this->odom_trans.header.stamp = current_time;
		this->odom_trans.header.frame_id = "odom";
		this->odom_trans.child_frame_id = "base_link";
		this->odom_trans.transform.translation.x = x;
		this->odom_trans.transform.translation.y = y;
		this->odom_trans.transform.translation.z = 0.0;
		this->odom_trans.transform.rotation = odom_quat;

		this->odom_broadcaster.sendTransform(this->odom_trans);

		this->odom.header.stamp = current_time;
		this->odom.header.frame_id = "odom";
		this->odom.child_frame_id = "base_link";

		this->odom.pose.pose.position.x = x;
		this->odom.pose.pose.position.y = y;
		this->odom.pose.pose.position.z = 0.0;
		this->odom.pose.pose.orientation = odom_quat;

		this->odom.twist.twist.linear.x = vx;
		this->odom.twist.twist.linear.y = vy;
		this->odom.twist.twist.linear.z = 0.0;
		this->odom.twist.twist.angular.x = 0.0;
		this->odom.twist.twist.angular.y = 0.0;
		this->odom.twist.twist.angular.z = vtheta;

		this->odom_pub.publish(this->odom);


	}
	else
	{
		// new odometry not available
		ROS_WARN("Odometry not updated due to old input");
	}

}








