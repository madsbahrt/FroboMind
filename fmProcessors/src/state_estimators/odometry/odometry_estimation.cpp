/*
 * odometry_estimation.cpp
 *
 *  Created on: Apr 22, 2012
 *      Author: morl
 */
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "PositionEstimator.h"
#include <kalman/ekfilter.hpp>



class OdometryKalmanNode
{
public:
	OdometryKalmanNode()
	{
		is_imu_initialised = false;
		is_odom_initialised = false;
		current_heading = 0;

		base_frame = "base_footprint";


		filter = new PositionEstimator(10000,0.01);

		Kalman::KVector<double,1,1> prior(3);
		Kalman::KMatrix<double,1,1> prior_cov(3,3);

		for(unsigned int i=1;i <= prior_cov.nrow();i++)
		{
			for(unsigned int j = 1; j<=prior_cov.ncol();j++)
			{
				if(i==j)
					prior_cov(i,j) = 1;
				else
					prior_cov(i,j) = 0;
			}
		}

		prior(1) = prior(2) = prior(3) = 0;
		prior(3) = -M_PI/2;

		filter->init(prior,prior_cov);

	}

	~OdometryKalmanNode()
	{

	}

	void processIMU(const sensor_msgs::Imu::ConstPtr& imu_msg)
	{
		double heading = tf::getYaw(imu_msg->orientation);

		heading = -(heading- M_PI/2);

		if(heading > M_PI)
		{
			heading -= 2*M_PI;
		}
		else if(heading < -M_PI)
		{
			heading += 2*M_PI;
		}

		current_heading = heading;
	}

	void processOdomControl(const nav_msgs::Odometry::ConstPtr& odom_msg)
	{
		Kalman::KVector<double,1,1> u(2);
		Kalman::KVector<double,1,1> state(3);


		u(1) = odom_msg->twist.twist.linear.x;
		u(2) = current_heading;
		//ROS_ERROR("Control: %.4f %.4f",u(1),u(2));
		state = filter->getX();
		//ROS_ERROR("State before time update: %.4f %.4f %.4f",state(1),state(2),state(3));
		filter->timeUpdateStep(u);
		state = filter->getX();
		//ROS_ERROR("State after time update: %.4f %.4f %.4f",state(1),state(2),state(3));
		publishEstimate();
	}


	void processOdomUpdate(const nav_msgs::Odometry::ConstPtr& odom_msg)
	{
		Kalman::KVector<double,1,1> state(3);

		Kalman::KVector<double,1,1> z(2);

		z(1) = odom_msg->pose.pose.position.x;
		z(2) = odom_msg->pose.pose.position.y;
		//ROS_ERROR("Measurement: %.4f %.4f",z(1),z(2));
		state = filter->getX();
		//ROS_ERROR("State before measurement update: %.4f %.4f %.4f",state(1),state(2),state(3));
		filter->measureUpdateStep(z);
		state = filter->getX();
		//ROS_ERROR("State after measurement update: %.4f %.4f %.4f",state(1),state(2),state(3));


	}

	void publishEstimate()
	{
		Kalman::KVector<double,1,1> state(3);
		state = filter->getX();

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state(3));

		this->odom_trans.header.stamp = ros::Time::now();
		this->odom_trans.header.frame_id = "odom_combined";
		this->odom_trans.child_frame_id = "base_footprint";
		this->odom_trans.transform.translation.x = state(1);
		this->odom_trans.transform.translation.y = state(2);
		this->odom_trans.transform.translation.z = 0.0;
		this->odom_trans.transform.rotation = odom_quat;

		this->odom_broadcaster.sendTransform(this->odom_trans);

		pub_odom_msg.header.stamp = ros::Time::now();

		pub_odom_msg.header.frame_id = "odom_combined";
		pub_odom_msg.pose.pose.position.x = state(1);
		pub_odom_msg.pose.pose.position.y = state(2);

		pub_odom_msg.pose.pose.orientation = odom_quat;

		odom_est_pub.publish(pub_odom_msg);
	}

	ros::Publisher odom_est_pub;

	tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::TransformStamped odom_trans;

private:

	bool is_imu_initialised;
	bool is_odom_initialised;

	nav_msgs::Odometry prev_odom_msg;
	sensor_msgs::Imu prev_imu_msg;

	std::string base_frame;

	PositionEstimator* filter;

	double current_heading;

	nav_msgs::Odometry pub_odom_msg;

	tf::TransformListener listener;
};





int main(int argc, char **argv)
{


	ros::init(argc, argv, "odometry_kalman_filter");

	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	std::string imu_sub_top;
	std::string odom_sub_top;
	std::string odom_est_pub_top;
	std::string  gps_odom_sub_top;

	ros::Subscriber imu_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber gps_sub;

	OdometryKalmanNode node;


	nh.param<std::string>("imu_subscriber_topic", imu_sub_top, "/fmSensors/IMU");
	nh.param<std::string>("odom_subscriber_topic", odom_sub_top, "/fmExtractors/wheel_odom");
	nh.param<std::string>("gps_odom_subscriber_topic", gps_odom_sub_top,"/fmExtractors/gps_odom");
	nh.param<std::string>("odom_estimate_publisher_topic", odom_est_pub_top,"/fmProcessors/odom_estimate");

	imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_sub_top.c_str(),10,&OdometryKalmanNode::processIMU,&node);
	odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_sub_top.c_str(),10,&OdometryKalmanNode::processOdomControl,&node);
	gps_sub = nh.subscribe<nav_msgs::Odometry>(gps_odom_sub_top.c_str(),10,&OdometryKalmanNode::processOdomUpdate,&node);

	node.odom_est_pub = n.advertise<nav_msgs::Odometry>(odom_est_pub_top.c_str(),5);

	ros::spin();
}
