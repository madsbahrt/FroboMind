/*
 * utm_to_odom.cpp
 *
 *  Created on: Apr 20, 2012
 *      Author: morl
 */
#include <ros/ros.h>
#include <fmMsgs/gps_state.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>
#include <math.h>

ros::Publisher odom_pub;

fmMsgs::gps_state ref_fix;
fmMsgs::gps_state prev_msg;


nav_msgs::Odometry odom;

typedef struct
{
	double x;
	double y;
}gps_points_t;

boost::circular_buffer<gps_points_t> gps_points_buffer(25);

double gps_variance;

geometry_msgs::TransformStamped odom_trans;
std::string frame_id,child_frame_id,tf_child_frame_id,tf_frame_id;


bool publish_relative = false;
bool utm_settled=false;
int utm_settled_count = 0;
int utm_settled_count_top = 0;

double heading_threshold;
double heading_variance;


bool calculate_heading(double& heading)
{
	bool ret = false;



	if(gps_points_buffer.capacity() == gps_points_buffer.size())
	{
		gps_points_t p_comp = gps_points_buffer[0];

		for(size_t i=1; i < gps_points_buffer.size();i++)
		{
			gps_points_t p_cur = gps_points_buffer[i];
			double dist = sqrt(pow(p_comp.x - p_cur.x,2) + pow(p_comp.y - p_cur.y,2));
			if(dist > heading_threshold)
			{
				heading = atan2(p_comp.y - p_cur.y,p_comp.x - p_cur.x);
				ret = true;
				break;
			}
		}
	}

	return ret;
}

void utmCallback(const fmMsgs::gps_state::ConstPtr& msg)
{

	static tf::TransformBroadcaster odom_broadcaster;
	if(msg->hdop < 2 && utm_settled == false)
	{
		utm_settled_count++;
		if(utm_settled_count >= utm_settled_count_top)
		{
			utm_settled = true;
			ref_fix = *msg;
			prev_msg = *msg;

		}
	}
	else if(utm_settled)
	{

		gps_points_t p;
		p.x = msg->utm_e;
		p.y = msg->utm_n;

		gps_points_buffer.push_front(p);

		odom.header.stamp = ros::Time::now();
		odom.header.frame_id =frame_id;
		odom.child_frame_id = child_frame_id;

		if(publish_relative){
			odom.pose.pose.position.x = -(ref_fix.utm_e - msg->utm_e);
			odom.pose.pose.position.y = -(ref_fix.utm_n - msg->utm_n);
		}
		else
		{
			// ENU
			odom.pose.pose.position.x =msg->utm_e;
			odom.pose.pose.position.y =msg->utm_n;
		}

		odom.pose.pose.position.z = 0;

		static double yaw=0;

		if(calculate_heading(yaw))
		{
			odom.pose.covariance[35] = heading_variance;
		}
		else
		{
			odom.pose.covariance[35] = 999999;

		}

		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		odom.pose.covariance[0] = msg->hdop * gps_variance;

		odom.pose.covariance[7] = msg->hdop * gps_variance;

		odom.pose.covariance[14] = 999999;

		odom.pose.covariance[21] = 999999;
		odom.pose.covariance[28] = 999999;


		odom_pub.publish(odom);

		prev_msg = *msg;
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "utm_to_odom");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string subscribe_topic_id;
	std::string publish_topic_id;

	n.param<std::string>("subscribe_topic_id", subscribe_topic_id,
			"/fmExtractors/utm");
	n.param<std::string>("publish_topic_id", publish_topic_id,
			"/fmExtractors/gps_odom");
	n.param<std::string>("odom_frame_id",frame_id,"odom_combined");
	n.param<std::string>("child_odom_frame_id",child_frame_id,"gps_link");
	n.param<bool>("publish_relative_coordinates",publish_relative,false);
	n.param<int>("receive_n_before_publish",utm_settled_count_top,3);
	n.param<double>("gps_variance",gps_variance,1);

	n.param<double>("gps_heading_threshold",heading_threshold,1);
	n.param<double>("gps_heading_variance",heading_variance,0.01);
	ros::Subscriber sub = n.subscribe(subscribe_topic_id, 10, utmCallback);
	odom_pub = n.advertise<nav_msgs::Odometry>(publish_topic_id, 1);

	ros::spin();
	return 0;
}
