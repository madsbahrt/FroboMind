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

ros::Publisher odom_pub;

fmMsgs::gps_state ref_fix;
fmMsgs::gps_state prev_msg;

nav_msgs::Odometry odom;


geometry_msgs::TransformStamped odom_trans;
std::string frame_id,tf_child_frame_id,tf_frame_id;


bool publish_relative = false;
bool utm_settled=false;
int utm_settled_count = 0;

void utmCallback(const fmMsgs::gps_state::ConstPtr& msg)
{

	static tf::TransformBroadcaster odom_broadcaster;
	if(msg->hdop < 2 && utm_settled == false)
	{
		utm_settled_count++;
		if(utm_settled_count > 30)
		{
			utm_settled = true;
			ref_fix = *msg;
			prev_msg = *msg;

		}
	}
	else if(utm_settled)
	{

		odom.header.stamp = ros::Time::now();
		odom.header.frame_id =frame_id;

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

		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

		odom.pose.covariance[0] = msg->hdop * msg->hdop * 0.01;

		odom.pose.covariance[7] = msg->hdop * msg->hdop* 0.01;

		odom.pose.covariance[14] = 999999;

		odom.pose.covariance[21] = 999999;
		odom.pose.covariance[28] = 999999;
		odom.pose.covariance[35] = 999999;

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
	n.param<std::string>("odom_frame_id",frame_id,"base_footprint");
	n.param<bool>("publish_relative_coordinates",publish_relative,false);

	ros::Subscriber sub = n.subscribe(subscribe_topic_id, 10, utmCallback);
	odom_pub = n.advertise<nav_msgs::Odometry>(publish_topic_id, 1);

	ros::spin();
	return 0;
}
