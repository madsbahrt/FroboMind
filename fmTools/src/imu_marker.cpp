/*
 * cam_row_marker.cpp
 *
 *  Created on: Mar 26, 2012
 *      Author: morl
 */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <fmMsgs/claas_row_cam.h>
#include <sensor_msgs/Imu.h>

visualization_msgs::Marker marker;

geometry_msgs::Point line_start,line_end;

ros::Publisher p;
ros::Subscriber s;


void processRowEvent(const sensor_msgs::Imu::ConstPtr& msg)
{

	marker.pose.orientation = msg->orientation;

	p.publish(marker);
}

int main(int argc,char** argv)
{
	  ros::init(argc, argv, "imu_marker_node");
	  ros::NodeHandle nh("~");
	  ros::NodeHandle n;

	  std::string sub_topic,pub_topic,frame_id;


	  marker.type = visualization_msgs::Marker::ARROW;
	  marker.action = visualization_msgs::Marker::ADD;
	  marker.scale.x = 3;
	  marker.scale.y = 3;
	  marker.scale.z = 3;
	  marker.id = 0;
	  marker.ns = "imu_maker";
	  marker.color.a = 1.0;
	  marker.color.g = 1.0;

	  nh.param<std::string>("imu_subscriber_topic",sub_topic,"/fmSensors/IMU");
	  nh.param<std::string>("marker_publisher_topic",pub_topic,"/fmTools/imu_marker");
	  nh.param<std::string>("frame_id",frame_id,"imu_link");

	  marker.header.frame_id = frame_id;

	  p = nh.advertise<visualization_msgs::Marker>(pub_topic.c_str(),10);
	  s = nh.subscribe(sub_topic.c_str(),10,processRowEvent);

	  ros::spin();

}
