/*
 * cam_row_marker.cpp
 *
 *  Created on: Mar 26, 2012
 *      Author: morl
 */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <fmMsgs/claas_row_cam.h>

visualization_msgs::Marker marker;
geometry_msgs::Point line_start,line_end;
ros::Publisher p;
ros::Subscriber s;

double line_length;

void processRowEvent(const fmMsgs::claas_row_cam::ConstPtr& msg)
{
	marker.points.clear();
	line_start.x = 0;
	line_start.y = -(msg->offset/100.0);

	line_end.x = line_length;
	line_end.y = line_length * tan(msg->heading/360.0 * 2* 3.1415) -(msg->offset/100.0);

	marker.points.push_back(line_start);
	marker.points.push_back(line_end);

	marker.color.a = 1.0;
	marker.color.r = 1 - msg->quality/255.0;
	marker.color.g = msg->quality/255.0;

	p.publish(marker);
}

int main(int argc,char** argv)
{
	  ros::init(argc, argv, "row_marker_node");
	  ros::NodeHandle nh("~");
	  ros::NodeHandle n;

	  std::string sub_topic,pub_topic,frame_id;


	  marker.type = visualization_msgs::Marker::LINE_STRIP;
	  marker.action = visualization_msgs::Marker::ADD;
	  marker.scale.x = 0.1;

	  nh.param<std::string>("row_subscriber_topic",sub_topic,"/fmSensors/row");
	  nh.param<std::string>("marker_publisher_topic",pub_topic,"/fmTools/row_marker");
	  nh.param<std::string>("frame_id",frame_id,"cam_link");
	  nh.param<double>("line_length",line_length,2);

	  marker.header.frame_id = frame_id;

	  p = nh.advertise<visualization_msgs::Marker>(pub_topic.c_str(),10);
	  s = nh.subscribe(sub_topic.c_str(),10,processRowEvent);

	  ros::spin();

}
