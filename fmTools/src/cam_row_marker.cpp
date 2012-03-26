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

	line_end.x = 2;
	line_end.y = 2 * tan(msg->heading/360.0 * 2* 3.1415);

	marker.points.push_back(line_start);
	marker.points.push_back(line_end);

	p.publish(marker);
}

int main(int argc,char** argv)
{
	  ros::init(argc, argv, "row_marker_node");
	  ros::NodeHandle nh("~");
	  ros::NodeHandle n;

	  std::string sub_topic,pub_topic;

	  marker.header.frame_id = "row_cam";
	  marker.type = visualization_msgs::Marker::LINE_STRIP;

	  n.param<std::string>("row_subscriber_topic",sub_topic,"/fmSensors/row");
	  n.param<std::string>("marker_publisher_topic",pub_topic,"/fmTools/row_marker");


	  p = nh.advertise<visualization_msgs::Marker>(pub_topic.c_str(),10);
	  s = nh.subscribe(sub_topic.c_str(),10,processRowEvent);

	  ros::spin();

}
