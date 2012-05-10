/*
 * pathExecutor.cpp
 *
 *  Created on: Jan 1, 2001
 *      Author: soeni05
 */

#include "rabbitPlanner.h"
#include <geometry_msgs/PoseStamped.h>
#include <fmExecutors/follow_pathAction.h>


rabbitPlanner * rabbit = NULL;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "rabbitPlanner");
	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	std::string filepath,path_pub_topic;
/*
	std::vector<geometry_msgs::PoseStamped> * path;
	path = new std::vector<geometry_msgs::PoseStamped> ;
	geometry_msgs::PoseStamped pose;


	pose.pose.position.y =6137265.86408 -2;
	pose.pose.position.x =588783.625059;

	path->push_back(pose);
	pose.pose.position.y =6137272.3533 -2;
	pose.pose.position.x =588728.99886;
	path->push_back(pose);

	pose.pose.position.y =6137281.24059 -2;
	pose.pose.position.x =588660.727205;

	path->push_back(pose);
	pose.pose.position.y =6137288.18503 -2;
	pose.pose.position.x =588613.910558;
	path->push_back(pose);
*/
/*
    pose.pose.position.x = 588813.821135;
    pose.pose.position.y = 6137242.22287;
    		path->push_back(pose);
	  pose.pose.position.x = 588831.458665;
	  pose.pose.position.y = 6137217.36418;
				path->push_back(pose);
	  pose.pose.position.x = 588845.501653;
	  pose.pose.position.y = 6137271.32139;

		path->push_back(pose);

*/

	rabbit = new rabbitPlanner("follow_path");

	nh.param<std::string>("path_publisher_topic",path_pub_topic,"/fmExecutors/path");
	//base_projected_to_A_B / deltaRabbit = rabbit
	nh.param<double>("deltaRabbit", rabbit->deltaRabbit, 2);
	//Waypoint threshold (0 = change waypoint when rabbit > B has been passed ; 1 = change waypoint when rabbit >= B-1 has been passed)
	nh.param<double>("deltaWaypoint", rabbit->deltaWaypoint, 0);

	nh.param<std::string>("rabbit_type", rabbit->rabbit_type, "float");

	nh.param<std::string>("odometry_frame", rabbit->odom_frame, "odom");
	nh.param<std::string>("vehicle_frame", rabbit->vehicle_frame, "base_link");
	nh.param<std::string>("rabbit_frame", rabbit->rabbit_frame, "rabbit");

	// latched topic so new subscriber get the most recent path
	rabbit->path_publisher = nh.advertise<nav_msgs::Path>(path_pub_topic.c_str(),5,true);

	ros::spin();

	return 0;

}
