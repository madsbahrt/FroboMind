/*
 * pathExecutor.cpp
 *
 *  Created on: Jan 1, 2001
 *      Author: soeni05
 */

//#include "pathParser.h"
#include "rabbitPlanner.h"
#include <geometry_msgs/PoseStamped.h>


//pathParser * pathPlan;
rabbitPlanner * rabbit;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rabbitPlanner");
	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	std::string filepath;

	nh.param<std::string>("pathfile_location",filepath,"waypoints.yaml");

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


//	pathPlan = new pathParser(filepath);

//	if(!pathPlan->path->size()){
//		ROS_ERROR("pathParser: no path loaded at %s",filepath.c_str());
//		return 0;
//	}

	rabbit = new rabbitPlanner(path);

	//base_projected_to_A_B / deltaRabbit = rabbit
	nh.param<double>("deltaRabbit", rabbit->deltaRabbit, 2);
	//Waypoint threshold (0 = change waypoint when rabbit > B has been passed ; 1 = change waypoint when rabbit >= B-1 has been passed)
	nh.param<double>("deltaWaypoint", rabbit->deltaWaypoint, 0);

	nh.param<std::string>("rabbit_type", rabbit->rabbit_type, "float");

	nh.param<std::string>("odometry_frame", rabbit->odom_frame, "odom");
	nh.param<std::string>("vehicle_frame", rabbit->vehicle_frame, "base_link");
	nh.param<std::string>("rabbit_frame", rabbit->rabbit_frame, "rabbit");

	rabbit->initRabbit();

	ros::Rate r(50);

	while(ros::ok()){
		rabbit->planRabbit();
		ros::spinOnce();
		r.sleep();
	}

	return 0;

}
