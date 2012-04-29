/*
 * pathExecutor.cpp
 *
 *  Created on: Jan 1, 2001
 *      Author: soeni05
 */

#include "pathParser.h"
#include "rabbitPlanner.h"

pathParser * pathPlan;
rabbitPlanner * rabbit;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rabbitPlanner");
	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	std::string filepath;

	nh.param<std::string>("pathfile_location",filepath,"waypoints.yaml");

	pathPlan = new pathParser(filepath);

	rabbit = new rabbitPlanner(pathPlan->path);

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
