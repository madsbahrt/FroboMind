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

	pathPlan = new pathParser;
	rabbit = new rabbitPlanner(pathPlan->path, 2);

	rabbit->initRabbit();

	ros::Rate r(50);

	int test = 0;

	while(ros::ok()){

		rabbit->planRabbit();

		ros::spinOnce();

		if(test++ > 500){
			test = 0;
			rabbit->current_waypoint = (rabbit->current_waypoint+1)%pathPlan->path->size() ;
		}


		r.sleep();

	}



	return 0;

}
