/*
 * rabbitPlanner.h
 *
 *  Created on: Jan 1, 2001
 *      Author: soeni05
 */

#ifndef RABBITPLANNER_H_
#define RABBITPLANNER_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class rabbitPlanner{
public:

	tf::TransformBroadcaster tf_broadcaster;
	tf::TransformListener tf_listener;


	int current_waypoint;

	rabbitPlanner(std::vector<geometry_msgs::PoseStamped>* path, double dRabbit);
	bool initRabbit();
	void planRabbit();

private:



	std::vector<geometry_msgs::PoseStamped>* path;
	double dRabbit;

	tf::Vector3 rabbit;
	tf::Vector3 base;
	tf::Vector3 A;
	tf::Vector3 B;

	geometry_msgs::TransformStamped tf_rabbit;
	geometry_msgs::TransformStamped tf_A;
	geometry_msgs::TransformStamped tf_B;



};



#endif /* RABBITPLANNER_H_ */
