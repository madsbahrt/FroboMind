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
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/server/simple_action_server.h>
#include <fmExecutors/follow_pathAction.h>

class rabbitPlanner{
public:

	int current_waypoint;

	double deltaRabbit,deltaWaypoint;
	std::string odom_frame, vehicle_frame,rabbit_frame,rabbit_type;
	rabbitPlanner(std::string name);

	void publishPath();
	void publishFeedback();
	void publishResult();
	bool planRabbit();



	void actionExecute(const fmExecutors::follow_pathGoalConstPtr& goal);

	ros::Publisher path_publisher;


private:

	bool locateVehicle();
	void publishWPAB();
	bool calculateRabbit();
	void publishRabbit();
	void place_safe_rabbit();

	std::vector<geometry_msgs::PoseStamped>* path;

	tf::TransformBroadcaster tf_broadcaster;
	tf::TransformListener tf_listener;

	tf::Vector3 rabbit;
	tf::Vector3 base;
	tf::Vector3 A;
	tf::Vector3 B;

	ros::NodeHandle nh;

	tf::StampedTransform transform;

	geometry_msgs::TransformStamped tf_rabbit;
	geometry_msgs::TransformStamped tf_A;
	geometry_msgs::TransformStamped tf_B;

	actionlib::SimpleActionServer<fmExecutors::follow_pathAction> action_server;
	std::string action_name;
	fmExecutors::follow_pathFeedback feedback_msg;
	fmExecutors::follow_pathResult   result_msg;

	bool success;
	bool once;




};



#endif /* RABBITPLANNER_H_ */
