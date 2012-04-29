/*
 * rabbitPlanner.cpp
 *
 *  Created on: Jan 1, 2001
 *      Author: soeni05
 */

#include "rabbitPlanner.h"

void operator >> (geometry_msgs::Point point, tf::Vector3& vec){
	vec[0] = point.x;
	vec[1] = point.y;
	vec[2] = point.z;
}

rabbitPlanner::rabbitPlanner(std::vector<geometry_msgs::PoseStamped> * in_path)
{
	this->path = in_path;
}

bool rabbitPlanner::initRabbit(){

	tf::StampedTransform transform;

	bool initialized = false;

	while(!initialized && ros::ok())
	{
		try
		{
			tf_listener.waitForTransform(odom_frame.c_str(),vehicle_frame.c_str(),ros::Time::now(),ros::Duration(2));
			tf_listener.lookupTransform(odom_frame.c_str(),vehicle_frame.c_str(),ros::Time(0), transform);
			base = transform.getOrigin();
			initialized = true;

			ROS_INFO("rabbitPlanner::initRabbit: DONE!");
		}
		catch (tf::TransformException ex){
			ROS_WARN("rabbitPlanner::initRabbit: FAILED!");
			ROS_WARN("%s",ex.what());
		}


	}

	double min_distance = 10000000;

	for(size_t i = 0; i < path->size(); i++){
		tf::Vector3 posevec;
		path->at(i).pose.position >> posevec;

		if(posevec.distance(base)<min_distance && fabs(posevec.distance(base)) > 0){
			min_distance = posevec.distance(base);
			current_waypoint = i;
		}
	}

	ROS_INFO("rabbitPlanner::initRabbit:current_waypoint (%d): DONE!",current_waypoint);

	return initialized;

}

void rabbitPlanner::planRabbit(){

	tf::StampedTransform transform;

	try{

		tf_listener.waitForTransform(odom_frame.c_str(),vehicle_frame.c_str(),ros::Time::now(),ros::Duration(2));
		tf_listener.lookupTransform(odom_frame.c_str(),vehicle_frame.c_str(),ros::Time(0), transform);

		base = transform.getOrigin();

		path->at(current_waypoint).pose.position >> B;

		if(B.distance(base) < deltaWaypoint){
			current_waypoint = (current_waypoint+1)%path->size();
			path->at(current_waypoint).pose.position >> B;
		}

		if(current_waypoint > 0){
			path->at(current_waypoint-1).pose.position >> A;
		}else{
			path->at(path->size()-1).pose.position >> A;
		}

		tf_A.header.stamp = ros::Time::now();
		tf_A.header.frame_id = odom_frame.c_str();
		tf_A.child_frame_id = "wpA";

		tf_A.transform.translation.x = A.getX();
		tf_A.transform.translation.y = A.getY();
		tf_A.transform.translation.z = A.getZ();
		tf_A.transform.rotation.x = 0;
		tf_A.transform.rotation.y = 0;
		tf_A.transform.rotation.z = 0;
		tf_A.transform.rotation.w = 1;
		tf_broadcaster.sendTransform(tf_A);

		tf_B.header.stamp = ros::Time::now();
		tf_B.header.frame_id = odom_frame.c_str();
		tf_B.child_frame_id = "wpB";
		tf_B.transform.rotation.x = 0;
		tf_B.transform.rotation.y = 0;
		tf_B.transform.rotation.z = 0;
		tf_B.transform.rotation.w = 1;

		tf_B.transform.translation.x = B.getX();
		tf_B.transform.translation.y = B.getY();
		tf_B.transform.translation.z = B.getZ();
		tf_broadcaster.sendTransform(tf_B);

		// NOW ! Calculate zhe rabbit !

		static tf::Vector3 AB;
		static double ABSquared;
		static tf::Vector3 Abase;
		static double rabbitScale;

		AB = B-A;
		ABSquared = AB[1]*AB[1]+AB[0]*AB[0];
		Abase = B-base;

		rabbitScale = (Abase[0]*AB[0] + Abase[1]*AB[1])/ABSquared;

		if (rabbitScale < 0){
			rabbit = B;
			current_waypoint = (current_waypoint+1)%path->size();
		} else {
			rabbit = ((B-(rabbitScale * AB)));

			if(rabbit_type == "fixed"){
				rabbit = rabbit + (deltaRabbit)*AB/ABSquared;
			}else if(rabbit_type == "float"){
				rabbit = (B-rabbit)/deltaRabbit+rabbit;
			}else{
				ROS_ERROR("RABBIT WENT BACK TO ITS HOLE! WRONG 'rabbit_type' ");
			}
		}

		tf_rabbit.header.stamp = ros::Time::now();
		tf_rabbit.header.frame_id = odom_frame.c_str();
		tf_rabbit.child_frame_id = rabbit_frame.c_str();

		tf_rabbit.transform.translation.x = rabbit.getX();
		tf_rabbit.transform.translation.y = rabbit.getY();
		tf_rabbit.transform.translation.z = rabbit.getZ();
		tf_rabbit.transform.rotation.x = 0;
		tf_rabbit.transform.rotation.y = 0;
		tf_rabbit.transform.rotation.z = 0;
		tf_rabbit.transform.rotation.w = 1;

		tf_broadcaster.sendTransform(tf_rabbit);


		std::cout << "rabbit:"<< rabbit[0] << " " << rabbit[1] << std::endl;

    }
    catch (tf::TransformException ex){
		ROS_WARN("%s",ex.what());
    }


}
