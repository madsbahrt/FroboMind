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

rabbitPlanner::rabbitPlanner(std::string name)
	: action_server(nh,name, boost::bind(&rabbitPlanner::actionExecute, this, _1), false),action_name(name)
{

	path = new std::vector<geometry_msgs::PoseStamped>();
	action_server.start();

}




bool rabbitPlanner::planRabbit()
{
	bool ret = false;
	static tf::Vector3 AB; // vector from A to B
	static double ABSquared;
	static tf::Vector3 Abase; // vector from A to Base
	static double rabbitScale; // distance from point to line

	if(locateVehicle())
	{
		//load a and B from plan

		path->at(current_waypoint-1).pose.position >> A;
		path->at(current_waypoint).pose.position >> B;


		// calculate distances
		AB = B-A;
		ABSquared = AB[1]*AB[1]+AB[0]*AB[0];
		Abase = B-base;

		rabbitScale = (Abase[0]*AB[0] + Abase[1]*AB[1])/ABSquared;


		// check if target reached either because it is close enough to B waypoint
		// or if it has passed the waypoint
		if(B.distance(base) < deltaWaypoint)
		{
			ROS_INFO("Switching waypoint due to deltaWaypoint check, %.4f",deltaWaypoint);

			current_waypoint++;
			if(current_waypoint >= path->size())
			{
				// we are done
				ret = true;
			}
			else
			{
				// create new A and B
				path->at(current_waypoint-1).pose.position >> A;
				path->at(current_waypoint).pose.position >> B;
				publishFeedback();
			}

		}
		else if(rabbitScale <= 0)
		{
			ROS_INFO("Switching waypoint due to rabbitScale check, %.4f",rabbitScale);

			current_waypoint++;
			if(current_waypoint >= path->size())
			{
				// we are done
				ret = true;
			}
			else
			{
				// create new A and B
				path->at(current_waypoint-1).pose.position >> A;
				path->at(current_waypoint).pose.position >> B;
				publishFeedback();
			}

		}

		if(!ret)
		{
			// place rabbit
			rabbit = ((B-(rabbitScale * AB)));

			if(rabbit_type == "fixed"){
				rabbit = rabbit + (deltaRabbit)*AB/ABSquared;
			}else if(rabbit_type == "float"){
				rabbit = (B-rabbit)/deltaRabbit+rabbit;
			}else if(rabbit_type == "auto")
			{
				double auto_factor = deltaRabbit * rabbit.distance(base);
				if(auto_factor < 1)
				{
					auto_factor = 1;
				}
				//ROS_INFO("Rabbit distance ABase is %.4f",rabbit.distance(base));
				rabbit = (B-rabbit)/(auto_factor)+rabbit;
			}else{
				ROS_ERROR("RABBIT WENT BACK TO ITS HOLE! WRONG 'rabbit_type' ");
			}

			if(B.distance(rabbit) < deltaWaypoint)
			{
				rabbit = B;
			}

			publishWPAB();
			publishRabbit();

		}
		else
		{
			place_safe_rabbit();
			publishResult();
		}

	}
	else
	{
		ROS_ERROR_THROTTLE(1,"Could not locate vehicle");
	}

	return ret;
}
/*
bool rabbitPlanner::planRabbit()
{
	bool ret = false;

	if(locateVehicle())
	{
		//load a and B from plan

		if((current_waypoint == 1) && (path->size() == 1 ))
		{
			A=base;
			path->at(current_waypoint-1).pose.position >> B;
		}
		else
		{
			path->at(current_waypoint-1).pose.position >> A;
			path->at(current_waypoint).pose.position >> B;
		}

		// check if target reached
		if(B.distance(base) < deltaWaypoint){
			ROS_DEBUG("Switching waypoint because base is %.4f from B",B.distance(base));

			current_waypoint++;

			if(current_waypoint > path->size())
			{
				// we are done
				place_safe_rabbit();
				publishResult();
				ret = true;
			}
			else
			{
				path->at(current_waypoint-1).pose.position >> A;
				path->at(current_waypoint).pose.position >> B;
				publishPath();
				publishFeedback();
			}

		}

		//publish the location of wpA and wpB
		publishWPAB();

		if(!ret)
		{
			// NOW ! Calculate zhe rabbit !
			if(calculateRabbit())
			{
				ret = true;
			}
			else
			{
				publishRabbit();
			}
		}
	}
	else
	{
		ROS_INFO_THROTTLE(1,"Vehicle frame cannot be found, not planning");
	}



	return ret;
}
*/



void rabbitPlanner::publishPath()
{
	nav_msgs::Path msg;

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = odom_frame;

	for(unsigned int i=current_waypoint; i< path->size();i++)
	{
		msg.poses.push_back(path->at(i));
	}

	path_publisher.publish(msg);

}

void rabbitPlanner::publishFeedback()
{
	ROS_INFO("Goal feedback published");
	feedback_msg.reached_pose_nr = current_waypoint;

	action_server.publishFeedback(feedback_msg);
}

void rabbitPlanner::publishResult()
{
	ROS_INFO("Goal result published");
	geometry_msgs::PoseStamped p;

	p.header.frame_id = odom_frame;
	p.header.stamp = ros::Time::now();
	p.pose.position.x = base.getX();
	p.pose.position.y = base.getY();
	p.pose.position.z = base.getZ();
	tf::quaternionTFToMsg(transform.getRotation(),p.pose.orientation);
	result_msg.resulting_pose = p;
	action_server.setSucceeded(result_msg);
}

void rabbitPlanner::actionExecute(const fmExecutors::follow_pathGoalConstPtr& goal)
{
	ROS_INFO("Goal received");

	//reset variables
	success = false;
	once = false;
	current_waypoint = 1;

	ros::Rate r(25);

	path->clear();
	for(unsigned int i=0;i<goal->path.poses.size();i++)
	{
		path->push_back(goal->path.poses.at(i));
	}
	ROS_INFO("Path has %d entries",path->size());
	publishPath();

	while(!success)
	{
		if(action_server.isPreemptRequested() || !ros::ok())
		{
			ROS_INFO("Goal is preempted");
			// action is preempted
			action_server.setPreempted();
			place_safe_rabbit();
			break;
		}
		else
		{
			success = planRabbit();
		}
		r.sleep();

	}
}

void rabbitPlanner::publishWPAB()
{
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
}

bool rabbitPlanner::calculateRabbit()
{
	bool ret = false;
	static tf::Vector3 AB;
	static double ABSquared;
	static tf::Vector3 Abase;
	static double rabbitScale;

	AB = B-A;
	ABSquared = AB[1]*AB[1]+AB[0]*AB[0];
	Abase = B-base;

	if(ABSquared == 0)
	{
		ROS_ERROR("Received path with two consecutive identical poses, does not compute!");
	}
	else
	{

		// distance from point to line
		rabbitScale = (Abase[0]*AB[0] + Abase[1]*AB[1])/ABSquared;

		// if the distance from the point (vehicle) to the line(AB) is negative we have passed the line segment
		// and should navigate to the next
		if (rabbitScale <= 0)
		{
			rabbit = B;
			ROS_INFO("Switching waypoint due to rabbitScale %.4f",rabbitScale);
			current_waypoint++;

			if(current_waypoint > path->size())
			{
				place_safe_rabbit();
				publishResult();
				ret = true;
			}
		} else {
			rabbit = ((B-(rabbitScale * AB)));

			if(rabbit_type == "fixed"){
				rabbit = rabbit + (deltaRabbit)*AB/ABSquared;
			}else if(rabbit_type == "float"){
				rabbit = (B-rabbit)/deltaRabbit+rabbit;
			}else if(rabbit_type == "auto")
			{
				double auto_factor = deltaRabbit * rabbit.distance(base);
				if(auto_factor < 1)
				{
					auto_factor = 1;
				}
				//ROS_INFO("Rabbit distance ABase is %.4f",rabbit.distance(base));
				rabbit = (B-rabbit)/(auto_factor)+rabbit;
			}else{
				ROS_ERROR("RABBIT WENT BACK TO ITS HOLE! WRONG 'rabbit_type' ");
			}
		}
	}
	return ret;
}

void rabbitPlanner::publishRabbit()
{
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
}

bool rabbitPlanner::locateVehicle()
{
	bool ret = false;
	try
	{
		tf_listener.waitForTransform(odom_frame.c_str(),vehicle_frame.c_str(),ros::Time::now(),ros::Duration(2));
		tf_listener.lookupTransform(odom_frame.c_str(),vehicle_frame.c_str(),ros::Time(0), transform);

		base = transform.getOrigin();
		ret = true;
	}
	catch (tf::TransformException& ex){
		ROS_WARN("%s",ex.what());
	}

	return ret;
}

void rabbitPlanner::place_safe_rabbit()
{
	ROS_INFO("Planning safe rabbit");
	tf_rabbit.header.stamp = ros::Time::now();
	tf_rabbit.header.frame_id = odom_frame.c_str();
	tf_rabbit.child_frame_id = rabbit_frame.c_str();

	try
	{
	tf_listener.lookupTransform(odom_frame.c_str(),vehicle_frame.c_str(),ros::Time(0), transform);
	tf::Vector3 v = transform.getOrigin();
	tf_rabbit.transform.translation.x = v.getX();
	tf_rabbit.transform.translation.y = v.getY();
	tf_rabbit.transform.translation.z = v.getZ();
	tf::quaternionTFToMsg(transform.getRotation(),tf_rabbit.transform.rotation);


	tf_broadcaster.sendTransform(tf_rabbit);
	}
	catch (tf::TransformException& ex){
		ROS_WARN("%s",ex.what());
	}
}



