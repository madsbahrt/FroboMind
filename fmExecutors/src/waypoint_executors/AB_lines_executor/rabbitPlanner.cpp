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

bool rabbitPlanner::planRabbit(){



	if(path != NULL)
	{
		if(path->size() > 1)
		{
			try{

				tf_listener.waitForTransform(odom_frame.c_str(),vehicle_frame.c_str(),ros::Time::now(),ros::Duration(2));
				tf_listener.lookupTransform(odom_frame.c_str(),vehicle_frame.c_str(),ros::Time(0), transform);

				base = transform.getOrigin();

				path->at(current_waypoint).pose.position >> B;

				if(B.distance(base) < deltaWaypoint){
					ROS_INFO("Switching waypoint to: %.4f",B.distance(base));
					current_waypoint = (current_waypoint+1);
					if(current_waypoint >= path->size())
					{
						// we are done
						place_safe_rabbit();
						publishResult();
						return true;
					}
					path->at(current_waypoint).pose.position >> B;
					publishPath();
					publishFeedback();
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
				if(ABSquared == 0)
				{
					ROS_INFO("Received path with two consecutive identical poses, does not compute!");
				}

				rabbitScale = (Abase[0]*AB[0] + Abase[1]*AB[1])/ABSquared;

				if (rabbitScale <= 0){
					rabbit = B;
					ROS_INFO("Switching waypoint to: %.4f due to rabbitScale",rabbitScale);
					current_waypoint = (current_waypoint+1);
					if(current_waypoint >= path->size())
					{
						place_safe_rabbit();
						publishResult();
						return true;
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
			catch (tf::TransformException ex){
				ROS_WARN("%s",ex.what());
			}
		}
	}
	else
	{
		ROS_INFO("No path is present");
	}

	return false;
}




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
	success = false;
	current_waypoint = 1;

	path->clear();
	for(unsigned int i=0;i<goal->path.poses.size();i++)
	{
		path->push_back(goal->path.poses.at(i));
	}

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
	}
}

void rabbitPlanner::place_safe_rabbit()
{
	ROS_INFO("Planning safe rabbit");
	tf_rabbit.header.stamp = ros::Time::now();
	tf_rabbit.header.frame_id = odom_frame.c_str();
	tf_rabbit.child_frame_id = rabbit_frame.c_str();

	tf::Vector3 v = transform.getOrigin();

	try
	{
	tf_listener.lookupTransform(odom_frame.c_str(),vehicle_frame.c_str(),ros::Time(0), transform);
	tf_rabbit.transform.translation.x = v.getX();
	tf_rabbit.transform.translation.y = v.getY();
	tf_rabbit.transform.translation.z = v.getZ();
	tf::quaternionTFToMsg(transform.getRotation(),tf_rabbit.transform.rotation);


	tf_broadcaster.sendTransform(tf_rabbit);
	}
	catch (tf::TransformException ex){
		ROS_WARN("%s",ex.what());
	}
}



