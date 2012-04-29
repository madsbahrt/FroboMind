/*
 * RabbitFollow.cpp
 *
 *  Created on: Apr 27, 2012
 *      Author: morl
 */

#include "RabbitFollow.h"
#include <math.h>

RabbitFollow::RabbitFollow(std::string rabbit,std::string alice)
{
	// TODO Auto-generated constructor stub
	vehicle_frame = alice;
	rabbit_frame = rabbit;
}

RabbitFollow::~RabbitFollow()
{
	// TODO Auto-generated destructor stub
}

void RabbitFollow::spin(const ros::TimerEvent& e)
{
	findTheRabbit();

	// prevent oscilation when the rabbit is directly behind us.
	if(previous_rabbit_heading < -M_PI+oscilation_bound && current_rabbit_heading > M_PI-oscilation_bound)
	{
		current_rabbit_heading = previous_rabbit_heading;
	}
	else if(previous_rabbit_heading > M_PI-oscilation_bound && current_rabbit_heading < -M_PI + oscilation_bound)
	{
		current_rabbit_heading = previous_rabbit_heading;
	}

	driveToTheRabbit();
}

void RabbitFollow::findTheRabbit()
{
	if(tf_listen.canTransform(vehicle_frame,rabbit_frame,ros::Time::now()))
	{
		tf::StampedTransform transformer;
		tf_listen.lookupTransform(vehicle_frame,rabbit_frame,ros::Time::now(),transformer);

		tf::Vector3 xyz = transformer.getOrigin();

		previous_rabbit_heading = current_rabbit_heading;
		current_rabbit_heading =atan2(xyz[1],xyz[0]);
		distance = sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1]);


		ROS_DEBUG("Found rabbit %.4f %.4f %.4f distance %.4f",xyz[0],xyz[1],current_rabbit_heading,distance);
	}
}

void RabbitFollow::driveToTheRabbit()
{

	//TODO: do some fancy scaling  of the cmd_vel

	if(distance > target_acquired_tolerance)
	{
		// do not move
		cmd_vel.twist.linear.x = 0;
		cmd_vel.twist.angular.z = 0;
	}
	else
	{
		if(current_rabbit_heading > 0.01) // 0.5 deg tolerance
		{
			cmd_vel.twist.angular.z = max_ang_vel;
		}
		else if  (current_rabbit_heading < -0.01)
		{
			cmd_vel.twist.angular.z = -max_ang_vel;
		}
		else
		{
			cmd_vel.twist.angular.z = 0;
		}
	}

	cmd_vel.header.stamp = ros::Time::now();

	cmd_vel.twist.linear.x = max_lin_vel;

	cmd_vel_pub.publish(cmd_vel);

}


