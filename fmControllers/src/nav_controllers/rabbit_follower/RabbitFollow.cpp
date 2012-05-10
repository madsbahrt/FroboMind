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

	I = P = 0;
	Ig = 0.001;
	Pg = 0.2;
	I_max = 0;
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

	// update PI parameters before running loop
	nh->getParamCached("P_gain",Pg);
	nh->getParamCached("I_gain",Ig);
	nh->getParamCached("I_max",I_max);

	driveToTheRabbit();
}

void RabbitFollow::findTheRabbit()
{
	try
	{
		tf::StampedTransform transformer;
		tf_listen.waitForTransform(vehicle_frame,rabbit_frame,ros::Time::now(),ros::Duration(2));
		tf_listen.lookupTransform(vehicle_frame,rabbit_frame,ros::Time(0), transformer);

		tf::Vector3 xyz = transformer.getOrigin();

				previous_rabbit_heading = current_rabbit_heading;
				current_rabbit_heading =atan2(xyz[1],xyz[0]);
				distance = sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1]);


				ROS_DEBUG_THROTTLE(1,"Found rabbit %.4f %.4f %.4f distance %.4f",xyz[0],xyz[1],current_rabbit_heading,distance);

	}
	catch (tf::TransformException& ex){
		ROS_WARN("follower: FAILED!");
		ROS_WARN("%s",ex.what());
	}
}

void RabbitFollow::driveToTheRabbit()
{

	//TODO: do some fancy scaling  of the cmd_vel
	// PI controller

	I += current_rabbit_heading*0.1;
	if(I > I_max)
	{
		I = I_max;
	}
	else if(I < -I_max)
	{
		I = -I_max;
	}

	P = current_rabbit_heading * Pg;

	cmd_vel.angular.z = (P + I*Ig);

	cmd_vel.linear.x = max_lin_vel;

	if(current_rabbit_heading > fov/2 || current_rabbit_heading < -fov/2)
	{
		// turn in place
		cmd_vel.linear.x = 0;
	}

	if(distance < 0.1)
	{
		ROS_INFO("target_reached");
		cmd_vel.angular.z = 0;
		cmd_vel.linear.x = 0;
	}


	cmd_vel_pub.publish(cmd_vel);

}


