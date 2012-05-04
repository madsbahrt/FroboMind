/*
 * RabbitFollow.h
 *
 *  Created on: Apr 27, 2012
 *      Author: morl
 */

#ifndef RABBITFOLLOW_H_
#define RABBITFOLLOW_H_

#include <tf/transform_listener.h>

class RabbitFollow
{
public:
	RabbitFollow(std::string,std::string);
	virtual ~RabbitFollow();

	void spin(const ros::TimerEvent& e);

	ros::Publisher cmd_vel_pub;
	double fov;
	double max_ang_vel;
	double max_lin_vel;
	double oscilation_bound;
	double target_acquired_tolerance;


private:
	void findTheRabbit();
	void driveToTheRabbit();


	double current_rabbit_heading;
	double previous_rabbit_heading;
	double distance;

	double P,I,D;
	double Pg,Ig,Dg;
	double error;
	double I_max;

	std::string rabbit_frame;
	std::string odom_frame;
	std::string vehicle_frame;

	tf::TransformListener tf_listen;

	geometry_msgs::TwistStamped cmd_vel;
};

#endif /* RABBITFOLLOW_H_ */
