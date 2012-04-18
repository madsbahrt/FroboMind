/*
 * PoseExtratorEncoderAckermanWithAngle.cpp
 *
 *  Created on: Feb 20, 2012
 *      Author: molar
 */

#include "PoseExtractorEncoderAckermanWithAngle.h"

PoseExtractorEncoderAckermanWithAngle::PoseExtractorEncoderAckermanWithAngle() {

	this->angle_ticks_to_rad = 1.0;
	this->vehicle_length = 1.0;
	x = y =  0.0;
	vx = vy = 0.0;
	previous_l = previous_r = 0.0;
}

PoseExtractorEncoderAckermanWithAngle::PoseExtractorEncoderAckermanWithAngle(ros::Publisher p, double rate,double max_diff)
 : PoseExtractorEncoderBase(p,rate,max_diff)
{
	this->angle_ticks_to_rad = 1.0;
	this->vehicle_length = 1.0;
	x = y =  0.0;
	vx = vy = 0.0;
	previous_l = previous_r = 0.0;
}

PoseExtractorEncoderAckermanWithAngle::~PoseExtractorEncoderAckermanWithAngle() {
	// TODO Auto-generated destructor stub
}

void PoseExtractorEncoderAckermanWithAngle::processEncoderAngle(const fmMsgs::encoder::ConstPtr & msg)
{
	if(msg->header.stamp < this->current_angle.header.stamp)
	{
		ROS_WARN("Ignoring old encoder value");
	}
	else
	{
		if((current_angle.header.stamp - previous_angle.header.stamp).toSec() < max_time_diff)
		{
			angle_updated = true;
		}
		this->previous_angle = this->current_angle;
		this->current_angle = *msg;
	}
}



void PoseExtractorEncoderAckermanWithAngle::setEncoderAngleConversionConst(double ticks_to_rad)
{
	this->angle_ticks_to_rad = ticks_to_rad;
}



void PoseExtractorEncoderAckermanWithAngle::calculatePose()
{
	double dx,dy,dtheta;


	double alpha = this->current_angle.encoderticks * (this->angle_ticks_to_rad + angle_offset);
	double dt_alpha;
	double odo_l,dt_odo_l;
	double odo_r;

	double odl,odr;

	odl = this->current_left.encoderticks * this->l_ticks_to_m;
	odr = this->current_right.encoderticks * this->r_ticks_to_m;


	odo_l = odl - previous_l;
	odo_r = odr - previous_r;

	dt_odo_l = (current_left.header.stamp - previous_left.header.stamp).toSec();

	dt_alpha = (current_angle.header.stamp - previous_angle.header.stamp).toSec();
	previous_l = odl;
	previous_r = odr;

	// calculate the mean distance traveled
	double dist = (odo_l + odo_r)/2;

	ROS_DEBUG("Distance traveled: %.5f",dist);

	dx = dist * cos(theta) * cos(alpha);
	dy = dist * sin(theta) * cos(alpha);
	dtheta = dist/this->vehicle_length * sin(alpha);


	ROS_DEBUG("dx: %.4f dy: %.4f dtheta: %.4f",dx,dy,dtheta);

	// update state
	x+=dx;
	y+=dy;
	theta+=dtheta;

	vx = dx / dt_odo_l;
	vy = dy / dt_odo_l;
	vtheta = dtheta/dt_alpha;
}


void PoseExtractorEncoderAckermanWithAngle::setVehicleLength(double length)
{
	this->vehicle_length = length;
}



void PoseExtractorEncoderAckermanWithAngle::clearUpdated()
{
	l_updated = r_updated = angle_updated = false;
}

bool PoseExtractorEncoderAckermanWithAngle::checkUpdated()
{
	return l_updated & r_updated & angle_updated;
}



