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
}

PoseExtractorEncoderAckermanWithAngle::PoseExtractorEncoderAckermanWithAngle(ros::Publisher p, double rate,double max_diff)
 : PoseExtractorEncoderBase(p,rate,max_diff)
{
	this->angle_ticks_to_rad = 1.0;
	this->vehicle_length = 1.0;
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


	double alpha = this->current_angle.encoderticks * this->angle_ticks_to_rad;
	double odo_l = this->current_left.encoderticks * this->l_ticks_to_m;
	double odo_r = this->current_right.encoderticks * this->r_ticks_to_m;

	double dt_angle,dt_odom;

	// temporary calculation variables
	double R,phi;
	double odo_mean;

	odo_mean = (odo_l+odo_r) / 2.0;

	dt_angle = (this->current_angle.header.stamp - this->previous_angle.header.stamp).toSec();
	dt_odom = (this->current_left.header.stamp - this->previous_left.header.stamp).toSec();

	if(alpha > 0.01 || alpha < 0.01)
	{
		// rotation large enough
		R = this->vehicle_length / tan(alpha);
		phi = odo_mean / R;

		dx = R*sin(phi);
		dy = R-R*cos(phi);
		dtheta = phi;
	}
	else
	{
		// rotation not large enough for good calculations
		dx = odo_mean;
		dy = 0.0;
		dtheta = 0.0;
	}


	// update state
	x+=dx;
	y+=dy;
	theta+=dtheta;

	vx = dx / dt_odom;
	vy = dy / dt_odom;
	vtheta = dtheta/dt_angle;

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



