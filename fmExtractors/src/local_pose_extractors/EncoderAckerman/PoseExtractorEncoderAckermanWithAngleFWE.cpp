/*
 * PoseExtractorEncoderWithAngleFWE.cpp
 *
 *  Created on: Feb 20, 2012
 *      Author: molar
 */

#include "PoseExtractorEncoderAckermanWithAngleFWE.h"

PoseExtractorEncoderAckermanWithAngleFWE::PoseExtractorEncoderAckermanWithAngleFWE() {
	// TODO Auto-generated constructor stub

}

PoseExtractorEncoderAckermanWithAngleFWE::PoseExtractorEncoderAckermanWithAngleFWE(ros::Publisher p, double rate, double max_diff)
 : PoseExtractorEncoderAckermanWithAngle(p,rate,max_diff)
{

}

PoseExtractorEncoderAckermanWithAngleFWE::~PoseExtractorEncoderAckermanWithAngleFWE() {
	// TODO Auto-generated destructor stub
}

void PoseExtractorEncoderAckermanWithAngleFWE::calculatePose()
{
	double odo_mean,phi;
	double dx,dy;

	double d_enc_l = (current_left.encoderticks - previous_left.encoderticks) * l_ticks_to_m;
	double d_enc_r = (current_right.encoderticks - previous_right.encoderticks) * r_ticks_to_m;
	double alpha = (current_angle.encoderticks * angle_ticks_to_rad);

	double dt_odo_l = (current_left.header.stamp - previous_left.header.stamp).toSec();
	double dt_odo_r = (current_right.header.stamp - previous_right.header.stamp).toSec();

	double dt_odo_mean = (dt_odo_l + dt_odo_r) / 2;

	double dt_theta = (current_angle.header.stamp - previous_angle.header.stamp).toSec();
	odo_mean = (d_enc_l - d_enc_r) / 2;

	phi = atan((sin(alpha)*odo_mean) /(vehicle_length + cos(alpha) * odo_mean) );

	dx = cos(phi)*cos(alpha)*odo_mean;
	dy = -sin(phi)*cos(alpha)*odo_mean;

	x+=dx;
	y+=dy;
	theta+=phi;

	vx = dx / dt_odo_mean;
	vy = dy / dt_odo_mean;
	vtheta = phi / dt_theta;

}



