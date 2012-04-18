/*
 * PoseExtratorEncoderAckermanWithAngle.h
 *
 *  Created on: Feb 20, 2012
 *      Author: molar
 */

#ifndef POSEEXTRATORENCODERACKERMANWITHANGLE_H_
#define POSEEXTRATORENCODERACKERMANWITHANGLE_H_

#include "PoseExtractorEncoderBase.h"

class PoseExtractorEncoderAckermanWithAngle: public PoseExtractorEncoderBase {
public:
	PoseExtractorEncoderAckermanWithAngle();
	PoseExtractorEncoderAckermanWithAngle(ros::Publisher p,double rate,double max_diff);
	virtual ~PoseExtractorEncoderAckermanWithAngle();

	void processEncoderAngle(const fmMsgs::encoder::ConstPtr & msg);
	void setEncoderAngleConversionConst(double ticks_to_rad);
	void setVehicleLength(double length);

	int angle_offset;

protected:

	virtual void calculatePose();
	bool checkUpdated();
	void clearUpdated();

	fmMsgs::encoder current_angle,previous_angle;

	double angle_ticks_to_rad;
	bool angle_updated;

	double previous_l,previous_r;

	double vehicle_length;

private:



};

#endif /* POSEEXTRATORENCODERACKERMANWITHANGLE_H_ */
