/*
 * PoseExtractorEncoderWithAngleFWE.h
 *
 *  Created on: Feb 20, 2012
 *      Author: molar
 */

#ifndef POSEEXTRACTORENCODERWITHANGLEFWE_H_
#define POSEEXTRACTORENCODERWITHANGLEFWE_H_

#include "PoseExtractorEncoderAckermanWithAngle.h"
#include <math.h>

class PoseExtractorEncoderAckermanWithAngleFWE: public PoseExtractorEncoderAckermanWithAngle {
public:
	PoseExtractorEncoderAckermanWithAngleFWE();
	PoseExtractorEncoderAckermanWithAngleFWE(ros::Publisher p, double rate,double max_diff);
	virtual ~PoseExtractorEncoderAckermanWithAngleFWE();

protected:
	void calculatePose();

};

#endif /* POSEEXTRACTORENCODERWITHANGLEFWE_H_ */
