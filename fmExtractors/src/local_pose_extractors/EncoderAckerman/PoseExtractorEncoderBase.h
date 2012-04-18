/*
 * PoseExtractorEncoderBase.h
 *
 *  Created on: Feb 20, 2012
 *      Author: molar
 */

#ifndef POSEEXTRACTORENCODERBASE_H_
#define POSEEXTRACTORENCODERBASE_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <fmMsgs/encoder.h>

class PoseExtractorEncoderBase {
public:
	PoseExtractorEncoderBase();
	PoseExtractorEncoderBase(ros::Publisher p,double rate,double max_diff);

	virtual ~PoseExtractorEncoderBase();

	void processEncoderLeft(const fmMsgs::encoder::ConstPtr& msg);
	void processEncoderRight(const fmMsgs::encoder::ConstPtr& msg);

	void setPublishTopic(ros::Publisher p);
	void setPublishRate(double rate);
	void setMaxOldWarnCount(int max);

	void spin(const ros::TimerEvent& e);

	void setEncoderLeftConversionConst(double ticks_to_m);
	void setEncoderRightConversionConst(double ticks_to_m);

protected:
	virtual void calculatePose();
	virtual bool checkUpdated();
	virtual void clearUpdated();

	double l_ticks_to_m,r_ticks_to_m;

	bool l_updated,r_updated;

	fmMsgs::encoder previous_left,previous_right;
	fmMsgs::encoder current_left,current_right;

	double x,y,theta;
	double vx,vy,vtheta;

	double max_time_diff;

private:
	void resetAll();



	ros::Publisher odom_pub;
	double pub_rate;

	int warn_count;
	int max_old_warn;


	tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::TransformStamped odom_trans;
	nav_msgs::Odometry odom;
};

#endif /* POSEEXTRACTORENCODERBASE_H_ */
