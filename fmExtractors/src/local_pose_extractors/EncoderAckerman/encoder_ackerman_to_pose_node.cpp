#include <ros/ros.h>
#include <ros/console.h>

#include <fmMsgs/encoder.h>
#include <ros/subscriber.h>

#include <string>
#include "PoseExtractorEncoderBase.h"
#include "PoseExtractorEncoderAckermanWithAngle.h"
#include "PoseExtractorEncoderAckermanWithAngleFWE.h"

using namespace std;

int main(int argc, char** argv) {

	ros::init(argc, argv, "odometry_publisher");

	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	string publish_topic;
	string subscribe_enc_l;
	string subscribe_enc_r;
	string subscribe_enc_a;

	PoseExtractorEncoderBase* encoder_to_pose;
	ROS_INFO("Encoder ackerman to pose started!\n");

	double l_ticks_to_meter, r_ticks_to_meter, angle_ticks_to_rad,
			vehicle_length;
	bool use_angle, fwenc;
	double max_diff, rate;
	int max_warn,offset;
	ros::Publisher p;
	ros::Subscriber s1,s2,s3;
	ros::Timer t;

	nh.param<string>("publisher_topic", publish_topic, "/fmExtractors/odom");
	nh.param<string>("enc_r_subscriber_topic", subscribe_enc_r,
			"/fmSensors/encoder_right");
	nh.param<string>("enc_l_subscriber_topic", subscribe_enc_l,
			"/fmSensors/encoder_left");
	nh.param<string>("angle_subscriber_topic", subscribe_enc_a,
			"/fmSensors/encoder_angle");

	nh.param<double>("conv_ticks_to_meter_left", l_ticks_to_meter, 1.0);
	nh.param<double>("conv_ticks_to_meter_right", r_ticks_to_meter, 1.0);
	nh.param<double>("conv_ticks_to_rad_angle", angle_ticks_to_rad, 1.0);
	nh.param<bool>("use_angle_sensor", use_angle, false);
	nh.param<bool>("encoders_on_front_wheel", fwenc, false);
	nh.param<double>("Publish_rate", rate, 1.0);
	nh.param<double>("max_time_difference", max_diff, 1.0);
	nh.param<int>("max_old_odometry_warn_count",max_warn,5);
	nh.param<int>("angle_encoder_zero_offset",offset,0);

	nh.param<double>("distance_between_axles_in_meter", vehicle_length, 1.18);

	p = n.advertise<nav_msgs::Odometry>(publish_topic.c_str(), 25);

	if (use_angle) {
		if (fwenc) {
			encoder_to_pose = new PoseExtractorEncoderAckermanWithAngleFWE(p,
					rate, max_diff);
		} else {
			encoder_to_pose = new PoseExtractorEncoderAckermanWithAngle(p, rate,
					max_diff);
		}
		((PoseExtractorEncoderAckermanWithAngle*) encoder_to_pose)->setVehicleLength(
				vehicle_length);
		((PoseExtractorEncoderAckermanWithAngle*) encoder_to_pose)->setEncoderAngleConversionConst(
				angle_ticks_to_rad);

		s3 = nh.subscribe<fmMsgs::encoder>(subscribe_enc_a.c_str(), 25,
				&PoseExtractorEncoderAckermanWithAngle::processEncoderAngle,
				(PoseExtractorEncoderAckermanWithAngle*) encoder_to_pose);

		((PoseExtractorEncoderAckermanWithAngle*)encoder_to_pose)->angle_offset = offset;
	} else {
		encoder_to_pose = new PoseExtractorEncoderBase(p, rate, max_diff);
	}

	encoder_to_pose->setMaxOldWarnCount(max_warn);

	s1 = nh.subscribe<fmMsgs::encoder>(subscribe_enc_l.c_str(), 25,
			&PoseExtractorEncoderAckermanWithAngle::processEncoderLeft,
			encoder_to_pose);
	s2 = nh.subscribe<fmMsgs::encoder>(subscribe_enc_r.c_str(), 25,
			&PoseExtractorEncoderAckermanWithAngle::processEncoderRight,
			encoder_to_pose);

	encoder_to_pose->setEncoderLeftConversionConst(l_ticks_to_meter);
	encoder_to_pose->setEncoderRightConversionConst(r_ticks_to_meter);

	// connect spin function
	t = nh.createTimer(ros::Duration(1.0 / rate), &PoseExtractorEncoderBase::spin,
			encoder_to_pose);

	ros::spin();

	return 0;
}
