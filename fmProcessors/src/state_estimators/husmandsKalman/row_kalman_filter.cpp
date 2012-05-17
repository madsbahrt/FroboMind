/*
 * row_kalman_filter.cpp
 *
 *  Created on: Apr 13, 2012
 *      Author: morl
 */
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <fmMsgs/gyroscope.h>
#include <fmMsgs/claas_row_cam.h>

// for definition of PI
#include <math.h>


class RowFilter
{
public:
	RowFilter()
	{
		estimated_angle = 0.0;
		estimated_variance = 0.0;

		system_angle = 0.0;
		system_variance = 0.0;

		measurement_angle = 0.0;
		measurement_variance = 0.0;

		kalman_gain = 0.0;

		gyro_variance = pow(2,2)*M_PI/180;
		quality_to_variance = 1.75;

		imu_initialised = false;
	}

	virtual ~RowFilter()
	{

	}

	void processIMUSystemUpdate(const sensor_msgs::ImuConstPtr& imu_msg)
	{
		if(!imu_initialised)
		{
			imu_initialised = true;
		}
		else
		{
			double ang = tf::getYaw(imu_msg->orientation) - tf::getYaw(prev_imu_msg.orientation);
			ROS_DEBUG_NAMED("kalman_estimate","performing system update with: %.4f and %.4f",ang,gyro_variance);
			system_update(ang,gyro_variance);
			ROS_DEBUG_NAMED("kalman_estimate","Estimate after system update: %.4f %.4f",estimated_angle,estimated_variance);
		}
		prev_imu_msg = *imu_msg;

	}

	void processRowMeasurementUpdate(const fmMsgs::claas_row_camConstPtr& row_msg)
	{
		double variance = (256 - row_msg->quality)*quality_to_variance;
		double angle_rad = row_msg->heading * M_PI/180.0;

		row_offset = row_msg->offset;

		ROS_DEBUG_NAMED("kalman_estimate","performing measurement update with: %.4f and %.4f",angle_rad,variance);
		measurement_update(angle_rad,variance);
		ROS_DEBUG_NAMED("kalman_estimate","Estimate after measurement: %.4f %.4f",estimated_angle,estimated_variance);

	}

	double gyro_variance;
	double quality_to_variance;
	ros::Publisher row_est_pub;


private:

	void system_update(double angle,double variance)
	{
		// make new angle prediction
		system_angle = estimated_angle + angle;
		system_variance = estimated_variance + variance;

		// correct for angle "overflow"
		correct_angle(system_angle);

		// update estimate with prediction
		estimated_angle = system_angle;
		estimated_variance = system_variance;
	}

	void measurement_update(double angle, double variance)
	{
		// update estimate with measurement
		kalman_gain = system_variance / (system_variance + variance);
		measurement_angle = system_angle + kalman_gain * (angle - system_angle);
		measurement_variance = system_variance * (1 - kalman_gain);

		correct_angle(measurement_angle);

		estimated_angle = measurement_angle;
		estimated_variance = measurement_variance;



		msg.header.stamp = ros::Time::now();
		msg.header.seq++;

		msg.heading = estimated_angle/M_PI * 180;
		msg.offset = row_offset;
		msg.quality = estimated_variance;

		row_est_pub.publish(msg);
	}

	void correct_angle(double& angle)
	{
		if(angle >= M_PI)
		{
			angle -= 2*M_PI;
		}
		else if (angle <= -M_PI)
		{
			angle += 2*M_PI;
		}
	}

	/*
	 * Kalman filter variables
	 * */
	double estimated_angle;
	double estimated_variance;

	double system_angle;
	double system_variance;

	double measurement_angle;
	double measurement_variance;

	double kalman_gain;

	/*
	 * other variables
	 * */
	double row_offset; // saved from row message
	fmMsgs::claas_row_cam msg;
	sensor_msgs::Imu prev_imu_msg;

	bool imu_initialised;



};

int main(int argc, char **argv)
{


	ros::init(argc, argv, "row_kalman_filter");

	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	std::string gyro_sub_top;
	std::string row_sub_top;
	std::string row_est_pub_top;

	ros::Subscriber imu_sub;
	ros::Subscriber row_sub;

	RowFilter filter;

	nh.param<std::string>("imu_subscriber_topic", gyro_sub_top, "/fmSensors/IMU");
	nh.param<std::string>("row_subscriber_topic", row_sub_top, "/fmSensors/row");
	nh.param<std::string>("row_estimate_publisher_topic", row_est_pub_top,"/fmProcessors/row_estimate");
	nh.param<double>("imu_variance",filter.gyro_variance,pow(2,2)*M_PI/180);
	nh.param<double>("row_quality_to_variance",filter.quality_to_variance,1.75);

	imu_sub = nh.subscribe<sensor_msgs::Imu>(gyro_sub_top.c_str(),1, &RowFilter::processIMUSystemUpdate,&filter);
	row_sub = nh.subscribe<fmMsgs::claas_row_cam>(row_sub_top.c_str(),1, &RowFilter::processRowMeasurementUpdate,&filter);

	filter.row_est_pub = nh.advertise<fmMsgs::claas_row_cam>(row_est_pub_top.c_str(),1);

	ros::spin();
}
