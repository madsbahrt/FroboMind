#include "ros/ros.h"
#include <string.h>
#include <sstream>
#include "fmMsgs/nmea.h"
#include <geometry_msgs/TwistStamped.h>

class FroboMotor
{
public:
	FroboMotor()
	{

	};

	~FroboMotor()
	{

	};

	void on_vel_msg_left(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		msg_left = *msg;
	}

	void on_vel_msg_right(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		msg_right = *msg;
	}

	void on_timer(const ros::TimerEvent& e)
	{
		double left_vel = 0;
		double right_vel = 0;
		active = true;
		if((ros::Time::now() - msg_left.header.stamp).toSec() > timeout)
		{
			ROS_WARN_THROTTLE(1,"Time for left cmd_vel is out of date");
			active = false;
		}

		if((ros::Time::now() - msg_right.header.stamp).toSec() > timeout)
		{
			ROS_WARN_THROTTLE(1,"Time for right cmd_vel is out of date");
			active = false;
		}

		if(active)
		{
			left_vel = msg_left.twist.linear.x * 100;
			right_vel = msg_right.twist.linear.x * 100;

			//correct high velocities
			if ( left_vel > max_velocity )
				left_vel = max_velocity;
			else if ( left_vel < - max_velocity )
				left_vel = - max_velocity;

			if ( right_vel > max_velocity )
				right_vel = max_velocity;
			else if ( right_vel < - max_velocity )
				right_vel = - max_velocity;

			left_vel *= vel_to_motor_const;
			right_vel *= vel_to_motor_const;


		}

		//build message
		duty_message.header.stamp = ros::Time::now();
		duty_message.id = "RC";
		duty_message.type = "VEL";
		duty_message.data.push_back( boost::lexical_cast<std::string>( (int)left_vel ) );
		duty_message.data.push_back( boost::lexical_cast<std::string>( (int)right_vel ) );

		//publish message
		nmea_pub.publish(duty_message);
	}


	ros::Publisher nmea_pub;
	double vel_to_motor_const;

	double timeout;
	double max_velocity;




private:

	bool active;
	geometry_msgs::TwistStamped msg_left,msg_right;
	fmMsgs::nmea duty_message;

};

int main(int argc, char **argv) {

	ros::init(argc, argv, "frobit_motorcontroller");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	std::string vel_l,vel_r;
	std::string publish_topic_id;
	double wheel_diameter;
	double ticks_pr_round;
	double ms_in_between;
	double interval;

	ros::Subscriber s1,s2;

	FroboMotor frobit_motor;

	n.param<std::string>("frobit_velocity_sub_left", vel_l, "/fmDecision/twist");
	n.param<std::string>("frobit_velocity_sub_right", vel_r, "/fmDecision/twist");
	n.param<std::string>("frobit_nmea_pub", publish_topic_id, "/fmActuators/duty");

	n.param<double>("max_velocity", frobit_motor.max_velocity,255);
	n.param<double>("wheel_diameter", wheel_diameter, 0.01);
	n.param<double>("ticks_pr_round", ticks_pr_round, 360);
	n.param<double>("ms_in_between", ms_in_between, 100);
	n.param<double>("vel_publish_interval",interval,0.05);

	/* m/s -> ticks/entry
	 * 1 m/s = 1/(pi*wheel_diameter) rps = ticks_pr_round/(pi*wheel_diameter) ticks/sec = */
	frobit_motor.vel_to_motor_const = ( ticks_pr_round / ( 3.14 * wheel_diameter ) ) * ( 1 / ms_in_between );

	s1 = nh.subscribe<geometry_msgs::TwistStamped>(vel_l,2,&FroboMotor::on_vel_msg_left,&frobit_motor);
	s2 = nh.subscribe<geometry_msgs::TwistStamped>(vel_r,2,&FroboMotor::on_vel_msg_right,&frobit_motor);

	frobit_motor.nmea_pub = nh.advertise<fmMsgs::nmea>(publish_topic_id, 1);

	nh.createTimer(ros::Duration(interval),&FroboMotor::on_timer,&frobit_motor);

	ros::spin();
	return 0;
}
