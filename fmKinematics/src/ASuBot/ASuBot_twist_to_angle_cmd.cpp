/****************************************************************************
 #
 # Copyright (c) 2011 Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.
 #
 *****************************************************************************
 # File: ASuBot_fwd_kinematics_node.cpp
 # Purpose: ASuBot forward kinematics.
 # Project: Field Robot - Vehicle Interface Computer
 # Author: Søren Hundevadt Nielsen <shn@kbm.sdu.dk>
 # Created: Aug 23, 2011 Søren Hundevadt Nielsen, Source written
 ****************************************************************************/

#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "fmMsgs/steering_angle_cmd.h"

ros::Timer can_tx_timer;
fmMsgs::steering_angle_cmd aes25_msg;
ros::Publisher wheel_pub;

void twistmsgCallbackHandler(const geometry_msgs::TwistStampedConstPtr& twist_msg) {

	aes25_msg.header.stamp = ros::Time::now();

	const double V = 3; // velocity m/s
	const double L = 1.2; // distance between back and frontwheels

	double omega = twist_msg->twist.angular.z;

	aes25_msg.steering_angle = atan2(omega*L,V);

	wheel_pub.publish(aes25_msg);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "ASuBot_twist_to_angle_node");

	ros::NodeHandle nh("~");
	ros::NodeHandle n;

	std::string publisher_topic;
	std::string twist_subscriber_topic;

	std::string ASubot_wheel_publisher_topic;

	nh.param<std::string> ("steering_angle_publisher_topic", ASubot_wheel_publisher_topic,"/ASuBot_wheel_angle");
	nh.param<std::string> ("twist_subscriber_topic", twist_subscriber_topic,"/cmd_vel");

	wheel_pub = nh.advertise<fmMsgs::steering_angle_cmd> (ASubot_wheel_publisher_topic.c_str(),1,1);
	ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::TwistStamped> (twist_subscriber_topic.c_str(), 1, &twistmsgCallbackHandler);

	aes25_msg.header.stamp = ros::Time::now();

	const double V = 1; // velocity m/s
	const double L = 1.2; // distance between back and frontwheels

	double omega = 0;
	aes25_msg.steering_angle = atan2(omega*L,V);

	wheel_pub.publish(aes25_msg);

	ros::Rate r(10);
	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

