#include <ros/ros.h>
#include "std_msgs/UInt32.h"

double spray_time;
bool spray_l,spray_r;
ros::Time l_start,r_start;


void on_spray(const std_msgs::UInt32::ConstPtr& msg)
{
	if(msg->data == 1)
	{
		spray_l = true;
	}
	if(msg->data == 2)
	{
		spray_r = true;
	}
	if(msg->data == 0)
	{
		spray_l = spray_r = false;
	}
}

void on_timer(const ros::TimerEvent& e)
{
	if(spray_l)
	{
		l_start = ros::Time::now();
		spray_l = false;
		// enable sprayer
	}

	if(spray_r)
	{
		r_start = ros::Time::now();
		spray_r = false;
	}

	if((ros::Time::now() - l_start) > ros::Duration(spray_time))
	{
		// disable left
	}

	if((ros::Time::now() - r_start) > ros::Duration(spray_time))
	{
		// disable right
	}

}

int main(int argc, char **argv)
{

	//Initialize ros usage
	ros::init(argc, argv, "sprayer_implement");

	//Create Nodehandlers
	ros::NodeHandle nn("");
	ros::NodeHandle nh("~");

	std::string sub;

	nh.param<std::string>("cmd_subsscriber_topic",sub,"/fmControllers/spray");

	ros::Subscriber s = nn.subscribe(sub, 10, &on_spray);

	ros::Timer t = nh.createTimer(ros::Duration(0.05),&on_timer);


	t.start();
	//Handle callbacks
	ros::spin();

}
