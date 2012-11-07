/*
 *  keyboardEventListener.cpp
 *
 *   Created on: 06/11/2012
 *       Author: Kent
 */

#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/Char.h>

int kfd = 0;
struct termios cooked, raw;

void listenForKeyEvents()
{
	char key = 0;
	std_msgs::Char msg;
	ros::NodeHandle globalNodeHandler;
	ros::Publisher keyPublisher = globalNodeHandler.advertise<std_msgs::Char>("/fmHMI/keyboardEventListener", 100);

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~(ICANON | ECHO);

	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Listening for keyboard events and publish these events...");

	while (ros::ok())
	{
		// Get events from keyboard
		if (read(kfd, &key, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		//	Publish keyboardEvent
		ROS_DEBUG("%c", key);
		msg.data = key;
		keyPublisher.publish(msg);
	}
}

void quit(int sig)
{
	puts("Quitting due to Ctrl-C...");

	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "keyboardEventListener");

	signal(SIGINT, quit);

	listenForKeyEvents();

	return 0;
}
