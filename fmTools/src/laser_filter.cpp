/****************************************************************************
# Ransac row extractor
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
# File: ransac_row_node.cpp
# Purpose:
# Project:
# Author: Søren Hundevadt Nielsen <soeni05@gmail.com>
# Created: Jun 25, 2012 Søren Hundevadt Nielsen, Source written
****************************************************************************/

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>

ros::Publisher pub;
double max,min;


void laser_in_cb(const sensor_msgs::LaserScanConstPtr& msg)
{
	bool first = true;
	double ma,mi;
	sensor_msgs::LaserScan m_out;
	for (int i =0 ; i< msg->ranges.size();i++)
	{
		double angle = i*msg->angle_increment + msg->angle_min;
		if(angle > min && angle < max)
		{
			if(first == true)
			{
				mi = angle;
				first = false;
			}
			ma = angle;
			m_out.ranges.push_back(msg->ranges[i]);
			m_out.intensities.push_back(0.0);
		}
	}

	m_out.header = msg->header;
	m_out.angle_increment = msg->angle_increment;
	m_out.angle_min = mi;
	m_out.angle_max = ma;
	m_out.scan_time = msg->scan_time;
	m_out.time_increment = msg->time_increment;
	m_out.range_max = msg->range_max;
	m_out.range_min = msg->range_min;

	pub.publish(m_out);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "row_extractor");

  ros::NodeHandle nh("~");
  ros::NodeHandle n;



  std::string in,out;

  nh.param<double>("max",max,M_PI/2);
  nh.param<double>("min",min,-M_PI/2);
  nh.param<std::string>("in", in, "/lrs/laser_msgs_1");
  nh.param<std::string>("out",out,"scan_filtered");

  pub = n.advertise<sensor_msgs::LaserScan>(out.c_str(),1,false);
  ros::Subscriber s1 = n.subscribe<sensor_msgs::LaserScan> (in.c_str(), 1, &laser_in_cb);

  ros::spin();

  return 0;
}
