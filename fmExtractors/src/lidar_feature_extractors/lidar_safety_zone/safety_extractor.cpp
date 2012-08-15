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
# File: ransac_extractor.cpp
# Purpose:
# Project:
# Author: Søren Hundevadt Nielsen <soeni05@gmail.com>
# Created: Jun 25, 2012 Søren Hundevadt Nielsen, Source written
****************************************************************************/


#include "safety_extractor.h"
#include "math.h"

SafetyExtractor::SafetyExtractor(){


	marker_rg.header.stamp = ros::Time::now();
	marker_rg.ns = "safety";
	marker_rg.pose.orientation.w = 1.0;
	marker_rg.type = visualization_msgs::Marker::CUBE;

	marker_rg.id = 1;

	marker_rg.pose.position.y = 0;
	marker_rg.text = "marker text";

	marker_rg.scale.x = 0.1;
	marker_rg.scale.y = 0.1;
	marker_rg.scale.z = 0.1;

	marker_rg.color.b = 0;
	marker_rg.color.r = 0;
	marker_rg.color.g = 0;
	marker_rg.color.a = 0.6;



}



double SafetyExtractor::getdistance(Eigen::Vector2d p0, Eigen::Vector2d p1)
{
	return (sqrt(pow((double)p0[0]-(double)p1[0],2)+pow((double)p0[1]-(double)p1[1],2)));
}

void SafetyExtractor::processPointCloudInRow(sensor_msgs::PointCloud& pointcloud,int& yellowcnt,int& redcnt)
{
	Eigen::Vector2d p0(headland_box_min_x,-headland_box_lim_y);
	Eigen::Vector2d p1(headland_box_max_x,headland_box_lim_y);
	Eigen::Vector2d p2(0,-headland_box_lim_y);
	Eigen::Vector2d p3(headland_box_min_x,headland_box_lim_y);

	redcnt = 0;
	yellowcnt = 0;
	for(size_t i = 0; i < pointcloud.points.size(); i++){
		if(pointcloud.points[i].y > p0(1) && pointcloud.points[i].y < p1(1) ){
			if(pointcloud.points[i].x > p0(0) && pointcloud.points[i].x < p1(0) ){
				yellowcnt++;
			}
		}
		if(pointcloud.points[i].y > p2(1) && pointcloud.points[i].y < p3(1) ){
					if(pointcloud.points[i].x > p2(0) && pointcloud.points[i].x < p3(0) ){
						redcnt++;
					}
				}

	}
}

void SafetyExtractor::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_scan) {

	tf::TransformListener listener_;
	sensor_msgs::PointCloud cloud;
	sensor_msgs::PointCloud cloud_filtered;


    try
    {
    	projector_.projectLaser(*laser_scan, cloud,this->laser_scan_max_distance);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    cloud_filtered.header = cloud.header;
    cloud_filtered.channels = cloud.channels;

    for(size_t i = 0; i < cloud.points.size();i++){
    	if (cloud.points[i].x >0.10 && cloud.points[i].x < this->look_x && fabs(cloud.points[i].y) < this->look_y ) {
    		cloud_filtered.points.push_back(cloud.points[i]);
		}
    }
    int rc,yc;
    if(cloud_filtered.points.size() > 10){
    	processPointCloudInRow(cloud_filtered,yc,rc);

    	marker_rg.header.stamp = ros::Time::now();
    	marker_rg.header.frame_id = frame_id;

    	zone_msg.header.stamp =ros::Time::now();


    	if(yc > num_yellow){
    		marker_rg.id = 1;
    		marker_rg.color.g = 1;
    		marker_rg.color.b = 1;
    		marker_rg.color.r = 0;
    		marker_rg.color.a = 0.8;
    		zone_msg.yellow_activated = true;
    	}else{
    		marker_rg.id = 1;
    		marker_rg.color.g = 1;
    		marker_rg.color.b = 1;
    		marker_rg.color.r = 0;
    		marker_rg.color.a = 0.3;
    		zone_msg.yellow_activated = false;
    	}

    	marker_rg.scale.x = (headland_box_max_x - headland_box_min_x);
    	marker_rg.scale.y = headland_box_lim_y*2;
    	marker_rg.scale.z = 0.1;

    	marker_rg.pose.position.x = headland_box_min_x + (headland_box_max_x - headland_box_min_x)/2;
    	marker_publisher.publish(marker_rg);


    	if(rc > num_red){
    		marker_rg.id = 2;
    		marker_rg.color.r = 1;
    		marker_rg.color.g = 0;
    		marker_rg.color.b = 0;
    		marker_rg.color.a = 0.8;
    		zone_msg.red_activated = true;

    	}else{
    		marker_rg.color.r = 1;
    		marker_rg.color.g = 0;
			marker_rg.color.b = 0;
    		marker_rg.id = 2;
    		marker_rg.color.a = 0.3;
    		zone_msg.red_activated = false;
    	}

    	marker_rg.scale.x = headland_box_min_x;
    	marker_rg.scale.y = headland_box_lim_y*2;
    	marker_rg.scale.z = 0.1;

    	marker_rg.pose.position.x =  headland_box_min_x/2;
    	marker_publisher.publish(marker_rg);

    	safety_publisher.publish(zone_msg);
    }
	pc_publisher.publish(cloud_filtered);
}
