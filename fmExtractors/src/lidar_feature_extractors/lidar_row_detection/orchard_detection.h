/*
 * ochard_detection.h
 *
 *  Created on: Jun 5, 2011
 *      Author: soeni05
 */

#ifndef OCHARD_DETECTION_H_
#define OCHARD_DETECTION_H_


#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Image.h"
#include "pcl/io/pcd_io.h"
#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"
#include "fmMsgs/row.h"

//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "visualization_msgs/Marker.h"

//#include <cv_bridge/CvBridge.h>


class OrchardDetector
{
private:

  IplImage* rawData_;
  IplImage* workingImg_;
  CvPoint point1, point2, point3, point4,point5;
  CvSeq* lines_;
  CvMemStorage* storage_;
  CvPoint intersection_;

  visualization_msgs::Marker marker_r;
  visualization_msgs::Marker marker_l;

  fmMsgs::row rows_;


  float    right_distance_;
  float    left_distance_;
  float    right_angle_;
  float	   left_angle_;

  int inrow_cnt;


  float 	rolling_mean_[100][4];

  sensor_msgs::Image image_;

  geometry_msgs::TwistStamped twist_msg_;

  void clearRawImage();
  int intersect(CvPoint p1, CvPoint p2, CvPoint p3, CvPoint p4);

public:


  ros::Publisher twist_pub;
  ros::Publisher pc_pub;
  ros::Publisher marker_r_pub;
  ros::Publisher marker_l_pub;

  ros::Publisher row_pub;

  ros::Subscriber laser_scan;

  laser_geometry::LaserProjection projector_;std::string image_topic_; //default output
  ros::Publisher image_pub_; //image message publisher
  OrchardDetector();
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_scan);

};

#endif /* OCHARD_DETECTION_H_ */
