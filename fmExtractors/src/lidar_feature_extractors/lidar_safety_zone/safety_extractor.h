#ifndef RansacRowExtractor_H_
#define RansacRowExtractor_H_

#define EIGEN2_SUPPORT

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "laser_geometry/laser_geometry.h"

#include "ros/ros.h"
#include "fmMsgs/lidar_safety_zone.h"

#include "tf/transform_listener.h"

#include "visualization_msgs/Marker.h"

#include <Eigen/Geometry>
#include <Eigen/LeastSquares>



class SafetyExtractor
{
private:

  laser_geometry::LaserProjection projector_;

  visualization_msgs::Marker marker_rg;
  fmMsgs::lidar_safety_zone zone_msg;

  void processPointCloudInRow(sensor_msgs::PointCloud& pointcloud,int& yellowcnt,int& redcnt);

  double getdistance(Eigen::Vector2d p0,Eigen::Vector2d p1);
public:

  ros::Subscriber ls_subscriber;
  ros::Publisher pc_publisher;
  ros::Publisher safety_publisher;
  ros::Publisher marker_publisher;

  std::string frame_id;

  double headland_box_min_x,headland_box_max_x,headland_box_lim_y;
  double num_yellow,num_red;
  double look_x,look_y;
  double laser_scan_max_distance;

  SafetyExtractor();
  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_scan);

};

#endif /* RansacRowExtractor_H_ */
