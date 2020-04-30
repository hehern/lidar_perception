#ifndef REMOVE_GROUND_H
#define REMOVE_GROUND_H

#include <cmath>
#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/point_cloud.h>
#include "types.h"
namespace remove_ground
{
class Remove_ground
{
public:
  Remove_ground(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~Remove_ground(){};
  void GetFilteredPclWithWindow(VPointCloud::Ptr &pcl_in_);
  void RemoveIsolatedPoints(VPointCloud::Ptr &raw_pcl_);
  VPointCloud::Ptr GetFilteredPclWithGrid(VPointCloud::Ptr raw_pcl_);
  VPointCloud::Ptr GetFilteredWithNearestNeighborVariance(VPointCloud::Ptr raw_pcl_);
  VPointCloud::Ptr GetFilteredWithNearestNeighborVariance_new(VPointCloud::Ptr raw_pcl_);

private:
  int KWindowRows;
  int KWindowCols;
  double height_of_hole_;
  double covariance_threshold_;
  double grid_length_;  //grid size in x
  double grid_width_;   //grid size in y
  double max_ground_height_threshold_;
  double min_dis_neighbour_points_threshold_;
  int min_num_neighbour_points_threshold_;
  template <typename T>
  T X2(const T &x) {return x * x;}
};
}//namespace remove_ground
#endif