#ifndef REGION_GROWNING_SEGMENTATION_H_
#define REGION_GROWNING_SEGMENTATION_H_
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <ros/ros.h>
#include "types.h"

namespace cluster{
class region_growing{
    public:
    region_growing(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~region_growing() {};
    void cluster_vector(const VPointCloud::ConstPtr cloud, std::vector<pcl::PointIndices> &clusters);
};
}


#endif