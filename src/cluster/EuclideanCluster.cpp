#include "EuclideanCluster.h"
namespace cluster{
euclidean_cluster::euclidean_cluster(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  pnh.param("clusterTolerance", clusterTolerance_, 0.02);
  pnh.param("minClusterSize", minClusterSize_, 5);
  pnh.param("maxClusterSize", maxClusterSize_, 300);
  // std::cout << "clusterTolerance_ = " << clusterTolerance_ << std::endl;
}
void euclidean_cluster::cluster_vector(const VPointCloud::ConstPtr cloud, std::vector<pcl::PointIndices> &clusters)
{
  //设置查找方式－kdtree
  pcl::search::Search<VPoint>::Ptr tree = boost::shared_ptr<pcl::search::Search<VPoint> > (new pcl::search::KdTree<VPoint>);

  pcl::EuclideanClusterExtraction<VPoint> ec;
  ec.setClusterTolerance (clusterTolerance_);// 2cm
  ec.setMinClusterSize (minClusterSize_); //100
  ec.setMaxClusterSize (maxClusterSize_);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (clusters);
}
}