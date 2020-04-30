#include "region_growing_segmentation.h"
namespace cluster{
region_growing::region_growing(ros::NodeHandle nh, ros::NodeHandle pnh)
{}
void region_growing::cluster_vector(const VPointCloud::ConstPtr cloud, std::vector<pcl::PointIndices> &clusters)
{
  //设置查找方式－kdtree
  pcl::search::Search<VPoint>::Ptr tree = boost::shared_ptr<pcl::search::Search<VPoint> > (new pcl::search::KdTree<VPoint>);
  //计算点法向量
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<VPoint, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);             //brief Set the number of k nearest neighbors to use for the feature estimation
  normal_estimator.compute (*normals);
  //segmentation
  pcl::RegionGrowing<VPoint, pcl::Normal> reg;
  reg.setMinClusterSize (10);              //minimum cluster size
  reg.setMaxClusterSize (50000);           //maxmum cluster size
  reg.setSearchMethod (tree);              //设置搜索方法kdtree     
  reg.setNumberOfNeighbours (30);          //设置最近邻搜索阈值
  reg.setInputCloud (cloud);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (28.0 / 180.0 * M_PI);     //设置法线差值阈值,单位弧度
  reg.setCurvatureThreshold (1.0);                      //设置曲率阈值
  reg.extract (clusters);
}
}