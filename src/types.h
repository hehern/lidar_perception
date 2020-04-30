#ifndef OBSTACLE_DETECT_IFM_TYPES_H
#define OBSTACLE_DETECT_IFM_TYPES_H
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// struct PointXYZIR
// {
//     PCL_ADD_POINT4D;                    // quad-word XYZ
//     float    intensity;                 ///< laser intensity reading
//     uint16_t ring;                      ///< laser ring number
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
// } EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
//                                   (float, x, x)
//                                   (float, y, y)
//                                   (float, z, z)
//                                   (float, intensity, intensity)
//                                   (uint16_t, ring, ring))
typedef pcl::PointXYZI VPoint; 
typedef pcl::PointCloud<VPoint> VPointCloud;

using IPoint = pcl::PointXYZI;
using IPointCloud = pcl::PointCloud<IPoint>;

template <typename T>
  T X2(const T &x) {return x * x;}

#endif