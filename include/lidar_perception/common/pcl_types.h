/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PERCEPTION_COMMON_PCL_TYPES_H_
#define MODULES_PERCEPTION_COMMON_PCL_TYPES_H_

#include "pcl/common/time.h"
#include "pcl/common/transforms.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/impl/kdtree.hpp"

namespace apollo {
namespace perception {
namespace pcl_util {

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
typedef pcl::PointCloud<Point>::ConstPtr PointCloudConstPtr;

typedef Point PointD;
typedef ::pcl::PointCloud<PointD> PointDCloud;
typedef ::pcl::PointCloud<PointD>::Ptr PointDCloudPtr;
typedef ::pcl::PointCloud<PointD>::ConstPtr PointDCloudConstPtr;
}
}
}

#endif  // MODULES_PERCEPTION_COMMON_PCL_TYPES_H_
