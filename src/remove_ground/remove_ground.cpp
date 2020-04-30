#include "remove_ground.h"

namespace remove_ground
{
Remove_ground::Remove_ground(ros::NodeHandle nh, ros::NodeHandle pnh)
{ 
    pnh.param("KWindowRows", KWindowRows, 3);
    pnh.param("KWindowCols", KWindowCols, 3); 
    pnh.param("grid_width", grid_width_, 0.1); 
    pnh.param("grid_length", grid_length_, 0.2);
    pnh.param("max_ground_height_threshold", max_ground_height_threshold_, 1.0);
    pnh.param("covariance_threshold_", covariance_threshold_, 0.05);
    pnh.param("height_of_hole", height_of_hole_, -0.1);
    pnh.param("min_dis_neighbour_points_threshold_", min_dis_neighbour_points_threshold_, 0.5);
    pnh.param("min_num_neighbour_points_threshold_", min_num_neighbour_points_threshold_, 2);
}
/*
void Remove_ground::GetFilteredPclWithWindow(VPointCloud::Ptr &pcl_in_)
{
    if((pcl_in_->points.size() > KWindowRows * KWindowCols) && (pcl_in_->width > KWindowCols) && (pcl_in_->height > KWindowRows))
    {
        const size_t cols = pcl_in_->height - KWindowRows;
        const size_t rows = pcl_in_->width - KWindowCols;
        double cov[cols][rows];
        for (size_t i = 0; i < pcl_in_->height; ++i) 
        {
            for (size_t j = 0; j < pcl_in_->width; ++j) 
            {
                pcl_in_->at(j, i).ring = 0;
            }
        }
        for (size_t i = 0; i < cols; ++i) 
        {
            for (size_t j = 0; j < rows; ++j) 
            {
                cov[i][j] = 0;
                size_t count = 0;
                double sum_z = 0;
                double sum_zz = 0;
                double max_z = -std::numeric_limits<float>::max();
                double max_z_m;
                double max_z_n;
                double min_z = std::numeric_limits<float>::max();
                double min_z_m;
                double min_z_n;
                for (size_t m = 0; m < KWindowRows; ++m) 
                {
                    for (size_t n = 0; n < KWindowCols; ++n) 
                    {
                        if (std::isfinite(pcl_in_->at(j + n, i + m).x)) 
                        {
                            sum_z += pcl_in_->at(j + n, i + m).z;
                            sum_zz += pcl_in_->at(j + n, i + m).z * pcl_in_->at(j + n, i + m).z;
                            ++count;

                            if (pcl_in_->at(j + n, i + m).z > max_z) {
                                max_z = pcl_in_->at(j + n, i + m).z;
                                max_z_m = m;
                                max_z_n = n;
                            }
                            if (pcl_in_->at(j + n, i + m).z < min_z) {
                                min_z = pcl_in_->at(j + n, i + m).z;
                                min_z_m = m;
                                min_z_n = n;
                            }
                        }
                    }
                }
                if (count > 0) 
                {
                    double mean_z = sum_z / count;
                    cov[i][j] = sum_zz / count - mean_z * mean_z;
                    if (cov[i][j] > covariance_threshold_ * covariance_threshold_) 
                    {
                        pcl_in_->at(j + max_z_n, i + max_z_m).ring = 1;
                        if (mean_z < height_of_hole_) 
                        {
                            pcl_in_->at(j + min_z_n, i + min_z_m).ring = 1;
                        }
                    }
                }
            }
        }
        VPoint p_nan;
        p_nan.x = NAN;
        p_nan.y = NAN;
        p_nan.z = NAN;
        for (size_t i = 0; i < pcl_in_->height; ++i) 
        {
            for (size_t j = 0; j < pcl_in_->width; ++j) 
            {
                if ((pcl_in_->at(j, i).ring > 0) && (std::isfinite(pcl_in_->at(j, i).x))) {}
                else  pcl_in_->at(j, i) = p_nan;
            }
        }
    }
}
VPointCloud::Ptr Remove_ground::GetFilteredPclWithGrid(VPointCloud::Ptr raw_pcl_)
{
   if(raw_pcl_->points.size() > 0)
   {
        float x_min = std::numeric_limits<float>::max();
        float y_min = std::numeric_limits<float>::max();
        float x_max = -std::numeric_limits<float>::max();
        float y_max = -std::numeric_limits<float>::max();
        for (const VPoint &p : raw_pcl_->points) 
        {
            x_max = std::isfinite(p.x) ? std::max(p.x, x_max) : x_max;
            y_max = std::isfinite(p.x) ? std::max(p.y, y_max) : y_max;
            x_min = std::isfinite(p.x) ? std::min(p.x, x_min) : x_min;
            y_min = std::isfinite(p.x) ? std::min(p.y, y_min) : y_min;
        }
        if ((x_max > x_min) && (y_max > y_min))
        {
            const size_t grid_cols = ceil((x_max - x_min) / grid_length_);
            const size_t grid_rows = ceil((y_max - y_min) / grid_width_);
            double height[grid_cols][grid_rows];
            double height2[grid_cols][grid_rows];
            bool is_obstacle[grid_cols][grid_rows];
            size_t points_count[grid_cols][grid_rows];
            // ROS_INFO("initialize all array:%zu,%zu", grid_cols, grid_rows);
            for (size_t i = 0; i < grid_cols; ++i)
            {
                for (size_t j = 0; j < grid_rows; ++j)
                {
                    height[i][j] = 0;
                    height2[i][j] = 0;
                    points_count[i][j] = 0;
                    is_obstacle[i][j] = false;
                }
            }
            // ROS_INFO("statistic height");
            VPointCloud::Ptr out(new VPointCloud);
            for (VPoint &p : raw_pcl_->points)
            {
                if (p.z < max_ground_height_threshold_)
                {
                    size_t j = floor((p.y - y_min) / grid_width_);
                    p.intensity = j;
                    p.ring = (floor((p.x - x_min) / grid_length_));
                    height[p.ring][j] += p.z;
                    height2[p.ring][j] += p.z * p.z;
                    ++points_count[p.ring][j];
                }
                else
                {
                    out->push_back(p);
                }
            }
            // ROS_INFO("judging");
            for (size_t i = 0; i < grid_cols; ++i)
            {
                for (size_t j = 0; j < grid_rows; ++j)
                {
                    if (points_count[i][j] > 3)
                    {
                        if (height2[i][j]*points_count[i][j] - height[i][j]*height[i][j] > covariance_threshold_ * covariance_threshold_ * points_count[i][j]*points_count[i][j]) 
                        {
                            is_obstacle[i][j] = true;
                        }
                    }
                }
            }
            // ROS_INFO("collecting points");
            for (const VPoint &p : raw_pcl_->points)
            {
                size_t i = static_cast<size_t>(p.ring);
                size_t j = static_cast<size_t>(p.intensity);
                if ((i < grid_cols) && (j < grid_rows) && (is_obstacle[i][j]))
                {
                    out->push_back(p);
                }
            }
            out->height = 1;
            return out;
        } 
        else 
        {
            return nullptr;
        }
    } 
    else 
    {
        return nullptr;
    }
}*/
VPointCloud::Ptr Remove_ground::GetFilteredWithNearestNeighborVariance(VPointCloud::Ptr raw_pcl_)
{
    const size_t pcl_size = raw_pcl_->points.size();
    size_t obstacle_flag[pcl_size];
    for(size_t i = 0; i < pcl_size; i++) obstacle_flag[i] = 0;
    if(pcl_size > 0)
    {
        for(size_t i = 0; i < pcl_size; i++)
        {
            const VPoint &p_1 = raw_pcl_->points[i];
            double sum_z = p_1.z;
            double sum_zz = X2(p_1.z);
            size_t neighbor_number = 0;
            for(size_t j = 0; j < pcl_size; j++)
            {
                if(j == i) continue;
                else
                {
                    const VPoint &p_2 = raw_pcl_->points[j];
                    double dis2 = X2(p_1.x - p_2.x) + X2(p_1.y - p_2.y) + X2(p_1.z - p_2.z);
                    if (dis2 < X2(min_dis_neighbour_points_threshold_))
                    {
                        sum_z += p_2.z;
                        sum_zz += X2(p_2.z);
                        neighbor_number++;
                    }
                }
                
            }
            if(neighbor_number > 0)
            {
                double mean_z = sum_z / neighbor_number;
                double cov = sum_zz / neighbor_number - mean_z * mean_z;
                if (cov > covariance_threshold_ * covariance_threshold_) obstacle_flag[i] = 1;
            }
        }
        VPointCloud::Ptr out(new VPointCloud);
        for(size_t k = 0; k < pcl_size; k++)
        {
            VPoint &p = raw_pcl_->points[k];
            if (p.z < max_ground_height_threshold_)
            {
                if(obstacle_flag[k] == 1) out->push_back(p);
            }
            else
            {
                out->push_back(p);
            }
        }
        return out;
    }
    return nullptr;
}
VPointCloud::Ptr Remove_ground::GetFilteredWithNearestNeighborVariance_new(VPointCloud::Ptr raw_pcl_)
{
  if(raw_pcl_->points.size() > 0){
    //step1:将z坐标低于某个值的点全部切除
    VPointCloud::Ptr obs(new VPointCloud);
    VPointCloud::Ptr suspect_ground(new VPointCloud);
    for(auto p : raw_pcl_->points){
      if(p.z > max_ground_height_threshold_){
        obs->points.push_back(p);
      }else{
        suspect_ground->points.push_back(p);
      }
    }
    //step2:建立kdtree，对每个点进行k半径查询，并计算方差,小于特定值的话存进非地面点云指针内
    pcl::KdTreeFLANN<VPoint> kdtree;
    kdtree.setInputCloud (suspect_ground);
    int num = suspect_ground->points.size();
    for(auto p : suspect_ground->points){
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      int neighbor_size = kdtree.radiusSearch(p, min_dis_neighbour_points_threshold_, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      if(neighbor_size > 0){
        double sum_z = p.z;
        double sum_zz = X2(p.z);
        for(int i=0; i<neighbor_size; i++){
          sum_z += suspect_ground->points[pointIdxRadiusSearch[i]].z;
          sum_zz += X2(suspect_ground->points[pointIdxRadiusSearch[i]].z);
        }
        double mean_z = sum_z / neighbor_size;
        double cov = sum_zz / neighbor_size - mean_z * mean_z;
        if(cov > covariance_threshold_ * covariance_threshold_) obs->points.push_back(p);
      }else{//k近邻不存在的话，为离散噪声

      }
    }

    return obs;
  }else{
    return nullptr;
  }
}
void Remove_ground::RemoveIsolatedPoints(VPointCloud::Ptr &raw_pcl_) 
{
  if (raw_pcl_->width * raw_pcl_->height > 0) {
    const size_t pcl_size = raw_pcl_->width * raw_pcl_->height;
    size_t neighbour_count[pcl_size];
    for (size_t i = 0; i < pcl_size; ++i) {
      neighbour_count[i] = 0;
    }
    for (size_t i = 0; i < pcl_size; ++i) {
      VPoint &p_1 = raw_pcl_->points[i];
      if (neighbour_count[i] < min_num_neighbour_points_threshold_) {
        //for (size_t j = pcl_size; j > i; --j) {
        for (size_t j = i + 1; j < pcl_size; ++j) {
          const VPoint &p_2 = raw_pcl_->points[j];
          double dis2 = X2(p_1.x - p_2.x) + X2(p_1.y - p_2.y) + X2(p_1.z - p_2.z);
          if (dis2 < X2(min_dis_neighbour_points_threshold_)) {
            ++neighbour_count[i];
            ++neighbour_count[j];
            if (neighbour_count[i] >= min_num_neighbour_points_threshold_) {
              break;
            }
          }
        }
      }
    }
    for (size_t i = 0; i < pcl_size; ++i) {
      if (neighbour_count[i] < min_num_neighbour_points_threshold_) {
        VPoint &p = raw_pcl_->points[i];
        //if(p.x < 4.0)
        //{
            p.x = NAN;
            p.y = NAN;
            p.z = NAN;
        //}
      }
    }
  }
}
}//namespace remove_ground