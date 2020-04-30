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

#include "hm_tracker.h"

#include <map>
#include <numeric>
#include "lidar_perception/common/geometry_util.h"
#include "feature_descriptor.h"
#include "hungarian_matcher.h"
#include "kalman_filter.h"
#include "track_object_distance.h"

namespace apollo {
namespace perception {

HmObjectTracker::HmObjectTracker(ros::NodeHandle nh, ros::NodeHandle pnh) {
  // Initialize tracker's configs
  pnh.param("name", config_.name, std::string("HmObjectTracker"));
  pnh.param("version", config_.version, std::string("1.1.0"));
  pnh.param("matcher_method", config_.matcher_method,std::string("HUNGARIAN_MATCHER"));
  pnh.param("filter_method", config_.filter_method, std::string("KALMAN_FILTER"));
  pnh.param("track_cached_history_size_maximum", config_.track_cached_history_size_maximum, 5);
  pnh.param("track_consecutive_invisible_maximum", config_.track_consecutive_invisible_maximum, 2);
  pnh.param("track_visible_ratio_minimum", config_.track_visible_ratio_minimum, 0.6);
  pnh.param("collect_age_minimum", config_.collect_age_minimum, 0);
  pnh.param("collect_consecutive_invisible_maximum", config_.collect_consecutive_invisible_maximum, 0);
  pnh.param("acceleration_noise_maximum", config_.acceleration_noise_maximum, 5);
  pnh.param("speed_noise_maximum", config_.speed_noise_maximum, 0.4); 
  pnh.param("match_distance_maximum", config_.match_distance_maximum, 4.0);
  pnh.param("location_distance_weight", config_.location_distance_weight, 0.6);
  pnh.param("direction_distance_weight", config_.direction_distance_weight, 0.2);
  pnh.param("bbox_size_distance_weight", config_.bbox_size_distance_weight, 0.1);
  pnh.param("point_num_distance_weight", config_.point_num_distance_weight, 0.1);
  pnh.param("histogram_distance_weight", config_.histogram_distance_weight, 0.5);
  pnh.param("histogram_bin_size", config_.histogram_bin_size, 10);
  pnh.param("use_adaptive", config_.use_adaptive, true);
  pnh.param("measurement_noise", config_.measurement_noise, 0.4);
  pnh.param("naminitial_velocity_noisee", config_.initial_velocity_noise, 5);
  pnh.param("xy_propagation_noise", config_.xy_propagation_noise, 10);
  pnh.param("z_propagation_noise", config_.z_propagation_noise, 10);
  pnh.param("breakdown_threshold_maximum", config_.breakdown_threshold_maximum, 10.0);

  // std::cout << "config_: = " << config_.track_cached_history_size_maximum << std::endl;
  // A. Basic tracker setup
  matcher_.reset(new HungarianMatcher());
  // B. Matcher setup
  // load match distance maximum
  if (!HungarianMatcher::SetMatchDistanceMaximum(config_.match_distance_maximum)) {
    std::cout << "Failed to set match distance maximum! " << config_.name << std::endl;
  }
  // load location distance weight
  if (!TrackObjectDistance::SetLocationDistanceWeight(
          config_.location_distance_weight)) {
    std::cout << "Failed to set location distance weight! " << config_.name << std::endl;
  }
  // load direction distance weight
  if (!TrackObjectDistance::SetDirectionDistanceWeight(
          config_.direction_distance_weight)) {
    std::cout << "Failed to set direction distance weight! " << config_.name << std::endl;
  }
  // load bbox size distance weight
  if (!TrackObjectDistance::SetBboxSizeDistanceWeight(
          config_.bbox_size_distance_weight)) {
    std::cout << "Failed to set bbox size distance weight! " << config_.name << std::endl;
  }
  // load point num distance weight
  if (!TrackObjectDistance::SetPointNumDistanceWeight(
          config_.point_num_distance_weight)) {
    std::cout << "Failed to set point num distance weight! " << config_.name << std::endl;
  }
  // load histogram distance weight
  if (!TrackObjectDistance::SetHistogramDistanceWeight(
          config_.histogram_distance_weight)) {
    std::cout << "Failed to set histogram distance weight! " << config_.name << std::endl;
  }
  use_histogram_for_match_ =
      config_.histogram_distance_weight > FLT_EPSILON ? true : false;
  if (config_.histogram_bin_size <= 0) {
    std::cout << "invalid histogram bin size of " << config_.name << std::endl;
  }
  // C. Filter setup
  double association_score_maximum = config_.match_distance_maximum;
  KalmanFilter::SetUseAdaptive(config_.use_adaptive);
  if (!KalmanFilter::SetAssociationScoreMaximum(association_score_maximum)) {
    std::cout << "Failed to set association score maximum! " << std::endl;
  }
  if (!KalmanFilter::InitParams(
          config_.measurement_noise, config_.initial_velocity_noise,
          config_.xy_propagation_noise, config_.z_propagation_noise)) {
    std::cout << "Failed to set params for kalman filter! " <<std::endl;
  }
  if (!KalmanFilter::SetBreakdownThresholdMaximum(
          config_.breakdown_threshold_maximum)) {
    std::cout << "Failed to set breakdown threshold maximum! " << std::endl;
  }

  first_time_ = true;
}
bool HmObjectTracker::Init()
{ 
}

const std::vector<ObjectTrackPtr>& HmObjectTracker::GetObjectTracks() const {
  return object_tracks_.GetTracks();
}

bool HmObjectTracker::Track(
    const std::vector<std::shared_ptr<Object>>& objects, double timestamp,
    const TrackerOptions& options,
    std::vector<std::shared_ptr<Object>>* tracked_objects) {
  
  // A. track setup
  if (tracked_objects == nullptr) return false;
  if (!valid_) {                                                    //如果程序是第一次运行就先进行初始化,如果不是第一次直接更新下面的就好
    valid_ = true;
    return InitializeTrack(objects, timestamp, options, tracked_objects);
  }
  Eigen::Matrix4d velo2world_pose = Eigen::Matrix4d::Identity();
  if (options.velodyne_trans != nullptr) {
    velo2world_pose = *(options.velodyne_trans);
  } else {
    std::cout << "Input velodyne_trans is null";
    return false;
  }
  //double time_diff = timestamp - time_stamp_;//时间戳相减
  double time_diff = 0.1;
  time_stamp_ = timestamp;
  //std::cout << "time_diff = " << time_diff <<std::endl;
  //std::cout << "time_stamp_ = " << time_stamp_ <<std::endl;

  // B. preprocessing
  // B.1 transform given pose to local one
  TransformPoseGlobal2Local(&velo2world_pose);
  //ADEBUG << "velo2local_pose\n" << velo2world_pose;
  // B.2 construct objects for tracking
  std::vector<std::shared_ptr<TrackedObject>> transformed_objects;
  ConstructTrackedObjects(objects, &transformed_objects, velo2world_pose,
                          options);
  // C. prediction
  std::vector<Eigen::VectorXf> tracks_predict;
  ComputeTracksPredict(&tracks_predict, time_diff);//计算每个跟踪对象的预测状态

  // D. match objects to tracks
  std::vector<std::pair<int, int>> assignments;
  std::vector<int> unassigned_objects;
  std::vector<int> unassigned_tracks;
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.GetTracks();
  if (matcher_ != nullptr) {
    matcher_->Match(&transformed_objects, tracks, tracks_predict, &assignments,
                    &unassigned_tracks, &unassigned_objects);
  } else {
    std::cout << "matcher_ is not initiated. Please call Init() function before "
              "other functions." << std::endl;
    return false;
  }
  std::cout << "multi-object-tracking: " << tracks.size() << "  "
         << assignments.size() << "  " << transformed_objects.size() << "  "
         << unassigned_objects.size() << "  " << time_diff << std::endl;
  // E. update tracks
  // E.1 update tracks with associated objects
  UpdateAssignedTracks(&tracks_predict, &transformed_objects, assignments,   //transformed_objects中存放已经匹配上的对象信息
                       time_diff);
  // E.2 update tracks without associated objects
  UpdateUnassignedTracks(tracks_predict, unassigned_tracks, time_diff);
  DeleteLostTracks();
  // E.3 create new tracks for objects without associated tracks
  CreateNewTracks(transformed_objects, unassigned_objects);
  // F. collect tracked results
  CollectTrackedResults(tracked_objects);
  return true;
}

bool HmObjectTracker::InitializeTrack(
    const std::vector<std::shared_ptr<Object>>& objects, const double timestamp,
    const TrackerOptions& options,
    std::vector<std::shared_ptr<Object>>* tracked_objects) {  
  // A. track setup
  Eigen::Matrix4d velo2world_pose = Eigen::Matrix4d::Identity();
  if (options.velodyne_trans != nullptr) {
    velo2world_pose = *(options.velodyne_trans);     //获取激光雷达坐标系到世界坐标系的转换矩阵
  } else {
    std::cout << "Input velodyne_trans is null";
    return false;
  }
  global_to_local_offset_ = Eigen::Vector3d(
      -velo2world_pose(0, 3), -velo2world_pose(1, 3), -velo2world_pose(2, 3));//lidar坐标系到世界坐标系的评议成分,即车坐标系到世界坐标系的转换矩阵

  // B. preprocessing
  // B.1 coordinate transformation
  TransformPoseGlobal2Local(&velo2world_pose);//雷达坐标系到车坐标系的变换矩阵
  //ADEBUG << "velo2local_pose\n" << velo2world_pose;
  // B.2 construct tracked objects
  std::vector<std::shared_ptr<TrackedObject>> transformed_objects;//构造TrackedObject对象,封装object类,并增加速度加速度等信息
  ConstructTrackedObjects(objects, &transformed_objects, velo2world_pose,
                          options);                         //transformed_objects内容填充,并计算SF特征 坐标转换   
                               
  // C. create tracks
  std::vector<int> unassigned_objects;
  unassigned_objects.resize(transformed_objects.size());
  std::iota(unassigned_objects.begin(), unassigned_objects.end(), 0);//iota批量递增
  CreateNewTracks(transformed_objects, unassigned_objects);
  time_stamp_ = timestamp;

  // D. collect tracked results
  CollectTrackedResults(tracked_objects);
  return true;
}

void HmObjectTracker::TransformPoseGlobal2Local(Eigen::Matrix4d* pose) {
  (*pose)(0, 3) += global_to_local_offset_(0);
  (*pose)(1, 3) += global_to_local_offset_(1);
  (*pose)(2, 3) += global_to_local_offset_(2);
}

void HmObjectTracker::ConstructTrackedObjects(
    const std::vector<std::shared_ptr<Object>>& objects,
    std::vector<std::shared_ptr<TrackedObject>>* tracked_objects,
    const Eigen::Matrix4d& pose, const TrackerOptions& options) {
  int num_objects = objects.size();
  tracked_objects->clear();
  tracked_objects->resize(num_objects);
  for (int i = 0; i < num_objects; ++i) {
    std::shared_ptr<Object> obj(new Object());
    obj->clone(*objects[i]);
    (*tracked_objects)[i].reset(new TrackedObject(obj));
    // Computing shape feature
    if (use_histogram_for_match_) {
      ComputeShapeFeatures(&((*tracked_objects)[i])); //计算SF特征
    }
    // Transforming all tracked objects
    TransformTrackedObject(&((*tracked_objects)[i]), pose);
    // Setting barycenter as anchor point of tracked objects
    Eigen::Vector3f anchor_point = (*tracked_objects)[i]->barycenter;
    (*tracked_objects)[i]->anchor_point = anchor_point;
    // Getting lane direction of tracked objects
    /*pcl_util::PointD query_pt;
    query_pt.x = anchor_point(0) - global_to_local_offset_(0);
    query_pt.y = anchor_point(1) - global_to_local_offset_(1);
    query_pt.z = anchor_point(2) - global_to_local_offset_(2);
    Eigen::Vector3d lane_dir;
    if (!options.hdmap_input->GetNearestLaneDirection(query_pt, &lane_dir)) {
      std::cout << "Failed to initialize the lane direction of tracked object!";
      // Set lane dir as host dir if query lane direction failed
      lane_dir = (pose * Eigen::Vector4d(1, 0, 0, 0)).head(3);
    }
    (*tracked_objects)[i]->lane_direction = lane_dir.cast<float>();*/
  }
}

void HmObjectTracker::ComputeShapeFeatures(
    std::shared_ptr<TrackedObject>* obj) {
  // Compute object's shape feature
  std::shared_ptr<Object>& temp_object = (*obj)->object_ptr;
  FeatureDescriptor fd(temp_object->cloud);
  fd.ComputeHistogram(config_.histogram_bin_size,
                      &temp_object->shape_features);
}

void HmObjectTracker::TransformTrackedObject(
    std::shared_ptr<TrackedObject>* obj, const Eigen::Matrix4d& pose) {
  // Transform tracked object with given pose
  TransformObject(&((*obj)->object_ptr), pose);
  // transform direction
  Eigen::Vector3f& dir = (*obj)->direction;       //dir和center为什么*两边???
  dir =
      (pose * Eigen::Vector4d(dir(0), dir(1), dir(2), 0)).head(3).cast<float>();
  // transform center
  Eigen::Vector3f& center = (*obj)->center;
  center = (pose * Eigen::Vector4d(center(0), center(1), center(2), 1))
               .head(3)
               .cast<float>();
  // transform barycenter
  Eigen::Vector3f& barycenter = (*obj)->barycenter; //barycenter什么时候填充啦?在这个结构体对象生成时候的构造函数里面
  barycenter =
      (pose * Eigen::Vector4d(barycenter(0), barycenter(1), barycenter(2), 1))
          .head(3)
          .cast<float>();
}

void HmObjectTracker::TransformObject(std::shared_ptr<Object>* obj,
                                      const Eigen::Matrix4d& pose) {
  // Transform object with given pose
  Eigen::Vector3d& dir = (*obj)->direction;
  //std::cout <<"object->direction = " << (*obj)->direction << std::endl;
  dir = (pose * Eigen::Vector4d(dir[0], dir[1], dir[2], 0)).head(3);
  // transform center
  Eigen::Vector3d& center = (*obj)->center;
  center = (pose * Eigen::Vector4d(center[0], center[1], center[2], 1)).head(3);
  // transform cloud & polygon
  TransformPointCloud<pcl_util::Point>(pose, (*obj)->cloud);
  TransformPointCloud<pcl_util::PointD>(pose, &((*obj)->polygon));
}

void HmObjectTracker::ComputeTracksPredict(
    std::vector<Eigen::VectorXf>* tracks_predict, const double time_diff) {
  // Compute tracks' predicted states
  int no_track = object_tracks_.Size();
  tracks_predict->resize(no_track);
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.GetTracks();
  for (int i = 0; i < no_track; ++i) {
    (*tracks_predict)[i] = tracks[i]->Predict(time_diff);
  }
}

void HmObjectTracker::UpdateAssignedTracks(
    std::vector<Eigen::VectorXf>* tracks_predict,
    std::vector<std::shared_ptr<TrackedObject>>* new_objects,
    const std::vector<std::pair<int, int>>& assignments,
    const double time_diff) {
  // Update assigned tracks
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.GetTracks();
  for (size_t i = 0; i < assignments.size(); ++i) {
    int track_id = assignments[i].first;
    int obj_id = assignments[i].second;
    tracks[track_id]->UpdateWithObject(&(*new_objects)[obj_id], time_diff);
  }
}

void HmObjectTracker::UpdateUnassignedTracks(
    const std::vector<Eigen::VectorXf>& tracks_predict,
    const std::vector<int>& unassigned_tracks, const double time_diff) {
  // Update tracks without matched objects
  std::vector<ObjectTrackPtr>& tracks = object_tracks_.GetTracks();
  for (size_t i = 0; i < unassigned_tracks.size(); ++i) {
    int track_id = unassigned_tracks[i];
    tracks[track_id]->UpdateWithoutObject(tracks_predict[track_id], time_diff);
  }
}

void HmObjectTracker::CreateNewTracks(
    const std::vector<std::shared_ptr<TrackedObject>>& new_objects,
    const std::vector<int>& unassigned_objects) {
  // Create new tracks for objects without matched tracks
  for (size_t i = 0; i < unassigned_objects.size(); ++i) {
    int obj_id = unassigned_objects[i];
    ObjectTrackPtr track(new ObjectTrack(new_objects[obj_id]));//有多少个object-tracked_object对象,就建立多少个object_track,并且加入到跟踪列表中
    object_tracks_.AddTrack(track); //object_tracks_是ObjectTrackSet对象
  }
}

void HmObjectTracker::DeleteLostTracks() {
  // Delete lost tracks
  object_tracks_.RemoveLostTracks();
}

void HmObjectTracker::CollectTrackedResults(
    std::vector<std::shared_ptr<Object>>* tracked_objects) {
  // Collect tracked results for reporting include objects may be occluded
  // temporarily
  const std::vector<ObjectTrackPtr>& tracks = object_tracks_.GetTracks();
  tracked_objects->resize(tracks.size());
  // std::cout << "maximum = " << config_.collect_consecutive_invisible_maximum << std::endl;
  int track_number = 0;
  for (size_t i = 0; i < tracks.size(); ++i) {
    if (tracks[i]->consecutive_invisible_count_ >
        config_.collect_consecutive_invisible_maximum)
      continue;
    if (tracks[i]->age_ < config_.collect_age_minimum) {
      continue;
    }
    //std::cout << "in CollectTrackedResults" << std::endl;
    std::shared_ptr<Object> obj(new Object);
    std::shared_ptr<TrackedObject> result_obj = tracks[i]->current_object_;
    obj->clone(*(result_obj->object_ptr));
    // fill tracked information of object
    obj->direction = result_obj->direction.cast<double>();
    if (fabs(obj->direction[0]) < DBL_MIN) {
      obj->theta = obj->direction(1) > 0 ? M_PI / 2 : -M_PI / 2;
    } else {
      obj->theta = atan2(obj->direction[1], obj->direction[0]);
    }
    obj->length = result_obj->size[0];
    obj->width = result_obj->size[1];
    obj->height = result_obj->size[2];
    obj->velocity = result_obj->velocity.cast<double>();
    obj->velocity_uncertainty = result_obj->velocity_uncertainty.cast<double>();
    obj->track_id = tracks[i]->idx_;
    obj->tracking_time = tracks[i]->period_;
    //obj->type = result_obj->type;
    obj->center = result_obj->center.cast<double>() - global_to_local_offset_;
    obj->anchor_point =
        result_obj->anchor_point.cast<double>() - global_to_local_offset_;
    // restore original world coordinates
    for (size_t j = 0; j < obj->cloud->size(); ++j) {
      obj->cloud->points[j].x -= global_to_local_offset_[0];
      obj->cloud->points[j].y -= global_to_local_offset_[1];
      obj->cloud->points[j].z -= global_to_local_offset_[2];
    }
    for (size_t j = 0; j < obj->polygon.size(); ++j) {
      obj->polygon.points[j].x -= global_to_local_offset_[0];
      obj->polygon.points[j].y -= global_to_local_offset_[1];
      obj->polygon.points[j].z -= global_to_local_offset_[2];
    }
    (*tracked_objects)[track_number] = obj;
    ++track_number;
  }
  // get the trajectory
  if(track_number!=0){
    if(first_time_){
      index_tracked_obj_.resize(track_number);
      drops_.resize(track_number);
      for(int i=0; i<track_number; i++){
        index_tracked_obj_[i] = (*tracked_objects)[i]->track_id;
        drops_[i].push_back((*tracked_objects)[i]->anchor_point);
        // drops_[i].push_back((*tracked_objects)[i]->center);
      }
      first_time_ = false;
      
    }else{
      std::vector<std::vector<Eigen::Vector3d>> drops_temp;
      std::vector<int> index_tracked_obj_temp;
      int i,j;
      for(i=0; i<track_number; i++){
        for(j=0; j<index_tracked_obj_.size(); j++){
          if((*tracked_objects)[i]->track_id == index_tracked_obj_[j]){
            drops_[j].push_back((*tracked_objects)[i]->center);
            drops_temp.push_back(drops_[j]);
            index_tracked_obj_temp.push_back(index_tracked_obj_[j]);
            break;
          }
        }
        if(j==index_tracked_obj_.size()){
          std::vector<Eigen::Vector3d> drop;
          drop.push_back((*tracked_objects)[i]->center);
          drops_temp.push_back(drop);
          index_tracked_obj_temp.push_back((*tracked_objects)[i]->track_id);
        }
      }

      for(int i=0; i<track_number; i++){
        (*tracked_objects)[i]->drops = drops_temp[i];
      }

      
      drops_.clear();
      drops_ = drops_temp;
      index_tracked_obj_.clear();
      index_tracked_obj_ = index_tracked_obj_temp;
      
      
    }// end
  }


  tracked_objects->resize(track_number);
}

}  // namespace perception
}  // namespace apollo
