#ifndef MODULES_PERCEPTION_TRACKER_CONFIG_H_
#define MODULES_PERCEPTION_TRACKER_CONFIG_H_

#include <string>

namespace apollo {
namespace perception {
namespace tracker_config {

struct alignas(16) ModelConfigs {
  // ModelConfigs();

  // 成员
  std::string name;
  std::string version;
  std::string matcher_method;
  std::string filter_method;

  int track_cached_history_size_maximum;
  int track_consecutive_invisible_maximum;
  double track_visible_ratio_minimum;
  int collect_age_minimum;
  int collect_consecutive_invisible_maximum;
  int acceleration_noise_maximum;
  double speed_noise_maximum;
  double match_distance_maximum;
  double location_distance_weight;
  double direction_distance_weight;
  double bbox_size_distance_weight;
  double point_num_distance_weight;
  double histogram_distance_weight;
  int histogram_bin_size;
  bool use_adaptive;
  double measurement_noise;
  int initial_velocity_noise;
  int xy_propagation_noise;
  int z_propagation_noise;
  double breakdown_threshold_maximum;

};

}  // namespace tracker_config
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRACKER_CONFIG_H_
