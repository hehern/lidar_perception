#ifndef LIDAR_PERSEPTION_H_
#define LIDAR_PERSEPTION_H_

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose2D.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <sstream>
#include <memory>
#include <time.h>

#include "types.h"
#include "pcl_clip/pcl_clip.h"
#include "remove_ground/remove_ground.h"
#include "cluster/region_growing_segmentation.h"
#include "cluster/EuclideanCluster.h"
#include "cluster/dbscan.h"
#include "min_box/min_box.h"
#include "hm_tracker/hm_tracker.h"
#include "lidar_perception/object.h"
// #include "path_map/path_map_ros.h"

namespace lidar_perception{
class lidar_perception{
    public:
    lidar_perception(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~lidar_perception() {};
    private:
    void PclCallback(const VPointCloud::ConstPtr &rec);
    void points_to_picture(const VPointCloud::ConstPtr &rec);
    void ClearAllMarker();

    ros::Subscriber pcl_subscriber_;
    ros::Publisher marker_pub_box_, marker_pub_tracinfo_, marker_pub_track_;
    ros::Publisher pub_pcl_cluster_, pub_pcl_cliped_;

    
    tf::TransformListener lidar2base_;


    int max_marker_size_;
    int picture_num_;

    bool is_using_path_roi_ = true;
    double speed_;
    double steer_;
    double wheelbase_ = 2.84;
    double front_overhang_ = 1.1;
    double vehicle_x_ = 0.0;
    double vehicle_y_ = 0.0;
    double vehicle_theta_ = 0.0;

    pcl_clip::Pcl_clip clip_;
    remove_ground::Remove_ground rm_;
    // cluster::region_growing cluster_;
    cluster::euclidean_cluster cluster_;
    // cluster::dbscan cluster_;
    std::unique_ptr<apollo::perception::MinBoxObjectBuilder> object_builder_;
    std::unique_ptr<apollo::perception::HmObjectTracker> tracker_;
    double timestamp_;
    apollo::perception::SeqId seq_num_;

};
}
#endif