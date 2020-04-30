#include "lidar_perception.h"

using namespace std;
using namespace cv;

namespace lidar_perception{
lidar_perception::lidar_perception(ros::NodeHandle nh, ros::NodeHandle pnh) : clip_(nh, pnh), rm_(nh, pnh), cluster_(nh, pnh)
{

    pcl_subscriber_ = nh.subscribe("/front/rslidar_points", 1, &lidar_perception::PclCallback, this);
    marker_pub_box_ = nh.advertise<visualization_msgs::MarkerArray>("/lidar_perception/marker_box",1);
    marker_pub_tracinfo_ = nh.advertise<visualization_msgs::MarkerArray>("/lidar_perception/marker_info",1);
    marker_pub_track_ = nh.advertise<visualization_msgs::MarkerArray>("/lidar_perception/marker_track",1);
    pub_pcl_cluster_ = nh.advertise<VPointCloud>("/lidar_perception/cluster_points",10);
    pub_pcl_cliped_ = nh.advertise<VPointCloud>("/lidar_perception/points",10);

    max_marker_size_ = 0;
    picture_num_ = 0;

    object_builder_.reset(new apollo::perception::MinBoxObjectBuilder);

    timestamp_ = 0.0;
    seq_num_ = 0;
    tracker_.reset(new apollo::perception::HmObjectTracker(nh, pnh));
    tracker_->Init();


}

void lidar_perception::PclCallback(const VPointCloud::ConstPtr &rec)
{
    if(rec->points.size() > 0)
    {
        clock_t time_begin = clock();
        // tf coordinate transformation, from lidr to car
        VPointCloud::Ptr rec_point(new VPointCloud);
        tf::StampedTransform tf_front_base;
        lidar2base_.lookupTransform("/vehicle","/front_lidar", ros::Time(0), tf_front_base);
        pcl_ros::transformPointCloud(*rec,*rec_point,tf_front_base);


        // clip teh pcl
        VPointCloud::Ptr cloud_rm_veh = clip_.Clip_vehicle(rec_point);
        cloud_rm_veh->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        cloud_rm_veh->header.frame_id = "vehicle";
        pub_pcl_cliped_.publish(cloud_rm_veh);
        VPointCloud::Ptr in = clip_.GetPcl(rec_point);


        //remove ground
        VPointCloud::Ptr points_rm = rm_.GetFilteredWithNearestNeighborVariance_new(in);

        // point cloud to picture
        // points_to_picture(points_rm);

    
        // cluster
        std::vector<pcl::PointIndices> clusters;
        cluster_.cluster_vector(points_rm, clusters);
        int counter = clusters.size();
        float intensity = 50.0;
        std::vector<VPointCloud::Ptr> points_vector;
        if(counter > 0)
        {
            points_vector.clear();
            for(int i = 0; i<counter; i++)
            {
                VPointCloud::Ptr temp_cluster(new VPointCloud);
                pcl::copyPointCloud(*points_rm, clusters[i], *temp_cluster);
                int points_size = temp_cluster->size();
                for(int i=0;i<points_size;i++){
                    temp_cluster->at(i).intensity = intensity;
                }
                intensity += 50;
                points_vector.push_back(temp_cluster);
            }
        }
        std::cout << "cluster done !" << std::endl;


        // display the pointcloud
        VPointCloud::Ptr pcl_cluster_ptr(new VPointCloud);
        for(int i=0; i<counter; i++){
            *pcl_cluster_ptr += *points_vector[i];
        }
        pcl_cluster_ptr->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        pcl_cluster_ptr->header.frame_id = "vehicle";
        pub_pcl_cluster_.publish(pcl_cluster_ptr);


        // min_box
        std::vector<std::shared_ptr<apollo::perception::Object>> objects;
        for(int i=0; i<counter; i++)
        {
            std::shared_ptr<apollo::perception::Object> out_obj(new apollo::perception::Object);
            out_obj->cloud = points_vector[i];
            objects.push_back(out_obj);
        }
        if (object_builder_ != nullptr) {
            apollo::perception::ObjectBuilderOptions object_builder_options;
            if (!object_builder_->Build(object_builder_options, &objects)) {
            return;
            }
        }
        std::cout << "minbox done !" << std::endl;


        // tracker
        std::shared_ptr<apollo::perception::SensorObjects> out_sensor_objects(new apollo::perception::SensorObjects);
        if(objects.size() > 0)
        {
            const double kTimeStamp = ros::Time::now().toSec();
            timestamp_ = kTimeStamp;
            ++seq_num_;
            std::shared_ptr<Eigen::Matrix4d> velodyne_trans = std::make_shared<Eigen::Matrix4d>();  //车辆坐标系向ENU坐标系转换的转换矩阵
            *velodyne_trans << 1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0,
                                0, 0, 0, 1;                  
            out_sensor_objects->timestamp = timestamp_;
            out_sensor_objects->sensor2world_pose = *velodyne_trans;
            out_sensor_objects->seq_num = seq_num_;
            if (tracker_ != nullptr) {
                apollo::perception::TrackerOptions tracker_options;
                tracker_options.velodyne_trans = velodyne_trans;
                if (!tracker_->Track(objects, timestamp_, tracker_options,
                                    &(out_sensor_objects->objects))) {
                std::cout << "tracker running error !" << std::endl;
                }
            }
            std::cout << "tracked size = " << out_sensor_objects->objects.size() << std::endl;

        }
        std::cout << "tracker done !" << std::endl;
   

        // display the markers
        if(out_sensor_objects->objects.size() > 0){
            ClearAllMarker();
            visualization_msgs::MarkerArray marker_array_box, marker_array_info, marker_array_track;

            visualization_msgs::Marker line_strip;
            line_strip.header.frame_id = "vehicle";
            line_strip.header.stamp = ros::Time::now();
            line_strip.ns = "apollo::perception";
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.scale.x = 0.01;
            line_strip.color.r = 0.0;
            line_strip.color.g = 1.0;
            line_strip.color.b = 0.0;
            line_strip.color.a = 1.0;
            line_strip.points.resize(5);
            int marker_id = 0;
            int trackerd_num = out_sensor_objects->objects.size();
            for(int i=0;i<trackerd_num;i++){
                geometry_msgs::Point p1,p2,p3,p4,p5,p6,p7,p8;
                p1.x = p5.x = out_sensor_objects->objects[i]->vertex1[0];
                p1.y = p5.y = out_sensor_objects->objects[i]->vertex1[1];
                p1.z = out_sensor_objects->objects[i]->min_height;
                p5.z = out_sensor_objects->objects[i]->max_height;
                p2.x = p6.x = out_sensor_objects->objects[i]->vertex2[0];
                p2.y = p6.y = out_sensor_objects->objects[i]->vertex2[1];
                p2.z = out_sensor_objects->objects[i]->min_height;
                p6.z = out_sensor_objects->objects[i]->max_height;
                p3.x = p7.x = out_sensor_objects->objects[i]->vertex3[0];
                p3.y = p7.y = out_sensor_objects->objects[i]->vertex3[1];
                p3.z = out_sensor_objects->objects[i]->min_height;
                p7.z = out_sensor_objects->objects[i]->max_height;
                p4.x = p8.x = out_sensor_objects->objects[i]->vertex4[0];
                p4.y = p8.y = out_sensor_objects->objects[i]->vertex4[1];
                p4.z = out_sensor_objects->objects[i]->min_height;
                p8.z = out_sensor_objects->objects[i]->max_height;
                line_strip.id = marker_id;
                line_strip.points[0] = p1;
                line_strip.points[1] = p2;
                line_strip.points[2] = p4;
                line_strip.points[3] = p3;
                line_strip.points[4] = p1;
                marker_array_box.markers.push_back(line_strip);
                marker_id++;
                line_strip.id = marker_id;
                line_strip.points[0] = p5;
                line_strip.points[1] = p6;
                line_strip.points[2] = p8;
                line_strip.points[3] = p7;
                line_strip.points[4] = p5;
                marker_array_box.markers.push_back(line_strip);
                marker_id++;
                line_strip.id = marker_id;
                line_strip.points[0] = p1;
                line_strip.points[1] = p5;
                line_strip.points[2] = p7;
                line_strip.points[3] = p3;
                line_strip.points[4] = p1;
                marker_array_box.markers.push_back(line_strip);
                marker_id++;
                line_strip.id = marker_id;
                line_strip.points[0] = p2;
                line_strip.points[1] = p6;
                line_strip.points[2] = p8;
                line_strip.points[3] = p4;
                line_strip.points[4] = p2;
                marker_array_box.markers.push_back(line_strip);
                marker_id++;
            }

            visualization_msgs::Marker marker_txt;
            marker_txt.header.frame_id = "vehicle";
            marker_txt.header.stamp = ros::Time::now();
            marker_txt.ns = "apollo::perception";
            marker_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_txt.scale.z = 0.1;
            marker_txt.color.r = 1.0;
            marker_txt.color.g = 1.0;
            marker_txt.color.b = 1.0;
            marker_txt.color.a = 1.0;
            for(int i=0; i<trackerd_num; i++){
                marker_txt.pose.position.x = out_sensor_objects->objects[i]->anchor_point[0];
                marker_txt.pose.position.y = out_sensor_objects->objects[i]->anchor_point[1];
                marker_txt.pose.position.z = out_sensor_objects->objects[i]->max_height + 0.3;
                marker_txt.id = i;
                marker_txt.text = std::string("id: ") + std::to_string(out_sensor_objects->objects[i]->track_id) + "\n";
                double speed = sqrt(X2(out_sensor_objects->objects[i]->velocity[0]) + X2(out_sensor_objects->objects[i]->velocity[1]));
                marker_txt.text += std::string("speed: ") + std::to_string(speed) + " m/s\n";
                marker_txt.text += std::string("age: ") + std::to_string(static_cast<int>(out_sensor_objects->objects[i]->tracking_time)) + "s\n";
                marker_array_info.markers.push_back(marker_txt);
            }

            for(int i=0; i<trackerd_num; i++){
                if(out_sensor_objects->objects[i]->drops.size() > 0){
                    visualization_msgs::Marker marker_track;
                    marker_track.header.frame_id = "vehicle";
                    marker_track.header.stamp = ros::Time::now();
                    marker_track.ns = "apollo::perception";
                    marker_track.type = visualization_msgs::Marker::LINE_STRIP;
                    marker_track.scale.x = 0.01;
                    marker_track.color.g = (1.0/trackerd_num)*i;
                    marker_track.color.a = 1.0;
                    marker_track.id = i;
                    for(auto p : out_sensor_objects->objects[i]->drops){
                        geometry_msgs::Point p1;
                        p1.x = p[0];
                        p1.y = p[1];
                        p1.z = p[2];
                        marker_track.points.push_back(p1);
                    }
                    marker_array_track.markers.push_back(marker_track);
                }

            }

            marker_pub_box_.publish(marker_array_box);
            marker_pub_tracinfo_.publish(marker_array_info);
            marker_pub_track_.publish(marker_array_track);
        }
        clock_t time_end = clock();
        std::cout << "all time = " << (double)(time_end-time_begin)/CLOCKS_PER_SEC << std::endl;

    }
}
void lidar_perception::points_to_picture(const VPointCloud::ConstPtr &rec)
{
    int row, col;
    Mat img(700, 1000, CV_8UC1, Scalar(0));//255是白色  左右Ｙ轴左2ｍ右5m　X轴上长10m
    for(const VPoint &p : rec->points)
    {
        col = floor((p.x-5)/0.01);//1cm
        row = floor((2-p.y)/0.01);

        if(row>=0 && row<1000 && col>=0 && col<1000)
        img.at<uchar>(row,col) = 255;
    }
    string s;
    stringstream ss;
    ss << picture_num_;
    ss >> s;
    imwrite("/home/ubuntu/3D_detector/people_tracker/src/people_tracker/pictures/"+s+".jpg",img);
    cout << "write ok" << endl;
    picture_num_ ++;

}
void lidar_perception::ClearAllMarker() {
  visualization_msgs::MarkerArray::Ptr clear_marker_array(new visualization_msgs::MarkerArray);
  visualization_msgs::Marker dummy_marker;
  dummy_marker.action = visualization_msgs::Marker::DELETEALL;
  clear_marker_array->markers.push_back(dummy_marker);
  marker_pub_box_.publish(clear_marker_array);
  marker_pub_tracinfo_.publish(clear_marker_array);
  marker_pub_track_.publish(clear_marker_array);
}



}// namespace people_tracker