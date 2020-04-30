#ifndef DBSCAN_H_
#define DBSCAN_H_
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "types.h"


namespace cluster{
class dbscan{
    public:
    dbscan(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~dbscan() {};
    void cluster_vector(const VPointCloud::ConstPtr cloud, std::vector<pcl::PointIndices> &clusters);

    private:
    cv::Mat points_to_picture(const VPointCloud::ConstPtr &rec);
    bool IsNoise(int row, int col);
    bool IsValid(int row, int col);
    bool IsVisited(int row, int col);
    void GenNeighborOffset();
    void QueryNeighbors(int row, int col, std::vector<cv::Point> &neighbor);
    void ExpandCluster(uchar c, const std::vector<cv::Point>& neighbor);

    int min_sample_ = 3;
    int eps_ = 10;
    int pic_height_ = 700; //row
    int pic_width_ = 1000; //col
    cv::Mat img_;
    cv::Mat label_;
    uchar num_class_ = 0;
    std::vector<cv::Point> neighbor_offset_;
};
}


#endif