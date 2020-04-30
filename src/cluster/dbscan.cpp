#include "dbscan.h"
namespace cluster{
dbscan::dbscan(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  pnh.param("min_sample", min_sample_, 3);
  pnh.param("eps", eps_, 10);
  pnh.param("pic_height", pic_height_, 400);
  pnh.param("pic_width", pic_width_, 1200);
  img_ = cv::Mat::zeros(pic_height_, pic_width_, CV_8UC1);
  label_ = cv::Mat::zeros(pic_height_, pic_width_, CV_8UC1);
  GenNeighborOffset();

}
void dbscan::cluster_vector(const VPointCloud::ConstPtr cloud, std::vector<pcl::PointIndices> &clusters){
  // prejection to binary image
  img_ = points_to_picture(cloud);
  label_ = cv::Mat::zeros(pic_height_, pic_width_, CV_8UC1);
  num_class_ = 0;
  for (int i = 0; i < pic_height_; i++){
    for (int j = 0; j < pic_width_; j++) {
      if (!IsValid(i, j) || IsVisited(i, j))
        continue;
      std::vector<cv::Point> neighbors;
      QueryNeighbors(i, j, neighbors);
      if (neighbors.size() < min_sample_){
        label_.at<uchar>(i, j) = 255;  // mark as noise or boundary points
      }else {
        num_class_++;
        label_.at<uchar>(i, j) = num_class_;
        ExpandCluster(num_class_, neighbors);
      }
    }
  }
  // std::cout << "number of clusters after clustering: " << (int)num_class_ << std::endl;

  if((int)num_class_>0){
    clusters.resize((int)num_class_);
    int num_points = cloud->points.size();
    for(int i=0; i<num_points; i++)
    {
      VPoint p = cloud->points[i];
      int col = floor((p.x-5)/0.01);//1cm
      int row = floor((2-p.y)/0.01);

      if(row>=0 && row<pic_height_ && col>=0 && col<pic_width_){
        uchar pixor = label_.at<uchar>(row,col);
        if(pixor>0 && pixor<255){
          clusters[(int)pixor-1].indices.push_back(i);
        }
      }
    }
  }
}
void dbscan::ExpandCluster(uchar c, const std::vector<cv::Point>& neighbor) {
  if(neighbor.size() > 0){
    for (const auto& pt: neighbor) {
      if (IsNoise(pt.x, pt.y)){
          label_.at<uchar>(pt.x, pt.y) = c;        // boundary points of c
      }else if (!IsVisited(pt.x, pt.y)) {
          label_.at<uchar>(pt.x, pt.y) = c;        // core points of c
          std::vector<cv::Point> recur_neighbors;
          QueryNeighbors(pt.x, pt.y, recur_neighbors);
          if (recur_neighbors.size() >= min_sample_)
              ExpandCluster(c, recur_neighbors);
      }else{
      }
    }
  
  }
}
void dbscan::QueryNeighbors(int row, int col, std::vector<cv::Point> &neighbor) {
  for (const auto& offset: neighbor_offset_) {
    int nrow = row + offset.x;
    int ncol = col + offset.y;
    if (IsValid(nrow, ncol))
      neighbor.emplace_back(nrow, ncol);
  }
}
void dbscan::GenNeighborOffset(){
  for (int i = -1*(int)eps_; i <= (int)eps_; i++){
    for (int j = -1*(int)eps_; j <= (int)eps_; j++) {
      if (i == 0 && j == 0)
        continue;
      double dist = sqrt(i*i + j*j);
      if (dist <= eps_) {
        neighbor_offset_.emplace_back(i, j);
      }
    }
  }   
}
bool dbscan::IsVisited(int row, int col) {
  return (label_.at<uchar>(row, col) > 0);
}
bool dbscan::IsValid(int row, int col) {
  if (row >= 0 && row < pic_height_ && col >= 0 && col < pic_width_)
      return (img_.at<uchar>(row, col) > 0);
  else
      return false;
}
bool dbscan::IsNoise(int row, int col) {
  return (label_.at<uchar>(row, col) == 255);
}
cv::Mat dbscan::points_to_picture(const VPointCloud::ConstPtr &rec)
{
  int row, col;
  cv::Mat img = cv::Mat::zeros(pic_height_, pic_width_, CV_8UC1);//255是白色  左右Ｙ轴左2ｍ右2m　X轴上长10m
  for(const VPoint p : rec->points)
  {
      col = floor((p.x-4)/0.01);//1cm
      row = floor((2-p.y)/0.01);

      if(row>=0 && row<pic_height_ && col>=0 && col<pic_width_)
      img.at<uchar>(row,col) = 255;
  }
  return img;
}

}// namespace cluster