#include "lidar_perception.h"

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"lidar_perception_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    lidar_perception::lidar_perception pt(nh,pnh);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}