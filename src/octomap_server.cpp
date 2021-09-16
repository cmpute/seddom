
#include <ros/ros.h>
#include "server.h"

constexpr int BlockDepth = 3;
using NuscenesMap = seddom::SemanticOccupancyMapServer<seddom::Nuscenes, BlockDepth>;
using SemanticKITTIMap = seddom::SemanticOccupancyMapServer<seddom::SemanticKITTI, BlockDepth>;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_server");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string semclass;
    nh_private.param<std::string>("semantic_class", semclass, "");

    if (semclass == "semantic_kitti")
    {
        SemanticKITTIMap server(nh, nh_private);
        ros::spin();
    }
    else if (semclass == "nuscenes")
    {
        NuscenesMap server(nh, nh_private);
        ros::spin();
    }
    else
    {
        ROS_ERROR("Incorrect semantic class type %s! Valid options are {semantic_kitti, nuscenes}.", semclass.c_str());
        return -1;
    }
    return 0;
}
