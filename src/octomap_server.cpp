
#include <ros/ros.h>
#include "server.h"

constexpr int BlockDepth = 3;
using NuscenesMap = seddom::SemanticOccupancyMapServer<seddom::Nuscenes, BlockDepth>;
using SemanticKITTIMap = seddom::SemanticOccupancyMapServer<seddom::SemanticKITTI, BlockDepth>;
using CarlaMap = seddom::SemanticOccupancyMapServer<seddom::Carla, BlockDepth>;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_server");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string semclass;
    std::string bag_path;
    nh_private.param<std::string>("semantic_class", semclass, "");
    nh_private.param<std::string>("bag_path", bag_path, "");

    if (semclass == "semantic_kitti")
    {
        SemanticKITTIMap server(nh, nh_private);
        if (!bag_path.empty())
            server.run_bag(bag_path);
        ros::spin();
    }
    else if (semclass == "nuscenes")
    {
        NuscenesMap server(nh, nh_private);
        if (!bag_path.empty())
            server.run_bag(bag_path);
        ros::spin();
    }
    else if (semclass == "carla")
    {
        CarlaMap server(nh, nh_private);
        if (!bag_path.empty())
            server.run_bag(bag_path);
        ros::spin();
    }
    else if (semclass.empty())
    {
        ROS_ERROR("Please specify semantic class type! Valid options are {semantic_kitti, nuscenes}.");
        return -1;
    }
    else
    {
        ROS_ERROR("Incorrect semantic class type %s! Valid options are {semantic_kitti, nuscenes}.", semclass.c_str());
        return -2;
    }
    return 0;
}
