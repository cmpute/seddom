
#include <ros/ros.h>
#include "server.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_server");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    seddom::SemanticOccupancyMapServer<seddom::Nuscenes, 3> server(nh, nh_private);
    ros::spin();

    return 0;
}
