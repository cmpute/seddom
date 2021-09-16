#pragma once

#include <string>
#include <iostream>
#include <memory>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/OccupancyGrid.h>
#include <thread>

#include "bkioctomap.h"
#include "visualizer.h"

namespace seddom
{
    template <typename SemanticClass, size_t BlockDepth>
    class SemanticOccupancyMapServer
    {
    public:
        typedef SemanticBKIOctoMap<SemanticClass, BlockDepth> MapType;

        SemanticOccupancyMapServer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : _nh(nh), _nh_private(nh_private), _listener()
        {
            std::string cloud_topic("/semantic_points");
            std::string output_topic("/octomap");
            int chunk_depth = 6;
            float sf2 = 10.0;
            float ell = 0.3;
            float resolution = 0.3;
            float prior = 0.01f;
            float max_range = -1;

            nh_private.param<std::string>("cloud_topic", cloud_topic, cloud_topic);
            nh_private.param<std::string>("output_topic", output_topic, output_topic);
            nh_private.param<std::string>("map_frame_id", _map_frame_id, _map_frame_id);

            nh_private.param<float>("resolution", resolution, resolution);
            nh_private.param<int>("chunk_depth", chunk_depth, chunk_depth);
            nh_private.param<float>("sf2", sf2, sf2);
            nh_private.param<float>("ell", ell, ell);
            nh_private.param<float>("prior", prior, prior);
            nh_private.param<float>("max_range", max_range, max_range);
            nh_private.param<float>("free_resolution", _free_resolution, _free_resolution);
            nh_private.param<float>("ds_resolution", _ds_resolution, _ds_resolution);
            nh_private.param<bool>("visualize", _visualize, _visualize);
            nh_private.param<int>("random_samples_per_beam", _samples_per_beam, _samples_per_beam);

            _map = std::make_shared<MapType>(resolution, chunk_depth, sf2, ell, prior, max_range);
            _cloud_sub = _nh.subscribe(cloud_topic, 4, &SemanticOccupancyMapServer::cloud_callback, this);
            if (_visualize)
                _vis_pub = std::make_unique<seddom::OctomapVisualizer>(nh, output_topic, _map_frame_id);

            ROS_INFO_STREAM("Parameters:" << std::endl
                                          << "block_depth: " << BlockDepth << std::endl
                                          << "chunk_depth: " << chunk_depth << std::endl
                                          << "sf2: " << sf2 << std::endl
                                          << "ell: " << ell << std::endl
                                          << "prior:" << prior << std::endl
                                          << "resolution: " << resolution << std::endl
                                          << "free_resolution: " << _free_resolution << std::endl
                                          << "ds_resolution: " << _ds_resolution << std::endl
                                          << "max_range: " << max_range << std::endl);
        }

        void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud)
        {
            tf::StampedTransform transform;
            try
            {
                _listener.lookupTransform(_map_frame_id, cloud->header.frame_id, cloud->header.stamp, transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                return;
            }

            tf::Vector3 translation = transform.getOrigin();
            // TODO: make sure that this is the correct way to calculate origin
            pcl::PointXYZ origin(translation.x(), translation.y(), translation.z());
            ROS_DEBUG("Point cloud origin %.3f, %.3f, %3f", translation.x(), translation.y(), translation.z());

            sensor_msgs::PointCloud2 cloud_map;
            pcl_ros::transformPointCloud(_map_frame_id, transform, *cloud, cloud_map);

            PointCloudXYZL::Ptr pcl_cloud(new PointCloudXYZL());
            pcl::fromROSMsg(cloud_map, *pcl_cloud);

            if (_busy)
                return;
            else
                if (_worker.joinable())
                    _worker.join();

            _worker = std::thread([this, origin, pcl_cloud]{
                _busy = true;

                ros::Time tstart = ros::Time::now();
                if (_free_resolution >= 0)
                    _map->template insert_pointcloud<seddom::KernelType::BGK>(pcl_cloud, origin, _ds_resolution, _free_resolution);
                else if (_samples_per_beam >= 0)
                    _map->template insert_pointcloud<seddom::KernelType::BGK>(pcl_cloud, origin, _ds_resolution, _samples_per_beam);
                else
                    _map->template insert_pointcloud_pl<seddom::KernelType::SBGK>(pcl_cloud, origin, _ds_resolution);

                ros::Time tend = ros::Time::now();
                ROS_INFO("Inserted point cloud with %d points, takes %.2f ms", (int)pcl_cloud->size(), (tend - tstart).toSec() * 1000);

                if (_visualize)
                    _vis_pub->publish_octomap<SemanticClass, BlockDepth, OctomapVisualizeMode::SEMANTICS>(*_map);

                _busy = false;
            });
        }

    protected:
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_private;

        std::shared_ptr<MapType> _map;
        tf::TransformListener _listener;
        ros::Subscriber _cloud_sub;
        std::unique_ptr<seddom::OctomapVisualizer> _vis_pub;

        std::string _map_frame_id = "/odom";
        int _samples_per_beam = -1;
        float _free_resolution = 10;
        float _ds_resolution = 0.3;
        bool _visualize = true;

        std::thread _worker;
        bool _busy = false;  // for thread executing
    };

}