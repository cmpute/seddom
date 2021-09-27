#pragma once

#include <string>
#include <iostream>
#include <memory>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_msgs/TFMessage.h>
#include <thread>
#include <deque>

#include "seddom/bkioctomap.h"
#include "seddom/visualizer.h"
#include "seddom/storage.h"
#include "seddom/visualizer.h"
#include "seddom/heightmap.h"
#include "seddom/DumpMap.h"

namespace seddom
{
    template <typename SemanticClass, size_t BlockDepth>
    class SemanticOccupancyMapServer
    {
    public:
        typedef SemanticBKIOctoMap<SemanticClass, BlockDepth> MapType;

        SemanticOccupancyMapServer(ros::NodeHandle &nh, ros::NodeHandle &nh_private) : _nh(nh), _nh_private(nh_private), _listener(_tfbuffer)
        {
            std::string visualize_topic("");
            std::string gridmap_topic("");
            std::string map_path("");

            bool occlusion_aware = true;
            int chunk_depth = 6;
            float sf2 = 10.0;
            float ell = 0.3;
            float resolution = 0.3;
            float prior = 0.01f;
            float max_range = -1;

            nh_private.param<std::string>("cloud_topic", _cloud_topic, _cloud_topic);
            nh_private.param<std::string>("visualize_topic", visualize_topic, visualize_topic);
            nh_private.param<std::string>("gridmap_topic", gridmap_topic, gridmap_topic);
            nh_private.param<std::string>("map_frame_id", _map_frame_id, _map_frame_id);
            nh_private.param<std::string>("map_path", map_path, map_path);

            nh_private.param<bool>("occlusion_aware", occlusion_aware, occlusion_aware);
            nh_private.param<float>("resolution", resolution, resolution);
            nh_private.param<int>("chunk_depth", chunk_depth, chunk_depth);
            nh_private.param<float>("sf2", sf2, sf2);
            nh_private.param<float>("ell", ell, ell);
            nh_private.param<float>("prior", prior, prior);
            nh_private.param<float>("max_range", max_range, max_range);
            nh_private.param<float>("free_resolution", _free_resolution, _free_resolution);
            nh_private.param<float>("ds_resolution", _ds_resolution, _ds_resolution);
            nh_private.param<int>("random_samples_per_beam", _samples_per_beam, _samples_per_beam);

            _map = std::make_shared<MapType>(occlusion_aware, resolution, chunk_depth, sf2, ell, prior, max_range);
            _cloud_sub = _nh.subscribe(_cloud_topic, 4, &SemanticOccupancyMapServer::cloud_callback, this);
            if (!visualize_topic.empty())
                _visualizer = std::make_unique<seddom::OctomapVisualizer>(nh, visualize_topic, _map_frame_id);
            if (!gridmap_topic.empty())
                _zmap_generator = std::make_unique<seddom::HeightMapGenerator>(nh, gridmap_topic, "odom", 50); // TODO: select correct frame_id for the generated gridmap
            _dump_service = nh_private.advertiseService("dump_map", &SemanticOccupancyMapServer::dump_map_callback, this);

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

            if (!map_path.empty())
            {
                _storage = std::make_unique<seddom::OctomapStorage>(map_path, /* active_range */ 100);
                if (_storage->check_params(*_map))
                    ROS_INFO_STREAM("Database params are compatible.");
                else
                    ROS_ERROR_STREAM("Database params are incompatible!");
            }
        }

        bool dump_map_callback(seddom::DumpMap::Request &req, seddom::DumpMap::Response &res)
        {
            res.size = _map->dump_map(req.output_path);
            ROS_INFO("Map dumping finished.");
            return true;
        }

        void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud)
        {
            geometry_msgs::TransformStamped transform;
            try
            {
                transform = _tfbuffer.lookupTransform(_map_frame_id, cloud->header.frame_id, cloud->header.stamp);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                return;
            }

            pcl::PointXYZ origin(transform.transform.translation.x,
                                 transform.transform.translation.y,
                                 transform.transform.translation.z);
            ROS_DEBUG("Point cloud origin %.3f, %.3f, %3f", origin.x, origin.y, origin.z);

            sensor_msgs::PointCloud2 cloud_map;
            pcl_ros::transformPointCloud(_map_frame_id, transform.transform, *cloud, cloud_map);

            PointCloudXYZL::Ptr pcl_cloud(new PointCloudXYZL());
            pcl::fromROSMsg(cloud_map, *pcl_cloud);

            if (_busy)
                return;
            else
                if (_worker.joinable())
                    _worker.join();

            _worker = std::thread([this, origin, pcl_cloud]{
                _busy = true;
                run_point_cloud(pcl_cloud, origin);
                _busy = false;
            });
        }

        // this function is used to parse the data directly from a ROS bag
        void run_bag(const std::string &bag_path)
        {
            tf2_ros::Buffer tfbuffer;
            rosbag::Bag bag;
            bag.open(bag_path);
            std::deque<sensor_msgs::PointCloud2Ptr> cloud_queue;

            if (_storage != nullptr) // pre-load existing map
                _storage->load_around(*_map, pcl::PointXYZ(0,0,0));

            for(rosbag::MessageInstance const m: rosbag::View(bag))
            {
                if (m.getTopic() == _cloud_topic)
                {
                    cloud_queue.push_back(m.instantiate<sensor_msgs::PointCloud2>());
                }
                else if (m.getTopic() == "/tf_static")
                {
                    auto tfmsg = m.instantiate<tf2_msgs::TFMessage>();
                    for(int i = 0; i < tfmsg->transforms.size(); i++)
                        tfbuffer.setTransform(tfmsg->transforms[i], "default", /*is_static*/ true);
                }
                else if (m.getTopic() == "/tf")
                {
                    auto tfmsg = m.instantiate<tf2_msgs::TFMessage>();
                    for(int i = 0; i < tfmsg->transforms.size(); i++)
                        tfbuffer.setTransform(tfmsg->transforms[i], "default", /*is_static*/ false);
                }

                if (cloud_queue.size() == 0)
                    continue;
                sensor_msgs::PointCloud2Ptr cloud = cloud_queue.front();
                // Wait for the transform to be updated before processing the point clouds
                if (tfbuffer.canTransform(_map_frame_id, cloud->header.frame_id, cloud->header.stamp))
                {
                    // process cloud
                    cloud_queue.pop_front();
                    geometry_msgs::TransformStamped transform;
                    transform = tfbuffer.lookupTransform(_map_frame_id, cloud->header.frame_id, cloud->header.stamp);

                    pcl::PointXYZ origin(transform.transform.translation.x,
                                         transform.transform.translation.y,
                                         transform.transform.translation.z);
                    ROS_DEBUG("Point cloud origin %.3f, %.3f, %3f", origin.x, origin.y, origin.z);

                    sensor_msgs::PointCloud2 cloud_map;
                    pcl_ros::transformPointCloud(_map_frame_id, transform.transform, *cloud, cloud_map);

                    PointCloudXYZL::Ptr pcl_cloud(new PointCloudXYZL());
                    pcl::fromROSMsg(cloud_map, *pcl_cloud);

                    run_point_cloud(pcl_cloud, origin);
                    ros::spinOnce();
                    if (!ros::ok())
                        break;
                }
            }

            bag.close();
            ROS_INFO("ROS bag process completed.");
        }

        void run_point_cloud(PointCloudXYZL::ConstPtr pcl_cloud, const pcl::PointXYZ &origin)
        {
            ros::Time tstart = ros::Time::now();
            if (_free_resolution >= 0)
                _map->template insert_pointcloud<seddom::KernelType::BGK>(pcl_cloud, origin, _ds_resolution, _free_resolution);
            else if (_samples_per_beam >= 0)
                _map->template insert_pointcloud<seddom::KernelType::BGK>(pcl_cloud, origin, _ds_resolution, _samples_per_beam);
            else
                _map->template insert_pointcloud_pl<seddom::KernelType::SBGK>(pcl_cloud, origin, _ds_resolution);

            ros::Time tend = ros::Time::now();
            ROS_INFO("Inserted point cloud with %d points, takes %.2f ms", (int)pcl_cloud->size(), (tend - tstart).toSec() * 1000);

            if (_zmap_generator != nullptr)
                _zmap_generator->publish_octomap<SemanticClass, BlockDepth>(*_map);
            if (_visualizer != nullptr)
                _visualizer->publish_octomap<SemanticClass, BlockDepth, OctomapVisualizeMode::SEMANTICS>(*_map);
            if (_storage != nullptr)
                _storage->sync(*_map);
        }

        ~SemanticOccupancyMapServer()
        {
            if (_storage != nullptr)
            {
                std::cout << "Save all map blocks..." << std::endl;
                _storage->dump_all(*_map);
            }
        }

    protected:
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_private;

        std::shared_ptr<MapType> _map;
        tf2_ros::Buffer _tfbuffer;
        tf2_ros::TransformListener _listener;
        ros::Subscriber _cloud_sub;
        ros::ServiceServer _dump_service;
        std::string _cloud_topic = "/semantic_points";
        std::unique_ptr<seddom::OctomapVisualizer> _visualizer;
        std::unique_ptr<seddom::OctomapStorage> _storage;
        std::unique_ptr<seddom::HeightMapGenerator> _zmap_generator;

        std::string _map_frame_id = "odom";
        int _samples_per_beam = -1;
        float _free_resolution = 10;
        float _ds_resolution = 0.3;
        bool _visualize = true;

        std::thread _worker;
        bool _busy = false;  // for thread executing
    };
}