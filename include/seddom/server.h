#pragma once

#include <chrono>
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
    template<typename T>
    void moveFromROSMsg(sensor_msgs::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud, std::string label_field = "label")
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::moveToPCL(cloud, pcl_pc2);
        pcl::MsgFieldMap field_map;

        // change the field name to the given one
        if (label_field != "label") {
            for (auto &field : pcl_pc2.fields) {
                if (field.name == label_field) {
                    field.name = "label";
                }
            }
        }
    
        pcl::createMapping<T>(pcl_pc2.fields, field_map);
        pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud, field_map);
    }

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
            std::string occlusion_handling("none");

            int chunk_depth = 6;
            float gridmap_range = 100;
            float sf2 = 10.0;
            float ell = 0.3;
            float resolution = 0.3;
            float prior = 0.01f;
            float max_range = -1;

            nh_private.param<std::string>("cloud_topic", _cloud_topic, _cloud_topic);
            nh_private.param<std::string>("visualize_topic", visualize_topic, visualize_topic);
            nh_private.param<std::string>("gridmap_topic", gridmap_topic, gridmap_topic);
            nh_private.param<std::string>("label_field", _label_field, _label_field);
            nh_private.param<std::string>("map_frame_id", _map_frame_id, _map_frame_id);
            nh_private.param<std::string>("map_path", map_path, map_path);
            nh_private.param<std::string>("occlusion_handling", occlusion_handling, occlusion_handling);

            nh_private.param<bool>("read_only", _read_only, false);
            nh_private.param<float>("resolution", resolution, resolution);
            nh_private.param<int>("chunk_depth", chunk_depth, chunk_depth);
            nh_private.param<float>("gridmap_range", gridmap_range, gridmap_range);
            nh_private.param<float>("sf2", sf2, sf2);
            nh_private.param<float>("ell", ell, ell);
            nh_private.param<float>("prior", prior, prior);
            nh_private.param<float>("max_range", max_range, max_range);
            nh_private.param<float>("free_resolution", _free_resolution, _free_resolution);
            nh_private.param<float>("ds_resolution", _ds_resolution, _ds_resolution);
            nh_private.param<int>("random_samples_per_beam", _samples_per_beam, _samples_per_beam);

            OcclusionHandling oh;
            if (occlusion_handling == "none")
                oh = OcclusionHandling::NONE;
            else if (occlusion_handling == "known")
                oh = OcclusionHandling::KNOWN;
            else if (occlusion_handling == "all")
                oh = OcclusionHandling::ALL;
            else
            {
                ROS_WARN("Unrecognized occlusion handling flag (possible options: none, known, all). Occlusion handling will be disabled.");
                oh = OcclusionHandling::NONE;
            }

            _map = std::make_shared<MapType>(oh, resolution, chunk_depth, sf2, ell, prior, max_range, _read_only);
            _cloud_sub = _nh.subscribe(_cloud_topic, 4, &SemanticOccupancyMapServer::cloud_callback, this);

            // these publisers are for messages from ros bag.
            _tf_pub = _nh.advertise<tf2_msgs::TFMessage>("/tf", 1, 1); 
            _tf_static_pub = _nh.advertise<tf2_msgs::TFMessage>("/tf_static", 1, 1);
#ifndef NDEBUG
            _cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>(_cloud_topic, 1, 1);
#endif

            if (!visualize_topic.empty())
            {
                ROS_INFO("Visualizer created, topic: %s", visualize_topic.c_str());
                _visualizer = std::make_unique<seddom::OctomapVisualizer>(nh, visualize_topic, _map_frame_id);
            }
            if (!gridmap_topic.empty())
                _zmap_generator = std::make_unique<seddom::HeightMapGenerator>(nh, gridmap_topic, _map_frame_id,
                    gridmap_range, _map->resolution(), oh); // TODO: select correct frame_id for the generated gridmap
            _dump_service = nh_private.advertiseService("dump_map", &SemanticOccupancyMapServer::dump_map_callback, this);

            ROS_INFO_STREAM("Parameters:" << std::endl
                << "block_depth: " << BlockDepth << std::endl
                << "chunk_depth: " << chunk_depth << std::endl
                << "sf2: " << sf2 << std::endl
                << "ell: " << ell << std::endl
                << "prior: " << prior << std::endl
                << "resolution: " << resolution << std::endl
                << "free_resolution: " << _free_resolution << std::endl
                << "ds_resolution: " << _ds_resolution << std::endl
                << "max_range: " << max_range << std::endl
                << "read_only: " << _read_only << std::endl);

            if (!map_path.empty())
            {
                _storage = std::make_unique<seddom::OctomapStorage>(map_path, /* active_range */ 300, _read_only);
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
            if (_busy)
                return; // don't accept new cloud when there is existing job
            _cloud_queue.push_back(cloud);

            // Wait for the transform to be updated before processing the point clouds
            if (_tfbuffer.canTransform(_map_frame_id, cloud->header.frame_id, cloud->header.stamp))
            {
                try
                {
                    // process cloud
                    _cloud_queue.pop_front();
                    auto transform = _tfbuffer.lookupTransform(_map_frame_id, cloud->header.frame_id, cloud->header.stamp);

                    pcl::PointXYZ origin(transform.transform.translation.x,
                                        transform.transform.translation.y,
                                        transform.transform.translation.z);
                    ROS_DEBUG("Point cloud origin %.3f, %.3f, %3f", origin.x, origin.y, origin.z);

                    sensor_msgs::PointCloud2 cloud_map;
                    pcl_ros::transformPointCloud(_map_frame_id, transform.transform, *cloud, cloud_map);

                    PointCloudXYZL::Ptr pcl_cloud(new PointCloudXYZL());
                    seddom::moveFromROSMsg(cloud_map, *pcl_cloud, _label_field);

                    if (_worker.joinable())
                        _worker.join();
                    _worker = std::thread([this, origin, pcl_cloud]{
                        _busy = true;
                        run_point_cloud(pcl_cloud, origin);
                        _busy = false;
                    });
                }
                catch (tf::TransformException ex)
                {
                    ROS_ERROR("%s", ex.what());
                    return;
                }
            } else {
                // TODO: discard very old messages
            }
        }

        // this function is used to parse the data directly from a ROS bag
        void run_bag(const std::string &bag_path)
        {
            rosbag::Bag bag;
            bag.open(bag_path);

            if (_storage != nullptr) // pre-load existing map
                _storage->load_around(*_map, pcl::PointXYZ(0,0,0));

            for(rosbag::MessageInstance const m: rosbag::View(bag))
            {
                if (m.getTopic() == _cloud_topic)
                {
                    _cloud_queue.push_back(m.instantiate<sensor_msgs::PointCloud2>());
                }
                else if (m.getTopic() == "/tf_static")
                {
                    _tf_static_pub.publish(m);
                    auto tfmsg = m.instantiate<tf2_msgs::TFMessage>();
                    for(int i = 0; i < tfmsg->transforms.size(); i++)
                        _tfbuffer.setTransform(tfmsg->transforms[i], "default", /*is_static*/ true);
                }
                else if (m.getTopic() == "/tf")
                {
                    _tf_pub.publish(m);
                    auto tfmsg = m.instantiate<tf2_msgs::TFMessage>();
                    for(int i = 0; i < tfmsg->transforms.size(); i++)
                        _tfbuffer.setTransform(tfmsg->transforms[i], "default", /*is_static*/ false);
                }

                if (_cloud_queue.size() == 0)
                    continue;
                sensor_msgs::PointCloud2ConstPtr cloud = _cloud_queue.front();
                // Wait for the transform to be updated before processing the point clouds
                if (_tfbuffer.canTransform(_map_frame_id, cloud->header.frame_id, cloud->header.stamp))
                {
                    // process cloud
                    _cloud_queue.pop_front();
#ifndef NDEBUG
                    _cloud_pub.publish(cloud);
#endif

                    geometry_msgs::TransformStamped transform;
                    transform = _tfbuffer.lookupTransform(_map_frame_id, cloud->header.frame_id, cloud->header.stamp);

                    pcl::PointXYZ origin(transform.transform.translation.x,
                                         transform.transform.translation.y,
                                         transform.transform.translation.z);
                    ROS_DEBUG("Point cloud origin %.3f, %.3f, %3f", origin.x, origin.y, origin.z);

                    sensor_msgs::PointCloud2 cloud_map;
                    pcl_ros::transformPointCloud(_map_frame_id, transform.transform, *cloud, cloud_map);

                    PointCloudXYZL::Ptr pcl_cloud(new PointCloudXYZL());
                    seddom::moveFromROSMsg(cloud_map, *pcl_cloud, _label_field);

                    run_point_cloud(pcl_cloud, origin);
                    ros::spinOnce();
                    if (!ros::ok())
                        break;
                }
            }

            bag.close();
            ROS_INFO("ROS bag process completed, remaining clouds: %ld", _cloud_queue.size());
        }

        void run_point_cloud(PointCloudXYZL::ConstPtr pcl_cloud, const pcl::PointXYZ &origin)
        {
            auto tstart = std::chrono::system_clock::now();
            if (_free_resolution >= 0)
                _map->template insert_pointcloud<seddom::KernelType::BGK>(pcl_cloud, origin, _ds_resolution, _free_resolution);
            else if (_samples_per_beam >= 0)
                _map->template insert_pointcloud<seddom::KernelType::BGK>(pcl_cloud, origin, _ds_resolution, _samples_per_beam);
            else
                _map->template insert_pointcloud_pl<seddom::KernelType::SBGK>(pcl_cloud, origin, _ds_resolution);

            auto tend = std::chrono::system_clock::now();
            ROS_INFO("Inserted point cloud with %d points, takes %ld ms", (int)pcl_cloud->size(),
                std::chrono::duration_cast<std::chrono::milliseconds>(tend - tstart).count());

            if (_zmap_generator != nullptr)
            {
                tstart = std::chrono::system_clock::now();
                _zmap_generator->publish_octomap<SemanticClass, BlockDepth>(*_map);
                tend = std::chrono::system_clock::now();
                ROS_INFO("(1) Height map are generated in %ld ms",
                    std::chrono::duration_cast<std::chrono::milliseconds>(tend - tstart).count());
            }
            if (_visualizer != nullptr)
            {
                tstart = std::chrono::system_clock::now();
                _visualizer->publish_octomap<SemanticClass, BlockDepth, OctomapVisualizeMode::SEMANTICS>(*_map);
                tend = std::chrono::system_clock::now();
                ROS_INFO("(2) 3D map markers are published in %ld ms",
                    std::chrono::duration_cast<std::chrono::milliseconds>(tend - tstart).count());
            }
            if (_storage != nullptr)
            {
                tstart = std::chrono::system_clock::now();
                _storage->sync(*_map);
                tend = std::chrono::system_clock::now();
                ROS_INFO("(3) Map storage is synced in %ld ms",
                    std::chrono::duration_cast<std::chrono::milliseconds>(tend - tstart).count());
            }
        }

        ~SemanticOccupancyMapServer()
        {
            if (_worker.joinable())
                _worker.join();
            if (_storage != nullptr && !_read_only)
            {
                std::cout << "Save all map blocks..." << std::endl;
                _storage->dump_all(*_map);
            }
        }

    protected:
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_private;
        ros::Subscriber _cloud_sub;
        ros::ServiceServer _dump_service;
        ros::Publisher _tf_pub, _tf_static_pub, _cloud_pub;

        std::shared_ptr<MapType> _map;
        tf2_ros::Buffer _tfbuffer;
        tf2_ros::TransformListener _listener;
        std::string _cloud_topic = "/semantic_points";
        std::string _label_field = "label";
        std::unique_ptr<seddom::OctomapVisualizer> _visualizer;
        std::unique_ptr<seddom::OctomapStorage> _storage;
        std::unique_ptr<seddom::HeightMapGenerator> _zmap_generator;
        std::deque<sensor_msgs::PointCloud2ConstPtr> _cloud_queue;

        std::string _map_frame_id = "odom";
        int _samples_per_beam = -1;
        float _free_resolution = 10;
        float _ds_resolution = 0.3;
        bool _visualize = true;
        bool _read_only = false;

        std::thread _worker;
        bool _busy = false;  // for thread executing
    };
}