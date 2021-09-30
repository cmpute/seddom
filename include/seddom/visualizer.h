#pragma once

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <algorithm>
#include <cmath>
#include <string>
#include "seddom/semantics.h"

namespace seddom
{

    enum OctomapVisualizeMode
    {
        HEIGHT,
        SEMANTICS,
        VARIANCE
    };

    class OctomapVisualizer // TODO: rename to ROSVisualizer and add separate hpp implementation
    {
    public:
        OctomapVisualizer(ros::NodeHandle nh, std::string topic, std::string frame_id = "map") : _nh(nh),
                                                                   _topic(topic),
                                                                   _frame_id(frame_id)
        {
            _pub = nh.advertise<visualization_msgs::MarkerArray>(topic, 1, true);
        }

        /*
         * @brief Visualize the octomap and publish ros message.
         * @param mode specify how to color the grids 0=height, 1=semantics, 2=variance
         */
        template <typename SemanticClass, size_t BlockDepth, OctomapVisualizeMode Mode>
        void publish_octomap(const SemanticBKIOctoMap<SemanticClass, BlockDepth>& map) const
        {
            visualization_msgs::MarkerArray::Ptr msg(new visualization_msgs::MarkerArray);

            // initialize markers
#ifndef NDEBUG
            msg->markers.resize(BlockDepth+2);
#else
            msg->markers.resize(BlockDepth+1);
#endif
            ros::Time ts = ros::Time::now();
            for (int i = 0; i < BlockDepth; ++i)
            {
                msg->markers[i].header.frame_id = _frame_id;
                msg->markers[i].header.stamp = ts;
                switch(Mode)
                {
                    default:
                    case OctomapVisualizeMode::HEIGHT:
                        msg->markers[i].ns = "octomap/height" + std::to_string(i);
                        break;
                    case OctomapVisualizeMode::SEMANTICS:
                        msg->markers[i].ns = "octomap/semantics" + std::to_string(i);
                        break;
                    case OctomapVisualizeMode::VARIANCE:
                        msg->markers[i].ns = "octomap/variance" + std::to_string(i);
                        break;
                }
                msg->markers[i].id = i;
                msg->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
                msg->markers[i].scale.x = map.resolution() * pow(2, BlockDepth - 1 - i);
                msg->markers[i].scale.y = map.resolution() * pow(2, BlockDepth - 1 - i);
                msg->markers[i].scale.z = map.resolution() * pow(2, BlockDepth - 1 - i);
            }

            // initialize marker for the occluded blocks
            msg->markers.emplace_back();
            msg->markers[BlockDepth].header.frame_id = _frame_id;
            msg->markers[BlockDepth].header.stamp = ts;
            msg->markers[BlockDepth].type = visualization_msgs::Marker::SPHERE_LIST;
            msg->markers[BlockDepth].scale.x = map.resolution() * 0.8;
            msg->markers[BlockDepth].scale.y = map.resolution() * 0.8;
            msg->markers[BlockDepth].scale.z = map.resolution() * 0.8;
            msg->markers[BlockDepth].ns = "octomap/occluded";

            msg->markers.emplace_back();
            msg->markers[BlockDepth+1].header.frame_id = _frame_id;
            msg->markers[BlockDepth+1].header.stamp = ts;
            msg->markers[BlockDepth+1].type = visualization_msgs::Marker::SPHERE_LIST;
            msg->markers[BlockDepth+1].scale.x = map.block_size() * 0.8;
            msg->markers[BlockDepth+1].scale.y = map.block_size() * 0.8;
            msg->markers[BlockDepth+1].scale.z = map.block_size() * 0.8;
            msg->markers[BlockDepth+1].ns = "octomap/occluded_full";

#ifndef NDEBUG
            // initialize marker for debug states
            msg->markers.emplace_back();
            msg->markers[BlockDepth+2].header.frame_id = _frame_id;
            msg->markers[BlockDepth+2].header.stamp = ts;
            msg->markers[BlockDepth+2].type = visualization_msgs::Marker::POINTS;
            msg->markers[BlockDepth+2].scale.x = map.resolution() * 0.2;
            msg->markers[BlockDepth+2].scale.y = map.resolution() * 0.2;
            msg->markers[BlockDepth+2].ns = "octomap/debug_state";
#endif

            // calculate scale
            float min_v = std::numeric_limits<float>::max(), max_v = std::numeric_limits<float>::min();
            if (Mode == OctomapVisualizeMode::HEIGHT)
            {
                for (auto it = map.cbegin_leaf(); it != map.cend_leaf(); ++it)
                {
                    auto z = it.get_loc().z;
                    min_v = std::min(min_v, z);
                    max_v = std::max(max_v, z);
                }
            }
            else if (Mode == OctomapVisualizeMode::VARIANCE)
            {
                for (auto it = map.cbegin_leaf(); it != map.cend_leaf(); ++it)
                {
                    auto var = it->get_vars()[it->get_semantics()];
                    min_v = std::min(min_v, var);
                    max_v = std::max(max_v, var);
                }
            }

            // fill blocks
            for (auto it = map.cbegin_leaf(); it != map.cend_leaf(); ++it)
            {
                pcl::PointXYZ p = it.get_loc();
                if (it->is_occupied())
                {
                    switch (Mode)
                    {
                    default:
                    case OctomapVisualizeMode::HEIGHT:
                        insert_point3d(msg, it.get_depth(), p.x, p.y, p.z, min_v, max_v);
                        break;
                    case OctomapVisualizeMode::SEMANTICS:
                        insert_point3d_semantics<SemanticClass>(msg, it.get_depth(), p.x, p.y, p.z, it->get_semantics());
                        break;
                    case OctomapVisualizeMode::VARIANCE:
                        insert_point3d_variance(msg, it.get_depth(), p.x, p.y, p.z, min_v, max_v, it->get_vars()[it->get_semantics()]);
                        break;
                    }
                }
                else if (it->is_occluded())
                {
                    geometry_msgs::Point center;
                    center.x = p.x;
                    center.y = p.y;
                    center.z = p.z;

                    std_msgs::ColorRGBA color; // calculate color from time history
                    const float max_hist = 5000; // 5 seconds
                    std::chrono::milliseconds history = std::chrono::duration_cast<std::chrono::milliseconds>(map.get_stamp() - it->get_stamp());
                    if (history.count() > max_hist) // only show newer occluded nodes
                        continue;
                    color.r = 0.5;
                    color.g = 0.5;
                    color.b = 0.5;
                    color.a = interpolate(history.count(), 1, 0, 0.2, max_hist);

                    msg->markers[BlockDepth].points.push_back(center);
                    msg->markers[BlockDepth].colors.push_back(color);
                }
#ifndef NDEBUG
                else if (it->debug_state != 0)
                {
                    geometry_msgs::Point center;
                    center.x = p.x;
                    center.y = p.y;
                    center.z = p.z;

                    std_msgs::ColorRGBA color;
                    switch (it->debug_state)
                    {
                        case 1:
                            color.r = 1; color.g = 0; color.b = 0; color.a = 1;
                            break;
                        case 2:
                            color.r = 0; color.g = 1; color.b = 0; color.a = 1;
                            break;
                        case 3:
                            color.r = 0; color.g = 0; color.b = 1; color.a = 1;
                            break;
                        default:
                            assert(false);
                    }
                    msg->markers[BlockDepth+2].points.push_back(center);
                    msg->markers[BlockDepth+2].colors.push_back(color);
                }
#endif
            }

            for (auto bkey : map.get_occluded_blocks())
            {
                pcl::PointXYZ block_center = map.block_key_to_center(bkey);
                geometry_msgs::Point center;
                center.x = block_center.x;
                center.y = block_center.y;
                center.z = block_center.z;

                std_msgs::ColorRGBA color;
                color.r = 0.5;
                color.g = 0.5;
                color.b = 0.5;
                color.a = 0.5;

                msg->markers[BlockDepth+1].points.push_back(center);
                msg->markers[BlockDepth+1].colors.push_back(color);
            }

            _pub.publish(*msg);
        }

    private:
        void insert_point3d(visualization_msgs::MarkerArray::Ptr msg, size_t depth,
            float x, float y, float z, float min_z, float max_z) const
        {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            msg->markers[depth].points.push_back(center);
            assert (min_z < max_z);

            float h = (1.0 - std::min(std::max((z - min_z) / (max_z - min_z), 0.0f), 1.0f)) * 0.8;
            msg->markers[depth].colors.push_back(heightMapColor(h));
        }

        template <typename SemanticClass>
        void insert_point3d_semantics(visualization_msgs::MarkerArray::Ptr msg, size_t depth,
            float x, float y, float z, int c) const
        {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            msg->markers[depth].points.push_back(center);

            std_msgs::ColorRGBA color;
            RGB rgb = SemanticClass::getColor(c);
            color.r = std::get<0>(rgb);
            color.g = std::get<1>(rgb);
            color.b = std::get<2>(rgb);
            color.a = 1;
            msg->markers[depth].colors.push_back(color);
        }

        void insert_point3d_variance(visualization_msgs::MarkerArray::Ptr msg, size_t depth,
            float x, float y, float z, float min_v, float max_v, float var) const
        {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            float middle = (max_v + min_v) / 2;
            var = (var - middle) / (middle - min_v);
            msg->markers[depth].points.push_back(center);
            msg->markers[depth].colors.push_back(JetMapColor(var));
        }

        void insert_color_point3d(visualization_msgs::MarkerArray::Ptr msg, size_t depth,
            float x, float y, float z, float min_v, float max_v, float v) const
        {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            msg->markers[depth].points.push_back(center);
            assert (min_v < max_v);

            float h = (1.0 - std::min(std::max((v - min_v) / (max_v - min_v), 0.0f), 1.0f)) * 0.8f;
            msg->markers[depth].colors.push_back(heightMapColor(h));
        }

        std_msgs::ColorRGBA heightMapColor(float h) const
        {

            std_msgs::ColorRGBA color;
            color.a = 1.0;

            // blend over HSV-values (more colors)
            float s = 1.0, v = 1.0;

            h -= floor(h);
            h *= 6;
            int i = floor(h);

            float f = h - i;
            if (!(i & 1))
                f = 1 - f; // if i is even
            float m = v * (1 - s);
            float n = v * (1 - s * f);

            switch (i)
            {
            case 6:
            case 0:
                color.r = v;
                color.g = n;
                color.b = m;
                break;
            case 1:
                color.r = n;
                color.g = v;
                color.b = m;
                break;
            case 2:
                color.r = m;
                color.g = v;
                color.b = n;
                break;
            case 3:
                color.r = m;
                color.g = n;
                color.b = v;
                break;
            case 4:
                color.r = n;
                color.g = m;
                color.b = v;
                break;
            case 5:
                color.r = v;
                color.g = m;
                color.b = n;
                break;
            default:
                color.r = 1;
                color.g = 0.5;
                color.b = 0.5;
                break;
            }

            return color;
        }

        inline float interpolate(float val, float y0, float x0, float y1, float x1) const
        {
            return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
        }

        inline float base(float val) const
        {
            if (val <= -0.75)
                return 0;
            else if (val <= -0.25)
                return interpolate(val, 0.0, -0.75, 1.0, -0.25);
            else if (val <= 0.25)
                return 1.0;
            else if (val <= 0.75)
                return interpolate(val, 1.0, 0.25, 0.0, 0.75);
            else
                return 0.0;
        }

        std_msgs::ColorRGBA JetMapColor(float gray) const
        {
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = base(gray - 0.5);
            color.g = base(gray);
            color.b = base(gray + 0.5);
            return color;
        }

    private:
        ros::NodeHandle _nh;
        ros::Publisher _pub;
        std::string _frame_id;
        std::string _topic;
    };
}
