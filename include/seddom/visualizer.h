#pragma once

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <algorithm>
#include <cmath>
#include <string>
#include "semantics.h"

namespace seddom
{

    enum OctomapVisualizeMode
    {
        HEIGHT,
        SEMANTICS,
        VARIANCE
    };

    class OctomapVisualizer
    {
    public:
        OctomapVisualizer(ros::NodeHandle nh, std::string topic, std::string frame_id = "/map") : _nh(nh),
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
            msg->markers.resize(BlockDepth);
            ros::Time ts = ros::Time::now();
            for (int i = 0; i < BlockDepth; ++i)
            {
                msg->markers[i].header.frame_id = _frame_id;
                msg->markers[i].header.stamp = ts;
                switch(Mode)
                {
                    default:
                    case OctomapVisualizeMode::HEIGHT:
                        msg->markers[i].ns = "octomap/height";
                        break;
                    case OctomapVisualizeMode::SEMANTICS:
                        msg->markers[i].ns = "octomap/semantics";
                        break;
                    case OctomapVisualizeMode::VARIANCE:
                        msg->markers[i].ns = "octomap/variance";
                        break;
                }
                msg->markers[i].id = i;
                msg->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
                msg->markers[i].scale.x = map.resolution() * pow(2, BlockDepth - 1 - i);
                msg->markers[i].scale.y = map.resolution() * pow(2, BlockDepth - 1 - i);
                msg->markers[i].scale.z = map.resolution() * pow(2, BlockDepth - 1 - i);
            }

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
                if (it->get_state() == seddom::State::OCCUPIED)
                {
                    pcl::PointXYZ p = it.get_loc();
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
