// This module supports output of gridmaps

#include <cmath>
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "bkioctomap.h"

namespace seddom
{
    const std::string GROUND_ELEVATION_LAYER = "elevation";
    const std::string GROUND_SEMANTIC_LAYER = "ground_semantic";
    const std::string CEILING_ELEVATION_LAYER = "ceiling_elevation";
    const std::string CEILING_SEMANTIC_LAYER = "ceiling_semantic";
    const std::string OCCLUTION_STATUS_LAYER = "occlusion_status";

    const float OCCSTAT_UNKNOWN = NAN;
    const float OCCSTAT_FREE = 0.;
    const float OCCSTAT_OCCUPIED = 1.;
    const float OCCSTAT_OCCLUDED = -1.;

    class HeightMapGenerator
    {
    public:
        HeightMapGenerator(
            ros::NodeHandle nh,
            std::string topic,
            std::string frame_id,
            float map_size // the size of the generated grid map in meters.
                           // The grid map will be centered at the position of query and expand in each direction by map_size/2
            ) : _topic(topic), _frame_id(frame_id), _map_size(map_size)
        {
            _pub = nh.advertise<grid_map_msgs::GridMap>(topic, 1, true);
        }
        
        template <typename SemanticClass, size_t BlockDepth>
        void publish_octomap(const SemanticBKIOctoMap<SemanticClass, BlockDepth>& map) const
        {
            pcl::PointXYZ latest_pos = map.get_position();
            grid_map::GridMap gmap({GROUND_ELEVATION_LAYER, GROUND_SEMANTIC_LAYER, CEILING_ELEVATION_LAYER, CEILING_SEMANTIC_LAYER, OCCLUTION_STATUS_LAYER});
            gmap.setFrameId(_frame_id);
            gmap.setGeometry(
                grid_map::Length(_map_size, _map_size),
                map.resolution(),
                grid_map::Position(latest_pos.x, latest_pos.y)
            );

            for (auto nit = map.cbegin_leaf(); nit != map.cend_leaf(); nit ++)
            {
                if ((nit->get_state() & State::OCCUPIED) == State::FREE)
                    continue;

                pcl::PointXYZ loc = nit.get_loc();
                grid_map::Position position(loc.x, loc.y);
                if (!gmap.isInside(position)) // skip blocks not in ROI
                    continue;

                if (nit->is_occupied())
                    if (loc.z < latest_pos.z)
                    {
                        float old_value = gmap.atPosition(GROUND_ELEVATION_LAYER, position);
                        if (std::isnan(old_value) || loc.z > old_value)
                        {
                            gmap.atPosition(GROUND_ELEVATION_LAYER, position) = loc.z;
                            gmap.atPosition(GROUND_SEMANTIC_LAYER, position) = nit->get_semantics();
                        }
                    }
                    else // loc.z >= latest_pos.z
                    {
                        float old_value = gmap.atPosition(CEILING_ELEVATION_LAYER, position);
                        if (std::isnan(old_value) || loc.z < old_value)
                        {
                            gmap.atPosition(CEILING_ELEVATION_LAYER, position) = loc.z;
                            gmap.atPosition(CEILING_SEMANTIC_LAYER, position) = nit->get_semantics();
                        }
                    }

                if (nit->is_classified())
                {
                    if (nit->is_occluded())
                        gmap.atPosition(OCCLUTION_STATUS_LAYER, position) = OCCSTAT_OCCLUDED;
                    else
                        gmap.atPosition(OCCLUTION_STATUS_LAYER, position) = nit->is_occupied() ? OCCSTAT_OCCUPIED : OCCSTAT_FREE;
                }
                // else the gridmap will be initialized as unknown, which is the expected value.
            }

            gmap.setTimestamp(std::chrono::duration_cast<std::chrono::nanoseconds>(map.get_stamp().time_since_epoch()).count());
            grid_map_msgs::GridMap gmap_msg;
            grid_map::GridMapRosConverter::toMessage(gmap, gmap_msg);
            _pub.publish(gmap_msg);
        }
    private:
        std::string _topic;
        std::string _frame_id;
        float _map_size;
        ros::Publisher _pub;
    };
}