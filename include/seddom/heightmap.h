// This module supports output of gridmaps

#include <cmath>
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "bkioctomap.h"

namespace seddom
{
    const std::string LAYER_GROUND_ELEVATION = "elevation";
    const std::string LAYER_GROUND_SEMANTIC = "semantic";
    const std::string LAYER_CEILING_HEIGHT = "ceiling_height";
    const std::string LAYER_CEILING_SEMANTIC = "ceiling_semantic";
    const std::string LAYER_OCCLUDED_HEIGHT = "occluded_height";
    const std::string LAYER_GRID_STATUS = "status";

    const float STATUS_UNKNOWN = NAN;
    const float STATUS_BLOCKED = 1.;
    const float STATUS_FREE = 0.;
    const float STATUS_OCCLUDED = -1.;

    // TODO: also calculate occlusion for dynamic objects, maybe a separate node tracking objects and generating occluded area based on detections?
    class HeightMapGenerator
    {
    public:
        HeightMapGenerator(
            ros::NodeHandle nh,
            std::string topic,
            std::string frame_id,
            float map_size, // the size of the generated grid map in meters.
                            // The grid map will be centered at the position of query and expand in each direction by map_size/2
            float map_resolution
            ) : _topic(topic),
                _resolution(map_resolution),
                _zmap({LAYER_GROUND_ELEVATION, LAYER_GROUND_SEMANTIC, LAYER_CEILING_HEIGHT, LAYER_CEILING_SEMANTIC, LAYER_OCCLUDED_HEIGHT, LAYER_GRID_STATUS})
        {
            _pub = nh.advertise<grid_map_msgs::GridMap>(topic, 1, true);
            _zmap.setFrameId(frame_id);
            _zmap.setGeometry(
                grid_map::Length(map_size, map_size),
                map_resolution * 1.05
            );
        }
        
        template <typename SemanticClass, size_t BlockDepth>
        void publish_octomap(const SemanticBKIOctoMap<SemanticClass, BlockDepth>& map)
        {
            pcl::PointXYZ latest_pos = map.get_position();
            _zmap.move(grid_map::Position(latest_pos.x, latest_pos.y));
            _zmap[LAYER_OCCLUDED_HEIGHT].setConstant(NAN); // reset occlusion height every time

            for (auto nit = map.cbegin_leaf(); nit != map.cend_leaf(); nit ++)
            {
                pcl::PointXYZ loc = nit.get_loc();
                grid_map::Position position(loc.x, loc.y);
                if (!_zmap.isInside(position)) // skip blocks not in ROI
                    continue;

                if (nit->is_occupied())
                    if (loc.z < (latest_pos.z + _resolution)) // consider one more block
                    {
                        // update ground height if the block is lower than sensor height
                        float old_value = _zmap.atPosition(LAYER_GROUND_ELEVATION, position);
                        if (std::isnan(old_value) || loc.z > old_value)
                        {
                            _zmap.atPosition(LAYER_GROUND_ELEVATION, position) = loc.z;
                            _zmap.atPosition(LAYER_GROUND_SEMANTIC, position) = nit->get_semantics();
                        }

                        // occluded height is at least ground height
                        float old_occ = _zmap.atPosition(LAYER_OCCLUDED_HEIGHT, position);
                        if (std::isnan(old_occ) || loc.z > old_occ)
                            _zmap.atPosition(LAYER_OCCLUDED_HEIGHT, position) = loc.z;
                    }
                    if (loc.z > (latest_pos.z - _resolution))
                    {
                        // update ceiling height if the block is higher than sensor height
                        float old_value = _zmap.atPosition(LAYER_CEILING_HEIGHT, position);
                        if (std::isnan(old_value) || loc.z < old_value)
                        {
                            _zmap.atPosition(LAYER_CEILING_HEIGHT, position) = loc.z;
                            _zmap.atPosition(LAYER_CEILING_SEMANTIC, position) = nit->get_semantics();
                        }
                    }

                if (nit->is_classified() && nit->is_occluded())
                {
                    float old_occ = _zmap.atPosition(LAYER_OCCLUDED_HEIGHT, position);
                    if (std::isnan(old_occ) || loc.z > old_occ)
                        _zmap.atPosition(LAYER_OCCLUDED_HEIGHT, position) = loc.z;
                }
            }

            for (grid_map::GridMapIterator git(_zmap); !git.isPastEnd(); ++git)
            {
                float gheight = _zmap.at(LAYER_GROUND_ELEVATION, *git);
                if (std::isnan(gheight))
                    continue;

                if (gheight > latest_pos.z)
                    _zmap.at(LAYER_GRID_STATUS, *git) = STATUS_BLOCKED;
                else if (_zmap.at(LAYER_OCCLUDED_HEIGHT, *git) > gheight)
                    _zmap.at(LAYER_GRID_STATUS, *git) = STATUS_OCCLUDED;
                else // occlusion height < ground height < sensor height
                    _zmap.at(LAYER_GRID_STATUS, *git) = STATUS_FREE;
            }

            _zmap.setTimestamp(ros::Time::now().toNSec());
            grid_map_msgs::GridMap zmap_msg;
            grid_map::GridMapRosConverter::toMessage(_zmap, zmap_msg);
            _pub.publish(zmap_msg);
        }
    private:
        std::string _topic;
        float _resolution;
        ros::Publisher _pub;
        grid_map::GridMap _zmap;
    };
}