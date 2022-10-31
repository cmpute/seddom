// This module supports output of gridmaps

#include <cmath>
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "bkioctomap.h"

namespace seddom
{
    const std::string LAYER_GROUND_ELEVATION = "elevation"; // height of the floor block (the highest block under sensor height)
    const std::string LAYER_GROUND_SEMANTIC = "semantic"; // semantics of the floor block
    const std::string LAYER_CEILING_HEIGHT = "ceiling_height"; // height of the ceiling block (the lowest block above sensor height)
    const std::string LAYER_CEILING_SEMANTIC = "ceiling_semantic"; // semantics of the ceiling block
    const std::string LAYER_OCCLUDED_HEIGHT = "occluded_height"; // height of occluded blocks under sensor height
    const std::string LAYER_OCCLUDED_HEIGHT_FULL = "occluded_height_full"; // height of occluded blocks
    const std::string LAYER_GRID_STATUS = "status"; // state for each grid, see the constants below
    const std::string LAYER_GRID_STATUS_HISTORY = "status_history"; // time from last change in seconds

    const float STATUS_UNKNOWN = NAN;
    const float STATUS_BLOCKED = 1.;
    const float STATUS_FREE = -1.;
    const float STATUS_OCCLUDED_UNKNOWN = 0.;
    const float STATUS_OCCLUDED_BLOCKED = 2.;
    const float STATUS_OCCLUDED_FREE = -2.;

    // TODO: also calculate occlusion for dynamic objects
    // mode 1: don't accept detected objects and directly consider the measured points of the objects
    // mode 2: accept detected objects, first remove the points inside these objects, then consider the bounding boxes when calculate occlusion
    class HeightMapGenerator
    {
    public:
        HeightMapGenerator(
            ros::NodeHandle nh,
            std::string topic,
            std::string frame_id,
            float map_size, // the size of the generated grid map in meters.
                            // The grid map will be centered at the position of query and expand in each direction by map_size/2
            float map_resolution,
            OcclusionHandling occlusion_handling // whether handle unknown occlusion blocks
            ) : _topic(topic),
                _resolution(map_resolution),
                _occlusion_handling(occlusion_handling)
        {
            _pub = nh.advertise<grid_map_msgs::GridMap>(topic, 1, true);

            if (occlusion_handling == OcclusionHandling::ALL)
                _zmap = grid_map::GridMap({
                    LAYER_GROUND_ELEVATION, LAYER_GROUND_SEMANTIC,
                    LAYER_CEILING_HEIGHT, LAYER_CEILING_SEMANTIC,
                    LAYER_OCCLUDED_HEIGHT, LAYER_OCCLUDED_HEIGHT_FULL,
                    LAYER_GRID_STATUS, LAYER_GRID_STATUS_HISTORY});
            else
                _zmap = grid_map::GridMap({
                    LAYER_GROUND_ELEVATION, LAYER_GROUND_SEMANTIC,
                    LAYER_CEILING_HEIGHT, LAYER_CEILING_SEMANTIC,
                    LAYER_OCCLUDED_HEIGHT, LAYER_GRID_STATUS,
                    LAYER_GRID_STATUS_HISTORY});

            _zmap.setFrameId(frame_id);
            _zmap.setGeometry(
                grid_map::Length(map_size, map_size),
                map_resolution * 1.001
            );
        }
        
        template <typename SemanticClass, size_t BlockDepth>
        void publish_octomap(const SemanticBKIOctoMap<SemanticClass, BlockDepth>& map)
        {
            pcl::PointXYZ latest_pos = map.get_position();
            _zmap.move(grid_map::Position(latest_pos.x, latest_pos.y));
            _zmap[LAYER_OCCLUDED_HEIGHT].setConstant(NAN); // reset occlusion height every time
            if (_occlusion_handling == OcclusionHandling::ALL) _zmap[LAYER_OCCLUDED_HEIGHT_FULL].setConstant(NAN);

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

                if (nit->is_occluded()) // TODO: skip very old occluded blocks here? or directly create a new function called is_newly_occluded (to unify the parameter `max_hist` used in visualize)
                {
                    // update full occluded height
                    if (_occlusion_handling == OcclusionHandling::ALL)
                    {
                        float old_occ = _zmap.atPosition(LAYER_OCCLUDED_HEIGHT_FULL, position);
                        if (std::isnan(old_occ) || loc.z > old_occ)
                            _zmap.atPosition(LAYER_OCCLUDED_HEIGHT_FULL, position) = loc.z;
                    }

                    // update occluded height
                    if (nit->is_classified())
                    {
                        float old_occ = _zmap.atPosition(LAYER_OCCLUDED_HEIGHT, position);
                        if (std::isnan(old_occ) || loc.z > old_occ)
                            _zmap.atPosition(LAYER_OCCLUDED_HEIGHT, position) = loc.z;
                    }
                }
            }

            if (_occlusion_handling == OcclusionHandling::ALL)
            {
                for (auto bkey : map.get_occluded_blocks())
                {
                    pcl::PointXYZ loc = map.block_key_to_center(bkey);
                    grid_map::Position position(loc.x, loc.y);
                    if (!_zmap.isInside(position)) // skip blocks not in ROI
                        continue;

                    float old_occ = _zmap.atPosition(LAYER_OCCLUDED_HEIGHT_FULL, position);
                    if (std::isnan(old_occ) || loc.z > old_occ)
                        _zmap.atPosition(LAYER_OCCLUDED_HEIGHT_FULL, position) = loc.z;
                }
            }

            // TODO: also calculate the value for status_history for each grid
            for (grid_map::GridMapIterator git(_zmap); !git.isPastEnd(); ++git)
            {
                float gheight = _zmap.at(LAYER_GROUND_ELEVATION, *git);
                float oheight = _occlusion_handling == OcclusionHandling::ALL ? _zmap.at(LAYER_OCCLUDED_HEIGHT_FULL, *git) : _zmap.at(LAYER_OCCLUDED_HEIGHT, *git);
                if (std::isnan(gheight))
                {
                    if (!std::isnan(oheight))
                        _zmap.at(LAYER_GRID_STATUS, *git) = STATUS_OCCLUDED_UNKNOWN;
                    continue;
                }

                if (oheight > gheight) // this condition implies oheight is not nan
                {
                    if (gheight > latest_pos.z)
                        _zmap.at(LAYER_GRID_STATUS, *git) = STATUS_OCCLUDED_BLOCKED;
                    else
                        _zmap.at(LAYER_GRID_STATUS, *git) = STATUS_OCCLUDED_FREE;
                }
                else
                {
                    if (gheight > latest_pos.z)
                        _zmap.at(LAYER_GRID_STATUS, *git) = STATUS_BLOCKED;
                    else
                        _zmap.at(LAYER_GRID_STATUS, *git) = STATUS_FREE;
                }
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
        OcclusionHandling _occlusion_handling;
    };
}