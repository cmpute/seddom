#pragma once

#include <queue>
#include <algorithm>
#include <stdexcept>

#include "bkiblock.h"

#define OCTOMAP_BLOCK_TDECL template <size_t NumClass, size_t MaxDepth>
#define OCTOMAP_BLOCK_CLASS Block<NumClass, MaxDepth>

namespace seddom
{
    OCTOMAP_BLOCK_TDECL pcl::PointXYZ
    OCTOMAP_BLOCK_CLASS::get_loc(const typename OCTOMAP_BLOCK_CLASS::DepthIndex& depth_index) const
    {
        int ix = 0, iy = 0, iz = 0;
        int d = depth_index.first, c = depth_index.second, scale = 1;
        while (d > 0)
        {
            int rel_pos = c % 8;
            ix += scale * (rel_pos & 4 ? 1 : -1);
            iy += scale * (rel_pos & 2 ? 1 : -1);
            iz += scale * (rel_pos & 1 ? 1 : -1);

            c = c / 8;
            d--;
            scale *= 2;
        }

        float x = _resolution / 2 * ix + _center.x;
        float y = _resolution / 2 * iy + _center.y;
        float z = _resolution / 2 * iz + _center.z;
        return pcl::PointXYZ(x, y, z);
    }
    
    OCTOMAP_BLOCK_TDECL inline float
    OCTOMAP_BLOCK_CLASS::get_size(const typename OCTOMAP_BLOCK_CLASS::DepthIndex& depth_index) const
    {
        return _resolution * (1 << (MaxDepth - 1 - depth_index.first));
    }

    OCTOMAP_BLOCK_TDECL size_t
    OCTOMAP_BLOCK_CLASS::_search_impl(float x, float y, float z) const
    {
        int xx = static_cast<int>((x - _center.x) / _resolution + cell_count() / 2);
        int yy = static_cast<int>((y - _center.y) / _resolution + cell_count() / 2);
        int zz = static_cast<int>((z - _center.z) / _resolution + cell_count() / 2);

        assert (xx > 0 && xx < cell_count() && yy > 0 && yy < cell_count() && zz > 0 && zz < cell_count());
        unsigned short ix = static_cast<unsigned short>(xx);
        unsigned short iy = static_cast<unsigned short>(yy);
        unsigned short iz = static_cast<unsigned short>(zz);

        // TODO: this part is basically calculating Morton code
        unsigned short index = 0;
        for (int i = 0; i < MaxDepth; i++)
        {
            unsigned short p = 0;
            p |= (ix & 1) << 2;
            p |= (iy & 1) << 1;
            p |= (iz & 1);
            ix >>= 1;
            iy >>= 1;
            iz >>= 1;
            index |= (p << (i * 3));
        }

        auto depth_index = std::make_pair(static_cast<unsigned short>(MaxDepth - 1), index);
        return OCTOMAP_BLOCK_CLASS::depth_index_to_index(depth_index);
    }

    OCTOMAP_BLOCK_TDECL Eigen::Matrix<float, -1, 4>
    OCTOMAP_BLOCK_CLASS::get_node_locs() const
    {
        Eigen::Matrix<float, -1, 4> xs(leaf_count(), 4);
        size_t r = 0;
        for (auto leaf_it = cbegin_leaf(); leaf_it != cend_leaf(); ++leaf_it, ++r)
        {
            assert(r < xs.rows());
            pcl::PointXYZ p = get_loc(leaf_it);
            xs.row(r) = p.getVector4fMap();
        }
        return xs;
    }
}

#undef OCTOMAP_BLOCK_TDECL
#undef OCTOMAP_BLOCK_CLASS
