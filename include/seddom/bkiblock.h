#pragma once

#include <array>

#include "bkioctree_node.h"
#include "bkioctree.h"

namespace seddom
{
    /*
     * @brief Block is built on top of Octree, providing the functions to locate nodes.
     *
     * Block stores the information needed to locate each OctreeNode's position:
     * fixed resolution, fixed block_size, both of which must be initialized.
     * The localization is implemented using Loop-Up Table.
     */

    template <size_t NumClass = 3, size_t MaxDepth = 0>
    class Block : public SemanticOctree<NumClass, MaxDepth>
    {
    public:
        using typename SemanticOctree<NumClass, MaxDepth>::leaf_iterator;
        using typename SemanticOctree<NumClass, MaxDepth>::const_leaf_iterator;
        using typename SemanticOctree<NumClass, MaxDepth>::DepthIndex;
        using SemanticOctree<NumClass, MaxDepth>::cell_count;
        using SemanticOctree<NumClass, MaxDepth>::leaf_count;
        using SemanticOctree<NumClass, MaxDepth>::node_count;
        using SemanticOctree<NumClass, MaxDepth>::leaf_at;
        using SemanticOctree<NumClass, MaxDepth>::begin_leaf;
        using SemanticOctree<NumClass, MaxDepth>::cbegin_leaf;
        using SemanticOctree<NumClass, MaxDepth>::end_leaf;
        using SemanticOctree<NumClass, MaxDepth>::cend_leaf;
        using SemanticOctree<NumClass, MaxDepth>::depth_index_to_index;
        using SemanticOctree<NumClass, MaxDepth>::_nodes;

    public:
        Block(float resolution) : SemanticOctree<NumClass, MaxDepth>(), _center(0.0f, 0.0f, 0.0f), _resolution(resolution) {}

        Block(pcl::PointXYZ center, float resolution) : SemanticOctree<NumClass, MaxDepth>(), _center(center), _resolution(resolution) {}

        /// @return location of the OctreeNode given Octree's leaf iterator.
        pcl::PointXYZ get_loc(const DepthIndex& depth_index) const;
        inline pcl::PointXYZ get_loc(const leaf_iterator &it) const { return get_loc(it.depth_index()); }
        inline pcl::PointXYZ get_loc(const const_leaf_iterator &it) const { return get_loc(it.depth_index()); }

        /// @return size of the OctreeNode given Octree's LeafIterator.
        inline float get_size(const DepthIndex& depth_index) const;
        inline float get_size(const leaf_iterator &it) const { return get_size(it.depth_index()); }
        inline float get_size(const const_leaf_iterator &it) const { return get_size(it.depth_index()); }

        /// @return center of current Block.
        inline pcl::PointXYZ center() const { return _center; }

        /// @return min lim of current Block.
        inline pcl::PointXYZ get_lim_min() const { return pcl::PointXYZ(_center.x - size() / 2.0f, _center.y - size() / 2.0f, _center.z - size() / 2.0f); }

        /// @return max lim of current Block.
        inline pcl::PointXYZ get_lim_max() const { return pcl::PointXYZ(_center.x + size() / 2.0f, _center.y + size() / 2.0f, _center.z + size() / 2.0f); }

        inline const_leaf_iterator search(float x, float y, float z) const
        {
            size_t i = _search_impl(x, y, z);
            return i < _nodes.size() ? leaf_at(i) : cend_leaf();
        }
        inline const_leaf_iterator search(const pcl::PointXYZ& p) const { return search(p.x, p.y, p.z); }
        inline leaf_iterator search(float x, float y, float z)
        {
            size_t i = _search_impl(x, y, z);
            return i < _nodes.size() ? leaf_at(i) : end_leaf();
        }
        inline leaf_iterator search(const pcl::PointXYZ& p) { return search(p.x, p.y, p.z); }

        template <typename Packer>
        inline void msgpack_pack(Packer &pk) const
        {
            pk.pack(static_cast<const SemanticOctree<NumClass, MaxDepth> &>(*this));
        }

        /// @return size of current Block.
        inline float size() const { return _resolution * cell_count(); }

        inline Eigen::Matrix<float, -1, 4> get_node_locs() const;

    private:
        size_t _search_impl(float x, float y, float z) const;

        float _resolution;
        pcl::PointXYZ _center;
    };
}

#include "impl/bkiblock.hpp"
