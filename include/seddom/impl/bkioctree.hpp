#pragma once

#include <cmath>

#include "bkioctree.h"

#define OCTOMAP_OCTREE_TDECL template <size_t NumClass, size_t MaxDepth>
#define OCTOMAP_OCTREE_CLASS SemanticOctree<NumClass, MaxDepth>

namespace seddom
{
    OCTOMAP_OCTREE_TDECL
    OCTOMAP_OCTREE_CLASS::SemanticOctree() : _nodes() {}

    OCTOMAP_OCTREE_TDECL
    OCTOMAP_OCTREE_CLASS::SemanticOctree(const OCTOMAP_OCTREE_CLASS &other)
    {
        _nodes = other._nodes;
    }

    OCTOMAP_OCTREE_TDECL
    OCTOMAP_OCTREE_CLASS &OCTOMAP_OCTREE_CLASS::operator=(const OCTOMAP_OCTREE_CLASS &other)
    {
        _nodes = other._nodes;
        return *this;
    }

    OCTOMAP_OCTREE_TDECL inline typename OCTOMAP_OCTREE_CLASS::DepthIndex
    OCTOMAP_OCTREE_CLASS::index_to_depth_index(size_t index)
    {
        if (index == 0)
            return std::make_pair(0, 0);

        unsigned short depth = 0, k = index * 7 + 1;
        while (k >= 8)
        {
            k >>= 3;
            depth++;
        }

        unsigned short dindex = index - (ipow8(depth) - 1) / 7;
        return std::make_pair(depth, dindex);
    }

    OCTOMAP_OCTREE_TDECL inline size_t
    OCTOMAP_OCTREE_CLASS::depth_index_to_index(const typename OCTOMAP_OCTREE_CLASS::DepthIndex& depth_index)
    {
        return (ipow8(depth_index.first) - 1) / 7 + depth_index.second;
    }
    
    OCTOMAP_OCTREE_TDECL template <typename Packer> 
    void
    OCTOMAP_OCTREE_CLASS::msgpack_pack(Packer &pk) const
    {
        pk.pack_array(_nodes.size());
        for (size_t i = 0; i < _nodes.size(); ++i)
            pk.pack(_nodes[i]);
    }
}

#undef OCTOMAP_OCTREE_TDECL
#undef OCTOMAP_OCTREE_CLASS
