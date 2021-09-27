#pragma once

#include <stack>
#include <vector>
#include <array>
#include <type_traits>

#include "seddom/bkioctree_node.h"

namespace seddom
{
    /*
     * @brief A simple Octree to organize occupancy data in one block.
     *
     * Octree doesn't store positions of nodes in order to reduce memory usage.
     * The nodes in Octrees are indexed by OctreeHashKey which can be used to
     * retrieve positions later (See Block).
     * For the purpose of mapping, this Octree has fixed depth which should be
     * set before using Octrees.
     */
    template <size_t NumClass = 3, size_t MaxDepth = 0>
    class SemanticOctree
    {
    public:
        typedef SemanticOctreeNode<NumClass> NodeType;
        typedef std::pair<unsigned short, unsigned short> DepthIndex;

        SemanticOctree();
        SemanticOctree(const SemanticOctree &other);
        SemanticOctree &operator=(const SemanticOctree &other);

        /*
         * @brief Rursively pruning OctreeNodes with the same state.
         *
         * Prune nodes by setting nodes to PRUNED.
         * Delete the layer if all nodes are pruned.
         */
        //bool prune();

        inline NodeType &operator[](size_t index) { return _nodes[index]; }
        inline NodeType &operator[](DepthIndex depth_index) { return _nodes[depth_index_to_index(depth_index)]; }

        /// Leaf iterator for Octrees: iterate all leaf nodes not pruned.
        template <bool Constant>
        class LeafIterator
        {
            friend class SemanticOctree;

        public:
            using iterator_category = std::forward_iterator_tag;
            using value_type = NodeType;
            using reference = typename std::conditional<Constant, const value_type&, value_type&>::type;
            using pointer = typename std::remove_reference<reference>::type*;
            using difference_type = ptrdiff_t;

        public:

            LeafIterator() : _tree(nullptr), _index(-1) {}

            bool operator==(const LeafIterator &other) const
            {
                return (_tree == other._tree) && (_index == other._index);
            }
            bool operator!=(const LeafIterator &other) const { return !(operator==(other)); }

            inline LeafIterator operator++(int)
            {
                LeafIterator result(*this);
                ++(*this);
                return result;
            }
            LeafIterator &operator++()
            {
                if (++_index == _tree->_nodes.size())
                {
                    _tree = nullptr;
                    _index = -1;
                }
                return *this;
            }

            inline reference operator*() const
            {
                assert(_tree != nullptr && _index >= 0 && _index < _tree->_nodes.size());
                return const_cast<reference>(_tree->_nodes[_index]);
            }
            inline pointer operator->() const { return const_cast<pointer>(&operator*()); }

            /// @return Return the BFS index of the leaf node
            inline size_t index() const { return _index; }

            /// @return Return the depth and layer index of the leaf node
            DepthIndex depth_index() const { return SemanticOctree::index_to_depth_index(_index); }

        private:
            LeafIterator(const SemanticOctree *tree, size_t index) : _tree(tree), _index(index) {}

            const SemanticOctree *_tree;
            size_t _index;
        };
        typedef LeafIterator<false> leaf_iterator;
        typedef LeafIterator<true> const_leaf_iterator;

        inline leaf_iterator begin_leaf() { return leaf_iterator(this, (ipow8(MaxDepth - 1) - 1) / 7); };
        inline leaf_iterator end_leaf() { return leaf_iterator(); };
        inline const_leaf_iterator cbegin_leaf() const { return const_leaf_iterator(this, (ipow8(MaxDepth - 1) - 1) / 7); };
        inline const_leaf_iterator cend_leaf() const { return const_leaf_iterator(); };

        /// @return number of cells in each direction
        constexpr unsigned short cell_count() const { return 1 << (MaxDepth - 1); }
        inline size_t leaf_count() const { return ipow8(MaxDepth - 1); };
        constexpr size_t node_count() const { return _nodes.size(); }

        template <typename Packer> void msgpack_pack(Packer &pk) const;
        void msgpack_unpack(msgpack::object const& o);

        static inline DepthIndex index_to_depth_index(size_t index);
        static inline size_t depth_index_to_index(const DepthIndex& depth_index);

    protected:
        inline leaf_iterator leaf_at(size_t index) { return leaf_iterator(this, index); }
        inline const_leaf_iterator leaf_at(size_t index) const { return const_leaf_iterator(this, index); }
        std::array<NodeType, (ipow8(MaxDepth) - 1) / 7> _nodes;
    };
}

#include "impl/bkioctree.hpp"
