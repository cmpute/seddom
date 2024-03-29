#pragma once

#include <vector>
#include <mutex>
#include <chrono>

#include <parallel_hashmap/phmap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <EigenRand/EigenRand>

#include "seddom/macros.h"
#include "seddom/bkiblock.h"
#include "seddom/bki.h"

namespace seddom
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
    typedef pcl::PointCloud<pcl::PointXYZL> PointCloudXYZL;
    template <typename K, typename V>
    using ParallelMap = phmap::parallel_node_hash_map<K, V,
                                                      phmap::priv::hash_default_hash<K>,
                                                      phmap::priv::hash_default_eq<K>,
                                                      std::allocator<std::pair<const K, V>>,
                                                      4, std::mutex>;
    template <typename T>
    using ParallelSet = phmap::parallel_flat_hash_set<T,
                                                      phmap::priv::hash_default_hash<T>,
                                                      phmap::priv::hash_default_eq<T>,
                                                      std::allocator<T>,
                                                      4, std::mutex>;

    typedef uint64_t BlockHashKey;
    typedef uint64_t ChunkHashKey;

    enum class OcclusionHandling
    {
        NONE,
        KNOWN,
        ALL
    };

    /*
     * @brief BGKOctoMap
     *
     * Bayesian Generalized Kernel Inference for Occupancy Map Prediction
     * The space is partitioned by Blocks in which Octrees with fixed
     * depth are rooted. Occupancy values in one Block is predicted by 
     * its ExtendedBlock via Bayesian generalized kernel inference.
     * 
     * BlockDepth: Max block depth
     */
    template <typename SemanticClass, size_t BlockDepth = 4>
    class SemanticBKIOctoMap
    {
    public:
        /// Hask key to index Block given block's center.
        static constexpr size_t NumClass = SemanticClass::NumClass;
        typedef Block<NumClass, BlockDepth> BlockType;
        typedef SemanticOctreeNode<NumClass> NodeType;
        typedef ParallelMap<BlockHashKey, BlockType> BlockMap;
        typedef ParallelSet<ChunkHashKey> BlockSet;
        typedef ParallelSet<ChunkHashKey> ChunkSet; // Chunk is used for map dumping and efficient range iterating
        static_assert(BlockDepth > 0);

        friend class OctomapStorage;

/// Extended Block. TODO: this is related to ell and block size, we could make this automatically determined
#ifdef EXPAND_PREDICTION
        typedef std::array<BlockHashKey, 27> ExtendedBlock;
#else
        typedef std::array<BlockHashKey, 7> ExtendedBlock;
#endif

    public:
        SemanticBKIOctoMap() : SemanticBKIOctoMap(
                                   OcclusionHandling::NONE, // process occlusion
                                   0.1f,  // resolution
                                   6,     // chunk_depth
                                   1.0,   // sf2
                                   1.0,   // ell
                                   1.0f,  // prior
                                   -1,    // max_range
                                   false  // read_only
                               )
        {
        }

        /*
         * @param occlusion_aware process occluded blocks or not
         * @param resolution (default 0.1m)
         * @param block_depth maximum depth of Octree (default 4)
         * @param sf2 signal variance in GPs (default 1.0)
         * @param ell length-scale in GPs (default 1.0)
         * @param prior prior value for categories
         * @param max_range max sensor range. This is responsible for cropping training data
         *                  max_range <= 0 means no limit
         */
        SemanticBKIOctoMap(
            OcclusionHandling occlusion_handling,
            float resolution,
            size_t chunk_depth,
            float sf2,
            float ell,
            float prior,
            float max_range,
            bool read_only);

        inline float resolution() const { return _resolution; }
        inline float block_depth() const { return BlockDepth; }
        inline float block_size() const { return _block_size; }
        inline size_t block_count() const { return _blocks.size(); }
        inline float chunk_depth() const { return _chunk_depth; }
        inline float chunk_size() const { return _chunk_size; }
        inline size_t chunk_count() const { return _chunks.size(); }
        inline size_t memory_size() const
        {
            size_t result = sizeof(*this);
            result += _blocks.size() * (sizeof(BlockType) + sizeof(BlockHashKey));
            result += _chunks.size() * (sizeof(ChunkHashKey));
            return result;
        }
        inline pcl::PointXYZ get_map_origin() const { return _map_origin; }
        inline void set_map_origin(const pcl::PointXYZ &origin) { _map_origin = origin; }

        /// LeafIterator for iterating all leaf nodes in blocks
        template <bool Constant>
        class LeafIterator
        {
            friend class SemanticBKIOctoMap;

        public:
            using iterator_category = std::forward_iterator_tag;
            using value_type = NodeType;
            using reference = typename std::conditional<Constant, const value_type &, value_type &>::type;
            using pointer = typename std::remove_reference<reference>::type *;
            using difference_type = ptrdiff_t;

        public:
            inline bool operator==(const LeafIterator &other)
            {
                return (block_it == other.block_it) && (leaf_it == other.leaf_it);
            }

            inline bool operator!=(const LeafIterator &other) { return !(this->operator==(other)); }

            inline LeafIterator operator++(int)
            {
                LeafIterator result(*this);
                ++(*this);
                return result;
            }

            LeafIterator &operator++();

            inline reference operator*() const { return const_cast<reference>(*leaf_it); }
            inline pointer operator->() const { return const_cast<pointer>(leaf_it.operator->()); }

            inline pcl::PointXYZ get_loc() const { return block_it->second.get_loc(leaf_it); }
            inline float get_size() const { return block_it->second.get_size(leaf_it); }
            inline uint16_t get_depth() const { return leaf_it.depth_index().first; }

        private:
            LeafIterator(const SemanticBKIOctoMap *map);
            LeafIterator(const SemanticBKIOctoMap *map,
                         typename BlockMap::const_iterator block_it,
                         typename BlockType::const_leaf_iterator leaf_it)
                : block_it(block_it), leaf_it(leaf_it), end_block(map->_blocks.cend()), end_leaf(typename BlockType::const_leaf_iterator()) {}

            typename BlockMap::const_iterator block_it;
            typename BlockMap::const_iterator end_block;

            typename BlockType::const_leaf_iterator leaf_it;
            typename BlockType::const_leaf_iterator end_leaf;
        };

        typedef LeafIterator<false> leaf_iterator;
        typedef LeafIterator<true> const_leaf_iterator;

        inline leaf_iterator begin_leaf() { return leaf_iterator(this); }
        inline leaf_iterator end_leaf() { return leaf_iterator(this, _blocks.cend(), typename BlockType::const_leaf_iterator()); }
        inline const_leaf_iterator cbegin_leaf() const { return const_leaf_iterator(this); }
        inline const_leaf_iterator cend_leaf() const { return const_leaf_iterator(this, _blocks.cend(), typename BlockType::const_leaf_iterator()); }

        /*
         * @brief Insert PCL PointCloud into BGKOctoMaps.
         * @param cloud one scan in PointCloudXYZL format
         * @param origin sensor origin in the scan
         * @param ds_resolution downsampling resolution for PCL VoxelGrid filtering (-1 if no downsampling)
         * @param free_res resolution for sampling free training points along sensor beams
         */
        template <KernelType KType>
        void insert_pointcloud(PointCloudXYZL::ConstPtr cloud, const pcl::PointXYZ &origin,
                               std::chrono::system_clock::time_point timestamp,
                               float ds_resolution, float free_resolution);
        template <KernelType KType>
        void insert_pointcloud(PointCloudXYZL::ConstPtr cloud, const pcl::PointXYZ &origin,
                               float ds_resolution, float free_resolution)
        {
            insert_pointcloud<KType>(cloud, origin, std::chrono::system_clock::now(), ds_resolution, free_resolution);
        }
        // TODO: support non-labelled point cloud input, support label+score input and support label+score array input

        /*
         * @brief Insert PCL PointCloud into BGKOctoMaps.
         * @param cloud one scan in PointCloudXYZL format
         * @param origin sensor origin in the scan
         * @param ds_resolution downsampling resolution for PCL VoxelGrid filtering (-1 if no downsampling)
         * @param samples_per_beam number of randomly sampled free training points along sensor beams
         */
        template <KernelType KType>
        void insert_pointcloud(PointCloudXYZL::ConstPtr cloud, const pcl::PointXYZ &origin,
                               std::chrono::system_clock::time_point timestamp,
                               float ds_resolution, int samples_per_beam = 0);
        template <KernelType KType>
        void insert_pointcloud(PointCloudXYZL::ConstPtr cloud, const pcl::PointXYZ &origin,
                               float ds_resolution, int samples_per_beam = 0)
        {
            insert_pointcloud<KType>(cloud, origin, std::chrono::system_clock::now(), ds_resolution, samples_per_beam);
        }

        /*
         * @brief Insert PCL PointCloud into BGKOctoMaps, with point-line distance support.
         */
        template <KernelType KType>
        void insert_pointcloud_pl(PointCloudXYZL::ConstPtr cloud, const pcl::PointXYZ &origin,
                                  std::chrono::system_clock::time_point timestamp, float ds_resolution);
        template <KernelType KType>
        void insert_pointcloud_pl(PointCloudXYZL::ConstPtr cloud, const pcl::PointXYZ &origin, float ds_resolution)
        {
            insert_pointcloud_pl<KType>(cloud, origin, std::chrono::system_clock::now(), ds_resolution);
        }

        size_t dump_map(const std::string &path) const;

        /// Get bounding box of the map.
        void get_bbox(pcl::PointXYZ &lim_min, pcl::PointXYZ &lim_max) const;

        /// Convert from hash key to block.
        inline pcl::PointXYZ block_key_to_center(BlockHashKey key) const;
        inline pcl::PointXYZ chunk_key_to_center(ChunkHashKey key) const;

        /// Convert from block to hash key.
        inline BlockHashKey loc_to_block_key(pcl::PointXYZ center) const { return loc_to_block_key(center.x, center.y, center.z); }
        inline BlockHashKey loc_to_block_key(pcl::PointXYZL center) const { return loc_to_block_key(center.x, center.y, center.z); }
        inline BlockHashKey loc_to_block_key(float x, float y, float z) const { return loc_to_block_key(x, y, z, _block_size); }
        inline BlockHashKey loc_to_block_key(float x, float y, float z, float block_size) const;
        inline ChunkHashKey block_to_chunk_key(BlockHashKey key) const;

        /// Get current block's extended block.
        ExtendedBlock get_extended_block(BlockHashKey key) const;

        const_leaf_iterator search(float x, float y, float z) const;
        inline const_leaf_iterator search(pcl::PointXYZ p) const { return search(p.x, p.y, p.z); }
        leaf_iterator search(float x, float y, float z);
        inline leaf_iterator search(pcl::PointXYZ p) { return search(p.x, p.y, p.z); }

        std::string summary() const;

        std::vector<BlockHashKey> get_occluded_blocks() const { return std::vector<BlockHashKey>(_occluded_blocks.cbegin(), _occluded_blocks.cend()); }
        // get the latest timestamp of the data
        std::chrono::system_clock::time_point get_stamp() const { return _latest_time; }
        // get the latest position of the sensor
        pcl::PointXYZ get_position() const { return _latest_position; }

    private:
        template <KernelType KType>
        void inference_points(const PointCloudXYZL::Ptr training_data, std::chrono::system_clock::time_point timestamp);

        /// Get all block indices inside a bounding box.
        std::vector<BlockHashKey> get_blocks_in_bbox(const pcl::PointXYZ &lim_min, const pcl::PointXYZ &lim_max) const;
        std::vector<BlockHashKey> get_blocks_in_bbox(const pcl::PointXYZ &lim_min, const pcl::PointXYZ &lim_max, const pcl::PointXYZ &origin) const;
        std::vector<BlockHashKey> get_mapped_blocks_in_bbox(const pcl::PointXYZ &lim_min, const pcl::PointXYZ &lim_max) const;
        std::vector<BlockHashKey> get_mapped_blocks_in_bbox(const pcl::PointXYZ &lim_min, const pcl::PointXYZ &lim_max, const pcl::PointXYZ &origin) const;

        /// Downsample point cloud using PCL VoxelGrid Filtering.
        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr downsample(typename pcl::PointCloud<PointT>::ConstPtr in, float ds_resolution) const;

        /// Sample free training points along sensor beams.
        PointCloudXYZ::Ptr beam_sample(const pcl::PointXYZL &hit, const pcl::PointXYZ &origin, float free_resolution) const;
        PointCloudXYZ::Ptr beam_sample_random(const pcl::PointXYZL &hit, const pcl::PointXYZ &origin, int samples_per_beam);

        /// Get training data from one sensor scan.
        PointCloudXYZL::Ptr get_training_data(PointCloudXYZL::ConstPtr cloud, const pcl::PointXYZ &origin,
                                              float ds_resolution, float free_resolution) const;
        PointCloudXYZL::Ptr get_random_training_data(PointCloudXYZL::ConstPtr cloud, const pcl::PointXYZ &origin,
                                                     float ds_resolution, int samples_per_beam);

        OcclusionHandling _occlusion_handling;
        bool _read_only;
        float _resolution;
        float _block_size;
        size_t _chunk_depth;
        float _chunk_size;
        float _max_range;
        float _min_range; // filter out points that are too close
        size_t _max_beams;

        float _sf2;
        float _ell; // length-scale

        BlockMap _blocks;
        BlockSet _occluded_blocks; // this set only stores occluded blocks that are not present in _blocks, and it's only effective for one frame
                                   // TODO: add flag to ignore these addtional occluded blocks
        ChunkSet _chunks;

        Eigen::Rand::Vmt19937_64 _rng;
        pcl::PointXYZ _map_origin;
        pcl::PointXYZ _latest_position;
        std::chrono::system_clock::time_point _latest_time;
        // TODO: update from new measurement or database, otherwise initialize as 0 (from utc start)
        // TODO: also store latest time for each block

        // TODO: add boost::signal for integration start and complete
        //       completion signal could be used for visualization update, map dump and load, 2D map generation
        //       start signal could be used to stop above operations (and continue after completion)
    };

}

#include "impl/bkioctomap.hpp"
