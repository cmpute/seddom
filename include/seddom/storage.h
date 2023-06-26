#pragma once

#include <string>

#include <sqlite3.h>
#include <parallel_hashmap/phmap.h>

#include "bkioctomap.h"
#include "bkioctree_node.h"

namespace seddom
{
    class OctomapStorage
    {
    public:
        /*
         * active_range: max range for sensor, if sensor can produce measurement beyond this range,
         *               unintended behavior will happen.
         */
        OctomapStorage(const std::string &database, float active_range, bool read_only);
        ~OctomapStorage();
        void close();

        /* Check whether the parameters of octomap are consistent with ones in database */
        template <typename SemanticClass, size_t BlockDepth>
        bool check_params(const SemanticBKIOctoMap<SemanticClass, BlockDepth> &map);

        /* Synchronize database and map data */
        template <typename SemanticClass, size_t BlockDepth>
        void sync(SemanticBKIOctoMap<SemanticClass, BlockDepth> &map);
        template <typename SemanticClass, size_t BlockDepth>
        void load_around(SemanticBKIOctoMap<SemanticClass, BlockDepth> &map, const pcl::PointXYZ &center);
        template <typename SemanticClass, size_t BlockDepth>
        void dump_all(SemanticBKIOctoMap<SemanticClass, BlockDepth> &map);

        int get_size() const;

    protected:
        void exec_sql(const std::string &sql);
        bool exec_find_sql(const std::string &sql);
        inline void assert_ok(int code) const;

        template <typename SemanticClass, size_t BlockDepth>
        void load_chunk(SemanticBKIOctoMap<SemanticClass, BlockDepth> &map, ChunkHashKey key, const std::string &table_name);
        template <typename SemanticClass, size_t BlockDepth>
        void dump_chunk(SemanticBKIOctoMap<SemanticClass, BlockDepth> &map, ChunkHashKey key, const std::string &table_name);

        /* Get the table name given map origin and create table if not existed */
        template <typename SemanticClass, size_t BlockDepth>
        std::string check_table(SemanticBKIOctoMap<SemanticClass, BlockDepth> &map);

        uint64_t key_to_morton(BlockHashKey key);
        BlockHashKey morton_to_key(uint64_t morton);

        sqlite3 *_db;
        bool _closed;
        bool _read_only;
        float _active_range;
        phmap::flat_hash_set<ChunkHashKey> _tracked_chunks;
        SaveOptions _save_options;
    };
}

#include "impl/storage.hpp"
