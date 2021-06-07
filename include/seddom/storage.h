#pragma once

#include <string>

#include <sqlite3.h>
#include <parallel_hashmap/phmap.h>

#include "bkioctomap.h"

namespace seddom
{
    class OctomapStorage
    {
    public:
        OctomapStorage(const std::string &database);
        ~OctomapStorage();
        void close();

        /* Check whether the parameters of octomap are consistent with ones in database */
        template <typename SemanticClass, size_t BlockDepth>
        bool check_params(const SemanticBKIOctoMap<SemanticClass, BlockDepth>& map);

        /* Synchronize database and map data */
        template <typename SemanticClass, size_t BlockDepth>
        void sync(SemanticBKIOctoMap<SemanticClass, BlockDepth>& map);

    protected:
        void exec_sql(const std::string &sql);
        bool exec_find_sql(const std::string &sql);

        uint64_t key_to_morton(BlockHashKey key);
        BlockHashKey morton_to_key(uint64_t morton);

        sqlite3 *_db;
        bool _closed;
        phmap::flat_hash_set<ChunkHashKey> _tracked_chunks;
    };
}

#include "impl/storage.hpp"
