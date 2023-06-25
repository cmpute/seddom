#include <vector>
#include <utility>
#include <stdexcept>
#include <sstream>
#include <typeinfo>

#include <morton-nd/mortonND_BMI2.h>
#include <msgpack.hpp>

#include "storage.h"

namespace seddom
{
    class sqlite_exception : public std::runtime_error
    {
    public:
        sqlite_exception(const char *msg, std::string sql, int code = -1) : runtime_error(msg), code(code), sql(sql) {}
        sqlite_exception(int code, std::string sql) : runtime_error(sqlite3_errstr(code)), code(code), sql(sql) {}
        int get_code() const { return code & 0xFF; }
        int get_extended_code() const { return code; }
        std::string get_sql() const { return sql; }

    private:
        int code;
        std::string sql;
    };

    // struct membuf : std::streambuf
    // {
    //     membuf(const void* begin, int n)
    //     {
    //         // MAKE SURE THE CLASS IS ONLY USED FOR ISTREAM
    //         char* cur = const_cast<char*>(static_cast<const char*>(begin));
    //         this->setg(cur, cur, cur + n);
    //     }
    // };

    OctomapStorage::OctomapStorage(const std::string &database, float active_range)
        : _closed(false), _active_range(active_range), _save_options(SaveOptions::ALL_BLOCKS)
    {
        assert_ok(sqlite3_open(database.c_str(), &_db));
        exec_sql("PRAGMA page_size=16384;");
        exec_sql("PRAGMA journal_mode=WAL;");
    }

    OctomapStorage::~OctomapStorage()
    {
        close();
    }

    void OctomapStorage::close()
    {
        if (!_closed)
        {
            sqlite3_close(_db);
            _closed = true;
        }
    }

    inline void OctomapStorage::assert_ok(int code) const
    {
        if (code != SQLITE_OK)
            throw sqlite_exception(code, "");
    }

    void OctomapStorage::exec_sql(const std::string &sql)
    {
        char *errmsg;
        int ec = sqlite3_exec(_db, sql.c_str(), NULL, NULL, &errmsg);
        if (ec != SQLITE_OK)
            throw sqlite_exception(errmsg, sql, ec);
    }

    bool OctomapStorage::exec_find_sql(const std::string &sql)
    {
        sqlite3_stmt *stmt;
        int ec;
        assert_ok(sqlite3_prepare_v2(_db, sql.c_str(), -1, &stmt, NULL));
        int step_result = sqlite3_step(stmt);
        ec = sqlite3_finalize(stmt);
        if (ec != SQLITE_OK)
            throw sqlite_exception(ec, sql);

        if (step_result == SQLITE_DONE)
            return false;
        else if (step_result == SQLITE_ROW)
            return true;
        else
            throw sqlite_exception(step_result, sql);
    }

    template <typename SemanticClass, size_t BlockDepth>
    bool OctomapStorage::check_params(const SemanticBKIOctoMap<SemanticClass, BlockDepth> &map)
    {
        sqlite3_stmt *stmt;
        int ec;
        bool meta_exists = exec_find_sql("SELECT 1 FROM sqlite_master WHERE type='table' AND name='meta';");
        if (meta_exists)
        {
            // read params
            const auto compare_param = [this, stmt, ec](const std::string &key, const auto &value) -> bool
            {
                std::stringstream ss;
                ss << "SELECT 1 FROM meta WHERE key='" << key << "' AND value='" << value << "';";
                return exec_find_sql(ss.str());
            };
            if (!compare_param("block_depth", BlockDepth))
                return false;
            if (!compare_param("chunk_depth", map.chunk_depth()))
                return false;
            if (!compare_param("resolution", map.resolution()))
                return false;
            if (!compare_param("semantic_class", typeid(SemanticClass).name()))
                return false;
            if (!compare_param("semantic_count", (size_t)(SemanticClass::NumClass)))
                return false;
            return true;
        }
        else
        {
            // create table
            const std::string sql_create_table = "CREATE TABLE meta ("
                                                 "key TEXT NOT NULL UNIQUE, value);";
            exec_sql(sql_create_table);

            // insert params
            const auto insert_param = [this](const std::string &key, const auto &value)
            {
                std::stringstream ss;
                ss << "INSERT INTO meta (key, value) VALUES ('" << key << "', '" << value << "');";
                exec_sql(ss.str());
            };
            insert_param("block_depth", BlockDepth);
            insert_param("chunk_depth", map.chunk_depth());
            insert_param("resolution", map.resolution());
            insert_param("semantic_class", typeid(SemanticClass).name());
            insert_param("semantic_count", (size_t)(SemanticClass::NumClass));
            return true;
        }
    }

    uint64_t OctomapStorage::key_to_morton(uint64_t key)
    {
        uint64_t ix = key >> 40, iy = (key >> 20) & 0xFFFFF, iz = key & 0xFFFFF;
        return mortonnd::MortonNDBmi_3D_64::Encode(ix, iy, iz);
    }
    uint64_t OctomapStorage::morton_to_key(uint64_t morton)
    {
        uint64_t ix, iy, iz;
        std::tie(ix, iy, iz) = mortonnd::MortonNDBmi_3D_64::Decode(morton);
        return (ix << 40) | (iy << 20) | iz;
    }

    template <typename SemanticClass, size_t BlockDepth>
    void OctomapStorage::load_chunk(SemanticBKIOctoMap<SemanticClass, BlockDepth> &map, ChunkHashKey key, const std::string &table_name)
    {
        sqlite3_stmt *stmt;
        const std::string sql_query_chunk = "SELECT hashkey, last_update, data FROM '" + table_name +
                                            "' WHERE hashkey >= ? AND hashkey < ? ;";

        size_t chunk_bits = 3 * (map._chunk_depth - 1);
        const uint64_t morton = key_to_morton(key);
        // DEBUG_WRITE("Loading chunk " << key << " (morton: " << morton << ")");
        assert_ok(sqlite3_prepare_v2(_db, sql_query_chunk.c_str(), -1, &stmt, NULL));
        assert_ok(sqlite3_bind_int64(stmt, 1, morton << chunk_bits));
        assert_ok(sqlite3_bind_int64(stmt, 2, (morton + 1) << chunk_bits));
        int step_result = sqlite3_step(stmt);
        size_t latest_timestamp = 0;
        if (step_result == SQLITE_ROW)
        {
            do
            {
                BlockHashKey key = morton_to_key(sqlite3_column_int64(stmt, 0));
                size_t timestamp = sqlite3_column_int64(stmt, 1);
                latest_timestamp = std::max(timestamp, latest_timestamp);
                int n = sqlite3_column_bytes(stmt, 2);
                const char *cur = reinterpret_cast<const char *>(sqlite3_column_blob(stmt, 2));
                msgpack::object_handle oh = msgpack::unpack(cur, n);

                auto block_iter = map._blocks.find(key);
                if (block_iter == map._blocks.end())
                    block_iter = map._blocks.emplace(key,
                                                     Block<SemanticClass::NumClass, BlockDepth>(
                                                         map.block_key_to_center(key), map._resolution))
                                     .first;
                oh.get().convert(block_iter->second);
            } while ((step_result = sqlite3_step(stmt)) == SQLITE_ROW);
        }
        assert_ok(sqlite3_finalize(stmt));
        if (map._chunks.find(key) == map._chunks.end())
            map._chunks.insert(key);
        _tracked_chunks.insert(key);

        std::chrono::system_clock::time_point latest_time = std::chrono::system_clock::time_point(std::chrono::milliseconds(latest_timestamp));
        if (map._latest_time < latest_time)
            map._latest_time = latest_time;
    }

    template <typename SemanticClass, size_t BlockDepth>
    void OctomapStorage::dump_chunk(SemanticBKIOctoMap<SemanticClass, BlockDepth> &map, ChunkHashKey key, const std::string &table_name)
    {
        // pre-calculate keys, TODO: move this step out sync loop to speed up
        std::vector<std::pair<BlockHashKey, ChunkHashKey>> blocks;
        blocks.reserve(map._blocks.size());
        for (const auto &it : map._blocks)
            blocks.emplace_back(it.first, map.block_to_chunk_key(it.first));

        sqlite3_stmt *stmt;
        const std::string sql_insert_block = "REPLACE INTO '" + table_name +
                                             "' (hashkey, last_update, data) VALUES(?,?,?);";

        // DEBUG_WRITE("Dumping chunk " << key);
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(map._latest_time.time_since_epoch()).count();
        assert_ok(sqlite3_prepare_v2(_db, sql_insert_block.c_str(), -1, &stmt, NULL));

        int ec;
        for (const auto &kit : blocks)
        {
            if (kit.second != key)
                continue;

            std::stringstream stream_out;
            auto block_iter = map._blocks.find(kit.first);
            assert(block_iter != map._blocks.end());
            msgpack::pack(stream_out, block_iter->second);
            assert_ok(sqlite3_bind_int64(stmt, 1, key_to_morton(kit.first)));
            assert_ok(sqlite3_bind_int64(stmt, 2, timestamp));
            auto data = stream_out.str();

            if (_save_options != SaveOptions::ALL_BLOCKS && data.size() > (block_iter->second.node_count()+3)) {
                // skip if all nodes are unknown
                continue;
            }

            assert_ok(sqlite3_bind_blob(stmt, 3, data.c_str(), data.size(), NULL));
            if ((ec = sqlite3_step(stmt)) != SQLITE_DONE)
                throw sqlite_exception(ec, sql_insert_block);

            assert_ok(sqlite3_reset(stmt));
            map._blocks.erase(block_iter);
        }
        assert_ok(sqlite3_finalize(stmt));

        // remove chunks tracking
        map._chunks.erase(key);
        _tracked_chunks.erase(key);
    }

    template <typename SemanticClass, size_t BlockDepth>
    std::string OctomapStorage::check_table(SemanticBKIOctoMap<SemanticClass, BlockDepth> &map)
    {
        // calculate map offset
        auto map_size = map.block_size() * (1 << 20);
        Eigen::Array3f origin = map.get_map_origin().getArray3fMap();
        auto origin_scaled = origin / map_size;
        auto origin_round = origin_scaled.round();
        auto map_id = origin_round.template cast<int>();

        // check whether the map center is aligned
        // it's possible to relieve this requirement to aligning map origin with chunk size, but
        // alignment is essential anyway to keep the integrity of chunks
        auto map_offset = (origin_round - origin_scaled) * map_size;
        if (map_offset.abs().sum() > map.resolution())
            WARN_WRITE("Map origin should be on the grid center with size " << map_size << " meters, "
                        << "current offset: " << map_offset.abs().sum() << ". This could lead to incorrect"
                        << " results if the map will span more than the capacity of a single world");

        // get map name
        std::stringstream ss;
        ss << "x" << map_id[0] << "y" << map_id[1] << "z" << map_id[2];
        std::string table_name = ss.str();

        // create table if needed
        const std::string sql_create_table = "CREATE TABLE IF NOT EXISTS '" + table_name +
            "' (hashkey INTEGER NOT NULL UNIQUE, last_update INTEGER NOT NULL, data BLOB, "
            "PRIMARY KEY(hashkey) ) WITHOUT ROWID;";
        exec_sql(sql_create_table);
        return table_name;
    }

    template <typename SemanticClass, size_t BlockDepth>
    void OctomapStorage::load_around(SemanticBKIOctoMap<SemanticClass, BlockDepth> &map, const pcl::PointXYZ &center)
    {
        std::string table_name = check_table(map);
        auto center_array = center.getVector4fMap();
        float threshold = _active_range + map.chunk_size();
        ChunkHashKey center_chunk = map.block_to_chunk_key(map.loc_to_block_key(center));
        uint64_t ix = center_chunk >> 40, iy = (center_chunk >> 20) & 0xFFFFF, iz = center_chunk & 0xFFFFF;
        uint64_t offset = std::ceil(_active_range / map.chunk_size()) + 1;
        for (uint64_t x = ix - offset; x <= ix + offset; x++)
            for (uint64_t y = iy - offset; y <= iy + offset; y++)
                for (uint64_t z = iz - offset; z <= iz + offset; z++)
                {
                    ChunkHashKey ckey = (x << 40) | (y << 20) | z;
                    pcl::PointXYZ chunk_center = map.chunk_key_to_center(ckey);
                    float center_distance = (chunk_center.getVector4fMap() - center_array).norm();
                    if (center_distance < threshold && _tracked_chunks.find(ckey) == _tracked_chunks.end())
                        load_chunk(map, ckey, table_name);
                }
    }

    template <typename SemanticClass, size_t BlockDepth>
    void OctomapStorage::sync(SemanticBKIOctoMap<SemanticClass, BlockDepth> &map)
    {
        PROFILE_FUNCTION;

        PROFILE_BLOCK("Check table existence");
        std::string table_name = check_table(map);

        // load chunks inside activate range
        PROFILE_SPLIT("Load chunks");
        load_around(map, map._latest_position);

        // dump chunks outside active range
        PROFILE_SPLIT("Dump chunks");
        auto center_array = map._latest_position.getVector4fMap();
        float threshold = _active_range + map.chunk_size();
        assert_ok(sqlite3_exec(_db, "BEGIN TRANSACTION;", NULL, NULL, NULL));
        for (ChunkHashKey ckey : map._chunks)
        {
            if (_tracked_chunks.find(ckey) == _tracked_chunks.end())
                continue;
            pcl::PointXYZ chunk_center = map.chunk_key_to_center(ckey);
            float center_distance = (chunk_center.getVector4fMap() - center_array).norm();
            if (center_distance > threshold)
                dump_chunk(map, ckey, table_name);
        }
        assert_ok(sqlite3_exec(_db, "END TRANSACTION;", NULL, NULL, NULL));

        #ifdef PROFILING
        INFO_WRITE("Updated database, size: " << get_size());
        #endif
    }

    template <typename SemanticClass, size_t BlockDepth>
    void OctomapStorage::dump_all(SemanticBKIOctoMap<SemanticClass, BlockDepth> &map)
    {
        PROFILE_FUNCTION;

        PROFILE_BLOCK("Check table existence");
        std::string table_name = check_table(map);

        // dump all chunks
        PROFILE_SPLIT("Dump chunks");
        assert_ok(sqlite3_exec(_db, "BEGIN TRANSACTION;", NULL, NULL, NULL));
        for (ChunkHashKey ckey : map._chunks)
        {
            if (_tracked_chunks.find(ckey) == _tracked_chunks.end())
                continue;
            dump_chunk(map, ckey, table_name);
        }
        assert_ok(sqlite3_exec(_db, "END TRANSACTION;", NULL, NULL, NULL));
    }

    int OctomapStorage::get_size() const
    {
        sqlite3_stmt *stmt; int ec;

        // get page size
        assert_ok(sqlite3_prepare_v2(_db, "PRAGMA page_size", -1, &stmt, NULL));
        sqlite3_step(stmt);
        int page_size = sqlite3_column_int(stmt, 0);
        assert_ok(sqlite3_finalize(stmt));

        // get page count
        assert_ok(sqlite3_prepare_v2(_db, "PRAGMA page_count", -1, &stmt, NULL));
        sqlite3_step(stmt);
        int page_count = sqlite3_column_int(stmt, 0);
        assert_ok(sqlite3_finalize(stmt));

        return page_size * page_count;
    }
}
