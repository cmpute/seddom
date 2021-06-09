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
    class sqlite_exception: public std::runtime_error
    {
        public:
            sqlite_exception(const char* msg, std::string sql, int code = -1): runtime_error(msg), code(code), sql(sql) {}
            sqlite_exception(int code, std::string sql): runtime_error(sqlite3_errstr(code)), code(code), sql(sql) {}
            int get_code() const {return code & 0xFF;}
            int get_extended_code() const {return code;}
            std::string get_sql() const {return sql;}
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
        : _closed(false), _active_range(active_range)
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

    inline void OctomapStorage::assert_ok(int code)
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
        sqlite3_stmt *stmt; int ec;
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
    bool OctomapStorage::check_params(const SemanticBKIOctoMap<SemanticClass, BlockDepth>& map)
    {
        sqlite3_stmt *stmt; int ec;
        bool meta_exists = exec_find_sql("SELECT 1 FROM sqlite_master WHERE type='table' AND name='meta';");
        if (meta_exists)
        {
            // read params
            const auto compare_param = [this, stmt, ec](const std::string& key, const std::string& value) -> bool
            {
                std::stringstream ss;
                ss << "SELECT 1 FROM meta WHERE key='" << key << "' AND value='" << value << "';";
                return exec_find_sql(ss.str());
            };
            if (!compare_param("block_depth", std::to_string(BlockDepth))) return false;
            if (!compare_param("chunk_depth", std::to_string(map.chunk_depth()))) return false;
            if (!compare_param("resolution", std::to_string(map.resolution()))) return false;
            if (!compare_param("semantic_class", typeid(SemanticClass).name())) return false;
            if (!compare_param("semantic_count", std::to_string(SemanticClass::NumClass))) return false;
            return true;
        }
        else
        {
            // create table
            const std::string sql_create_table = "CREATE TABLE meta (" \
                "key TEXT NOT NULL UNIQUE, value TEXT);";
            exec_sql(sql_create_table);

            // insert params
            const auto insert_param = [this](const std::string& key, const std::string& value)
            {
                std::stringstream ss;
                ss << "INSERT INTO meta (key, value) VALUES ('" << key << "', '" << value << "');";
                exec_sql(ss.str());
            };
            insert_param("block_depth", std::to_string(BlockDepth));
            insert_param("chunk_depth", std::to_string(map.chunk_depth()));
            insert_param("resolution", std::to_string(map.resolution()));
            insert_param("semantic_class", typeid(SemanticClass).name());
            insert_param("semantic_count", std::to_string(SemanticClass::NumClass));
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

    // TODO: add virtual function for determine whether a chunk is activate
    // TODO: add function to dump whole map (remained part)
    // TODO: add function to load map at first frame

    template <typename SemanticClass, size_t BlockDepth>
    void OctomapStorage::sync(SemanticBKIOctoMap<SemanticClass, BlockDepth>& map)
    {
        // calculate map offset
        auto map_size = map.block_size() * (1 << 20);
        Eigen::Array3f origin = map.get_map_origin().getArray3fMap();
        auto origin_scaled = origin / map_size;
        auto origin_round = origin_scaled.round();
        auto map_id = origin_round.template cast<int>();
        auto map_offset = (origin_round - origin_scaled) * map_size;
        if (map_offset.abs().sum() > map.resolution())
            WARN_WRITE("Map origin should be on the grid center with size " << map_size << " meters, " <<
                       "current offset: " << map_offset.abs().sum());

        // get map name
        std::stringstream ss;
        ss << "x" << map_id[0] << "y" << map_id[1] << "z" << map_id[2];
        std::string table_name = ss.str();

        // create table if needed
        const std::string sql_create_table = "CREATE TABLE IF NOT EXISTS '" + table_name +
            "' (hashkey INTEGER NOT NULL UNIQUE, last_update INTEGER NOT NULL, data BLOB, " \ 
            "PRIMARY KEY(hashkey) ) WITHOUT ROWID;";
        exec_sql(sql_create_table);

        // find chunks to modify
        std::vector<std::pair<ChunkHashKey, bool>> chunks_to_process;
        auto sensor_origin = map._latest_position.getArray4fMap();
        float threshold = _active_range + map.chunk_size();
        threshold = threshold * threshold;
        for (ChunkHashKey ckey : map._chunks)
        {
            pcl::PointXYZ chunk_center = map.chunk_key_to_center(ckey);
            float center_distance = (chunk_center.getArray4fMap() - sensor_origin).square().sum();
            if (center_distance < threshold)
            {
                if (_tracked_chunks.find(ckey) == _tracked_chunks.end())
                    chunks_to_process.push_back(std::make_pair(ckey, true));
            }
            else
            {
                if (_tracked_chunks.find(ckey) != _tracked_chunks.end())
                    chunks_to_process.push_back(std::make_pair(ckey, false));
            }
        }

        // pre-calculate keys
        std::vector<std::pair<BlockHashKey, ChunkHashKey>> blocks;
        blocks.reserve(map._blocks.size());
        for (const auto& it : map._blocks)
            blocks.emplace_back(it.first, map.block_to_chunk_key(it.first));

        // process chunks
        int counter = 0, ec;
        size_t chunk_bits = 3 * (map._chunk_depth - 1);
        sqlite3_stmt *stmt;
        const std::string sql_insert_block = "REPLACE INTO '" + table_name +
            "' (hashkey, last_update, data) VALUES(?,?,?);";
        const std::string sql_query_chunk = "SELECT hashkey, last_update, data FROM '" + table_name +
            "' WHERE hashkey >= ? AND hashkey < ? ;";
        assert_ok(sqlite3_exec(_db, "BEGIN TRANSACTION;", NULL, NULL, NULL));
        for (const auto& it : chunks_to_process)
        {
            if (it.second)
            {
                // Load chunk from database
                const uint64_t morton = key_to_morton(it.first);
                INFO_WRITE("Loading chunk " << it.first << " (morton: " << morton << ")");
                assert_ok(sqlite3_prepare_v2(_db, sql_query_chunk.c_str(), -1, &stmt, NULL));
                assert_ok(sqlite3_bind_int64(stmt, 1, morton << chunk_bits));
                assert_ok(sqlite3_bind_int64(stmt, 2, (morton + 1) << chunk_bits));
                ec = sqlite3_step(stmt);
                if (ec == SQLITE_ROW)
                {
                    do
                    {
                        BlockHashKey key = sqlite3_column_int64(stmt, 0);
                        size_t timestamp = sqlite3_column_int64(stmt, 1);
                        int n = sqlite3_column_bytes(stmt, 2);
                        const char* cur = reinterpret_cast<const char*>(sqlite3_column_blob(stmt, 2));
                        msgpack::object_handle oh = msgpack::unpack(cur, n);

                        auto block_iter = map._blocks.find(key);
                        if (block_iter == map._blocks.end())
                            block_iter = map._blocks.emplace(key,
                                Block<SemanticClass::NumClass, BlockDepth>(
                                    map.block_key_to_center(key), map._resolution)
                            ).first;
                        oh.get().convert(block_iter->second);
                    } while ((ec = sqlite3_step(stmt)) == SQLITE_ROW);
                }
                assert_ok(sqlite3_finalize(stmt));
                _tracked_chunks.insert(it.first);
            }
            else
            {
                // Dump chunk into database
                INFO_WRITE("Dumping chunk " << it.first);
                auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(map._latest_time.time_since_epoch()).count();
                assert_ok(sqlite3_prepare_v2(_db, sql_insert_block.c_str(), -1, &stmt, NULL));

                for (const auto& kit : blocks)
                {
                    if (kit.second != it.first)
                        continue;

                    std::stringstream stream_out;
                    auto block_iter = map._blocks.find(kit.first);
                    assert(block_iter != map._blocks.end());
                    msgpack::pack(stream_out, block_iter->second);
                    assert_ok(sqlite3_bind_int64(stmt, 1, key_to_morton(kit.first)));
                    assert_ok(sqlite3_bind_int64(stmt, 2, timestamp));
                    auto data = stream_out.str(); // TODO: possible data copy
                    assert_ok(sqlite3_bind_blob(stmt, 3, data.c_str(), data.size(), NULL));

                    if ((ec = sqlite3_step(stmt)) != SQLITE_DONE)
                        throw sqlite_exception(ec, sql_insert_block);
                    assert_ok(sqlite3_reset(stmt));
                    map._blocks.erase(block_iter);
                }
                assert_ok(sqlite3_finalize(stmt));

                // remove chunks tracking
                map._chunks.erase(it.first);
                _tracked_chunks.erase(it.first);
            }
        }
        assert_ok(sqlite3_exec(_db, "END TRANSACTION;", NULL, NULL, NULL));
    }
}
