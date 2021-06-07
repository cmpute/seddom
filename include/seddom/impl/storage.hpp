#include <stdexcept>
#include <sstream>
#include <typeinfo>

#include "storage.h"

#include <morton-nd/mortonND_BMI2.h>

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

    OctomapStorage::OctomapStorage(const std::string &database) : _closed(false)
    {
        int ec = sqlite3_open(database.c_str(), &_db);
        if (ec != SQLITE_OK)
            throw sqlite_exception(ec, "");
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
            sqlite3_close(_db);
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
        ec = sqlite3_prepare_v2(_db, sql.c_str(), -1, &stmt, NULL); assert (ec == SQLITE_OK);
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
            sqlite3_stmt *stmt;
            const std::string sql_insert_param = "SELECT value FROM meta WHERE key=?';";
            ec = sqlite3_prepare_v2(_db, sql_insert_param.c_str(), -1, &stmt, NULL); assert (ec == SQLITE_OK);
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
    // TODO: move the block center by map_origin offset
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
    void OctomapStorage::sync(SemanticBKIOctoMap<SemanticClass, BlockDepth>& map)
    {
        // Loop through trunks
        // 0. determine table to insert (x?y?z?)
        // 1. dump inactive chunks (out of range) into database
        // 2. load new active chunks (not in _tracked_chunks) into database and merge data
    }
}
