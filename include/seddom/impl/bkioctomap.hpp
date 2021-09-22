#pragma once

#include <cmath>
#include <algorithm>
#include <fstream>
#include <thread>
#include <array>
#include <utility>

#include <msgpack.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/geometry.hpp>
#include <boost/range/algorithm/nth_element.hpp>

#include "bkioctomap.h"
#include "bki.h"

#define OCTOMAP_TDECL template <typename SemanticClass, size_t BlockDepth>
#define OCTOMAP_CLASS SemanticBKIOctoMap<SemanticClass, BlockDepth>

namespace seddom
{
    inline void catesian_to_spherical(float x, float y, float z, float &phi, float &theta, float &r)
    {
        phi = atan2(y, x);
        phi = phi >= 0 ? phi : phi + DPI;
        float rxy2 = x * x + y * y;
        theta = atan2(z, sqrt(rxy2));
        r = sqrt(rxy2 + z * z);
    }

    // get a bound of a point cloud, while discarding n outlier points
    template <typename PointT>
    void getMinMax3Dnth(const pcl::PointCloud<PointT> &cloud, pcl::PointXYZ &min_pt, pcl::PointXYZ &max_pt, size_t nth)
    {
        std::vector<float> c(cloud.size());
        assert(nth < cloud.size() / 2);
        size_t upper = cloud.size() - nth, lower = nth;

        for (size_t i = 0; i < cloud.size(); i++)
            c[i] = cloud.at(i).x;
        boost::range::nth_element(c, c.begin() + lower);
        min_pt.x = c[lower];
        boost::range::nth_element(c, c.begin() + upper);
        max_pt.x = c[upper];

        for (size_t i = 0; i < cloud.size(); i++)
            c[i] = cloud.at(i).y;
        boost::range::nth_element(c, c.begin() + lower);
        min_pt.y = c[lower];
        boost::range::nth_element(c, c.begin() + upper);
        max_pt.y = c[upper];

        for (size_t i = 0; i < cloud.size(); i++)
            c[i] = cloud.at(i).z;
        boost::range::nth_element(c, c.begin() + lower);
        min_pt.z = c[lower];
        boost::range::nth_element(c, c.begin() + upper);
        max_pt.z = c[upper];
    }

    // https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
    OCTOMAP_TDECL
    constexpr size_t OCTOMAP_CLASS::NumClass;

    OCTOMAP_TDECL
    OCTOMAP_CLASS::SemanticBKIOctoMap(bool occlusion_aware,
                                      float resolution,
                                      size_t chunk_depth,
                                      float sf2,
                                      float ell,
                                      float prior,
                                      float max_range)
        : _occlusion_aware(occlusion_aware),
          _resolution(resolution),
          _chunk_depth(chunk_depth),
          _block_size((float)pow(2, BlockDepth - 1) * resolution),
          _chunk_size((float)pow(2, BlockDepth + chunk_depth - 2) * resolution),
          _sf2(sf2), _ell(ell),
          _max_range(max_range),
          _min_range(0.5),
          _max_beams(500),
          _map_origin(0, 0, 0),
          _latest_time()
    {
        SemanticOctreeNode<NumClass>::prior = prior;
        assert(chunk_depth > 0 && chunk_depth <= 20 && "Chunk depth should not be greater than 20!");

        _blocks.reserve(1 << 16);
        if (chunk_depth > 1)
            _chunks.reserve(1 << (17 - chunk_depth));

        DEBUG_WRITE("SemanticOctomap params:");
        DEBUG_WRITE("\tnum_class: " << NumClass);
        DEBUG_WRITE("\tresolution (node size): " << _resolution << "m");
        DEBUG_WRITE("\tblock size: " << _block_size << "m");
        DEBUG_WRITE("\tchunk size: " << _chunk_size << "m");
    }

    OCTOMAP_TDECL template <KernelType KType>
    void
    OCTOMAP_CLASS::insert_pointcloud(PointCloudXYZL::ConstPtr cloud, const pcl::PointXYZ &origin,
                                     std::chrono::system_clock::time_point timestamp,
                                     float ds_resolution, float free_resolution)
    {
        PROFILE_FUNCTION;
        DEBUG_WRITE("Insert pointcloud: "
                    << "cloud size = " << cloud->size() << ", origin = " << origin);

        PROFILE_BLOCK("Get Training Data");
        PointCloudXYZL::Ptr xy = get_training_data(cloud, origin, ds_resolution, free_resolution);
        DEBUG_WRITE("Training data size: " << xy->size());

        if (xy->size() == 0)
            return;

        PROFILE_SPLIT("Inference");
        inference_points<KType>(xy);

        _latest_position = origin;
        _latest_time = timestamp;

        #ifdef PROFILING
        INFO_WRITE("Map updated, memory size: " << memory_size());
        #endif
    }

    OCTOMAP_TDECL template <KernelType KType>
    void
    OCTOMAP_CLASS::insert_pointcloud(PointCloudXYZL::ConstPtr cloud, const pcl::PointXYZ &origin,
                                     std::chrono::system_clock::time_point timestamp,
                                     float ds_resolution, int samples_per_beam)
    {
        PROFILE_FUNCTION;
        DEBUG_WRITE("Insert pointcloud: "
                    << "cloud size = " << cloud->size() << ", origin = " << origin);

        PROFILE_BLOCK("Get Training Data");
        PointCloudXYZL::Ptr xy = get_random_training_data(cloud, origin, ds_resolution, samples_per_beam);
        DEBUG_WRITE("Training data size: " << xy->size());

        if (xy->size() == 0)
            return;

        PROFILE_SPLIT("Inference");
        inference_points<KType>(xy);

        _latest_position = origin;
        _latest_time = timestamp;

        #ifdef PROFILING
        INFO_WRITE("Map updated, memory size: " << memory_size());
        #endif
    }

    OCTOMAP_TDECL template <KernelType KType>
    void
    OCTOMAP_CLASS::inference_points(const PointCloudXYZL::Ptr training_data)
    {
        PROFILE_FUNCTION;

        PROFILE_BLOCK("Gathering data");
        phmap::node_hash_map<BlockHashKey, std::vector<size_t>> train_blocks;
        phmap::flat_hash_set<BlockHashKey> test_blocks;
        for (size_t i = 0; i < training_data->size(); ++i)
        {
            // add current block
            BlockHashKey hkey = loc_to_block_key(training_data->at(i));
            auto lit = train_blocks.emplace(hkey, std::vector<size_t>()).first;
            lit->second.push_back(i);

            // add extended block
            ExtendedBlock ehkey = get_extended_block(hkey);
            for (auto eit = ehkey.cbegin(); eit != ehkey.cend(); ++eit)
                test_blocks.emplace(*eit);
        }

        PROFILE_SPLIT("Training");
        DEBUG_WRITE("Training blocks: " << train_blocks.size());
        decltype(train_blocks)::const_iterator train_iters[train_blocks.size()];
        auto bit = train_blocks.cbegin();
        for (size_t i = 0; i < train_blocks.size(); ++i)
            train_iters[i] = bit++;

        ParallelMap<BlockHashKey, std::unique_ptr<SBGKI3f<NumClass>>> bgk_arr;
        bgk_arr.reserve(train_blocks.size());
#ifdef OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (size_t i = 0; i < train_blocks.size(); ++i)
        {
            BlockHashKey key = train_iters[i]->first;
            const std::vector<size_t> &indices = train_iters[i]->second;

            PROFILE_THREAD_BLOCK("Copy point");
            Eigen::Matrix<float, -1, 3> block_x(indices.size(), 3);
            Eigen::Matrix<uint32_t, -1, 1> block_y(indices.size(), 1);
            for (size_t j = 0; j < block_x.rows(); j++)
            {
                block_x.row(j) = training_data->at(indices[j]).getVector3fMap();
                block_y[j] = training_data->at(indices[j]).label;
            }

            PROFILE_THREAD_SPLIT("Train and add");
            std::unique_ptr<SBGKI3f<NumClass>> bgk(new SBGKI3f<NumClass>(_sf2, _ell));
            bgk->train(block_x, block_y);
            bgk_arr.emplace(key, std::move(bgk));
        }
        DEBUG_WRITE("Training done");

        ////////// Prediction //////////
        PROFILE_SPLIT("Copying test blocks");
        DEBUG_WRITE("Test blocks: " << test_blocks.size());

        BlockHashKey test_keys[test_blocks.size()];
        auto kit = test_blocks.cbegin();
        for (size_t i = 0; i < test_blocks.size(); ++i)
            test_keys[i] = *(kit++);

        PROFILE_SPLIT("Prediction");
#ifdef OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 0; i < test_blocks.size(); ++i)
        {
            PROFILE_THREAD_BLOCK("Get block from hashmap");
            BlockHashKey key = test_keys[i];
            auto block_iter = _blocks.find(key);
            if (block_iter == _blocks.end())
            {
                block_iter = _blocks.emplace(key,
                                             Block<NumClass, BlockDepth>(block_key_to_center(key), _resolution))
                                 .first;
                if (_occluded_blocks.find(key) != _occluded_blocks.end())
                    _occluded_blocks.erase(key);
                if (_chunk_depth > 1)
                {
                    ChunkHashKey ckey = block_to_chunk_key(key);
                    if (_chunks.find(ckey) == _chunks.end())
                        _chunks.insert(ckey);
                }
            }
            BlockType &block = block_iter->second;

            PROFILE_THREAD_SPLIT("Collect points");
            Eigen::Matrix<float, -1, 4> xs = block.get_node_locs();

            if (KType == KernelType::CSM)
            {
                PROFILE_THREAD_BLOCK("Query inference object");
                auto bgk = bgk_arr.find(key);
                if (bgk == bgk_arr.end())
                    continue;

                PROFILE_THREAD_SPLIT("Predict CSM");
                Eigen::Matrix<float, -1, NumClass> ybars = bgk->second->template predict<KType>(xs.leftCols(3));

                int j = 0;
                for (auto leaf_it = block.begin_leaf(); leaf_it != block.end_leaf(); ++leaf_it, ++j)
                {
                    // Only need to update if kernel density total kernel density est > 0
                    leaf_it->update(ybars.row(j));
                }
            }
            else if (KType == KernelType::BGK || KType == KernelType::SBGK)
            {
                PROFILE_THREAD_BLOCK("Predict per block");
                ExtendedBlock eblock = get_extended_block(key);
                for (auto block_it = eblock.cbegin(); block_it != eblock.cend(); ++block_it)
                {
                    PROFILE_THREAD_BLOCK("Query inference");
                    auto bgk = bgk_arr.find(*block_it);
                    if (bgk == bgk_arr.end())
                        continue;

                    PROFILE_THREAD_SPLIT("Predict BKI");
                    Eigen::Matrix<float, -1, NumClass> ybars = bgk->second->template predict<KType>(xs.leftCols(3));

                    int j = 0;
                    for (auto leaf_it = block.begin_leaf(); leaf_it != block.end_leaf(); ++leaf_it, ++j)
                    {
                        leaf_it->update(ybars.row(j));
                    }
                }
            }
            else
                assert(false);
        }

        DEBUG_WRITE("Prediction done");
    }

    OCTOMAP_TDECL template <KernelType KType>
    void
    OCTOMAP_CLASS::insert_pointcloud_pl(PointCloudXYZL::ConstPtr cloud, const pcl::PointXYZ &origin,
                                        std::chrono::system_clock::time_point timestamp,
                                        float ds_resolution)
    {
        PROFILE_FUNCTION;
        DEBUG_WRITE("Insert pointcloud: " << "cloud size = " << cloud->size() << ", origin = " << origin);

        PROFILE_BLOCK("Downsample hits");
        PointCloudXYZL::Ptr sampled_hits = downsample<pcl::PointXYZL>(cloud, ds_resolution);

        PROFILE_SPLIT("Inference hits");
        if (sampled_hits->size() > 0)
            inference_points<KType>(sampled_hits);

        PROFILE_SPLIT("Create Rtree");
        namespace bg = boost::geometry;
        namespace bgi = boost::geometry::index;

        typedef bg::cs::spherical_equatorial<bg::radian> scoord_t;
        typedef bg::model::point<float, 3, scoord_t> spoint_t; // point in spherical coordinate
        typedef size_t srtree_data_t;                          // index of beam
        typedef std::pair<spoint_t, srtree_data_t> srtree_value_t;
        typedef bg::model::box<spoint_t> sbox_t;
        typedef bgi::rstar<16, 4> srtree_params_t;
        typedef bgi::rtree<srtree_value_t, srtree_params_t> srtree_t; // rtree in spherical coordinate

        std::vector<srtree_value_t> projections;
        projections.reserve(sampled_hits->size());
        const Eigen::Vector4f v_origin = origin.getVector4fMap();
        Eigen::Matrix<float, 4, -1> ptarray = sampled_hits->getMatrixXfMap().topRows(4).colwise() - v_origin; // point cloud relative to origin
        for (size_t i = 0; i < sampled_hits->size(); i++)
        {
            const auto v_p = ptarray.col(i);
            float phi, theta, r;
            catesian_to_spherical(v_p[0], v_p[1], v_p[2], phi, theta, r);
            if (r < _min_range) // don't skip beam beyond max_range since it will affect free beam calculation
                continue;

            spoint_t sp = {phi, theta, r};
            projections.push_back(std::make_pair(sp, i));
        }
        srtree_t beam_tree(projections.cbegin(), projections.cend());

        PROFILE_SPLIT("Get blocks limit");
        pcl::PointXYZ min_pt, max_pt;
        getMinMax3Dnth(*sampled_hits, min_pt, max_pt, 50); // skip outlier points
        DEBUG_WRITE("Beam points range min: " << min_pt);
        DEBUG_WRITE("Beam points range max: " << max_pt);

        ////////// Training //////////
        PROFILE_SPLIT("Train free beams");
        _occluded_blocks.clear();
        std::vector<BlockHashKey> in_blocks;
        if (_occlusion_aware)
            // TODO: add ground range filter, and further optimize which blocks to be considered
            in_blocks = get_blocks_in_bbox(min_pt, max_pt, origin);
        else
            in_blocks = get_mapped_blocks_in_bbox(min_pt, max_pt, origin);
        float search_range = _block_size / 2 * SQ3 + _ell;

#ifdef OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (size_t i = 0; i < in_blocks.size(); ++i) // TODO: try node-wise query instead of block, this might be faster for node inference (much less data points)
        {
            PROFILE_THREAD_BLOCK("Calculate spherical coordinate");
            BlockHashKey hkey = in_blocks[i];
            pcl::PointXYZ loc = block_key_to_center(hkey);
            const auto v_p = loc.getVector4fMap() - v_origin;
            float phi, theta, r;
            catesian_to_spherical(v_p[0], v_p[1], v_p[2], phi, theta, r);

            float rs = search_range / r;
            float rs_phi = rs / cos(theta);
            spoint_t p1 = {phi - rs_phi, std::max(theta - rs, -float(PI / 2)), r};
            spoint_t p2 = {phi + rs_phi, std::min(theta + rs, float(PI / 2)), std::numeric_limits<float>::max()};
            sbox_t bq(p1, p2);

            PROFILE_THREAD_SPLIT("Query rtree");
            auto rit = beam_tree.qbegin(bgi::within(bq));
            if (rit == beam_tree.qend())
            {
                PROFILE_THREAD_BLOCK("Occluded beam");
                if (!_occlusion_aware)
                    continue;

                // this block is occluded if it lies on some extended beams but not on any actual beams
                p1.set<2>(0);
                p2.set<2>(r - _ell); // add tolerance of _ell
                bq = { p1, p2 };
                rit = beam_tree.qbegin(bgi::within(bq));
                if (rit != beam_tree.qend())
                {
                    auto bit = _blocks.find(hkey);
                    if (bit == _blocks.end())
                        _occluded_blocks.insert(hkey);
                    else
                    {
                        std::vector<size_t> indices;
                        for (size_t j = 0; rit != beam_tree.qend() && j < _max_beams; ++rit, ++j)
                            indices.push_back(rit->second);
                        Eigen::Matrix<float, -1, 4> lines(indices.size(), 4);
                        for (size_t j = 0; j < lines.rows(); j++)
                            lines.row(j) = ptarray.col(indices[j]);

                        BlockType &block = bit->second;
                        Eigen::Matrix<float, -1, 4> points = block.get_node_locs();
                        points = points.rowwise() - v_origin.transpose();
                        Eigen::Matrix<float, -1, -1> _; // used to select correct function
                        auto pldist = seddom::dist_pl<float, 3>(lines.leftCols(3), points.leftCols(3), _);
                        Eigen::Matrix<bool, 1, -1> validity = pldist.colwise().minCoeff().array() < _ell;
                        
                        size_t j = 0;
                        for (auto leaf_it = block.begin_leaf(); leaf_it != block.end_leaf(); ++leaf_it, j++)
                            if (validity(j)) // only mark nodes with close enough distance
                                leaf_it->mark_occluded(timestamp);
                    }
                }
            }
            else
            {
                // add to training data
                // TODO: skip predicted blocks
                // TODO: skip free blocks
                // TODO: only update for object blocks
                auto bit = _blocks.find(hkey);
                if (bit == _blocks.end())
                    continue;
                BlockType &block = bit->second;

                PROFILE_THREAD_BLOCK("Train free beam");
                std::vector<size_t> indices;
                for (size_t j = 0; rit != beam_tree.qend() && j < _max_beams; ++rit, ++j)
                    indices.push_back(rit->second);
                Eigen::Matrix<float, -1, 4> block_x(indices.size(), 4);
                for (size_t j = 0; j < block_x.rows(); j++)
                    block_x.row(j) = ptarray.col(indices[j]);
                BGKLI3f bgkl(_sf2, _ell);
                bgkl.train(block_x.leftCols(3));

                PROFILE_THREAD_SPLIT("Collect points");
                Eigen::Matrix<float, -1, 4> xs = block.get_node_locs();
                xs = xs.rowwise() - v_origin.transpose();

                PROFILE_THREAD_SPLIT("Predict free beam");
                if (KType == KernelType::BGK || KType == KernelType::SBGK)
                {
                    PROFILE_THREAD_SPLIT("Predict BKI");
                    Eigen::VectorXf ybar = bgkl.template predict<KType>(xs.leftCols(3));

                    int j = 0;
                    for (auto leaf_it = block.begin_leaf(); leaf_it != block.end_leaf(); ++leaf_it, ++j)
                        leaf_it->update_free(ybar[j]);
                }
                else
                    assert(false);
            }
        }

        DEBUG_WRITE("Free prediction done");
        _latest_position = origin;
        _latest_time = timestamp;

        #ifdef PROFILING
        INFO_WRITE("Map updated, memory size: " << memory_size());
        #endif
    }

    OCTOMAP_TDECL PointCloudXYZL::Ptr
    OCTOMAP_CLASS::get_training_data(
        PointCloudXYZL::ConstPtr cloud, const pcl::PointXYZ &origin,
        float ds_resolution, float free_resolution) const
    {
        PROFILE_FUNCTION;

        PROFILE_BLOCK("Downsample hits");
        PointCloudXYZL::Ptr sampled_hits = downsample<pcl::PointXYZL>(cloud, ds_resolution);
        PointCloudXYZL::Ptr xy = sampled_hits; // output data
        if (free_resolution <= 0)
            return xy;

        PROFILE_SPLIT("Sample beams by fixed resolution");
        PointCloudXYZ::Ptr frees(new PointCloudXYZ());
        frees->push_back(origin);

        for (size_t i = 0; i < sampled_hits->size(); ++i)
        {
            float l = (sampled_hits->at(i).getVector3fMap() - origin.getVector3fMap()).norm();
            if (_max_range > 0 && l > _max_range)
                continue;
            if (l < _min_range)
                continue;

            PROFILE_BLOCK("Beam sampling");
            PointCloudXYZ::Ptr frees_n = beam_sample(sampled_hits->at(i), origin, free_resolution);

            PROFILE_SPLIT("Append samples");
            frees->operator+=(*frees_n);
        }

        PROFILE_SPLIT("Downsample free positions");
        PointCloudXYZ::Ptr sampled_frees = downsample<pcl::PointXYZ>(frees, ds_resolution);
        DEBUG_WRITE("Training data of free points: hits=" << xy->size() << ", free=" << frees->size() << ", ds_free=" << sampled_frees->size());

        xy->reserve(xy->size() + sampled_frees->size());
        for (auto it = sampled_frees->begin(); it != sampled_frees->end(); ++it)
        {
            pcl::PointXYZL p;
            p.x = it->x;
            p.y = it->y;
            p.z = it->z;
            p.label = 0;
            xy->push_back(p);
        }

        return xy;
    }

    OCTOMAP_TDECL PointCloudXYZL::Ptr
    OCTOMAP_CLASS::get_random_training_data(
        PointCloudXYZL::ConstPtr cloud, const pcl::PointXYZ &origin,
        float ds_resolution, int samples_per_beam)
    {
        PROFILE_FUNCTION;

        PROFILE_BLOCK("Downsample hits");
        PointCloudXYZL::Ptr sampled_hits = downsample<pcl::PointXYZL>(cloud, ds_resolution);
        PointCloudXYZL::Ptr xy = sampled_hits; // output data
        if (samples_per_beam <= 0)
            return xy;

        PROFILE_SPLIT("Sample beams randomly");
        PointCloudXYZ::Ptr frees(new PointCloudXYZ());
        frees->resize(1 + sampled_hits->size() * std::max(samples_per_beam, 0));
        frees->at(0) = origin;

#ifdef OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (size_t i = 0; i < sampled_hits->size(); ++i)
        {
            float l = (sampled_hits->at(i).getVector3fMap() - origin.getVector3fMap()).norm();
            if (l < _min_range || (_max_range > 0 && l > _max_range))
            {
                // fill samples with origin for invalid hits
                for (size_t j = i * samples_per_beam + 1; j <= (i + 1) * samples_per_beam; ++j)
                    frees->at(j) = origin;
                continue;
            }

            PROFILE_BLOCK("Beam sampling");
            PointCloudXYZ::Ptr frees_n = beam_sample_random(sampled_hits->at(i), origin, samples_per_beam);

            PROFILE_SPLIT("Append samples");
            for (size_t j = 0; j < samples_per_beam; ++j)
                frees->at(i * samples_per_beam + 1 + j) = frees_n->at(j);
        }

        PROFILE_SPLIT("Downsample free positions");
        PointCloudXYZ::Ptr sampled_frees = downsample<pcl::PointXYZ>(frees, ds_resolution);
        DEBUG_WRITE("Training data of free points (random): hits=" << xy->size() << ", free=" << frees->size() << ", ds_free=" << sampled_frees->size());

        xy->reserve(xy->size() + sampled_frees->size());
        for (auto it = sampled_frees->begin(); it != sampled_frees->end(); ++it)
        {
            pcl::PointXYZL p;
            p.x = it->x;
            p.y = it->y;
            p.z = it->z;
            p.label = 0;
            xy->push_back(p);
        }
        return xy;
    }

    OCTOMAP_TDECL template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr
    OCTOMAP_CLASS::downsample(typename pcl::PointCloud<PointT>::ConstPtr in, float ds_resolution) const
    {
        PROFILE_FUNCTION;
        assert(ds_resolution > 0);

        typename pcl::PointCloud<PointT>::Ptr pc_out(new pcl::PointCloud<PointT>());
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(in);
        sor.setLeafSize(ds_resolution, ds_resolution, ds_resolution);
        sor.filter(*pc_out);

        return pc_out;
    }

    OCTOMAP_TDECL PointCloudXYZ::Ptr
    OCTOMAP_CLASS::beam_sample(
        const pcl::PointXYZL &hit, const pcl::PointXYZ &origin, float free_resolution) const
    {
        PointCloudXYZ::Ptr frees(new PointCloudXYZ());

        Eigen::Vector3f d = hit.getVector3fMap() - origin.getVector3fMap();
        float l = d.norm();
        Eigen::Vector3f normd = d.array() / l;

        size_t ns = l / free_resolution;
        if (ns > 0)
        {
            frees->resize(ns);
            auto free_matrix = normd * Eigen::VectorXf::LinSpaced(
                                           ns, l - ns * free_resolution, l - free_resolution)
                                           .transpose(); // 3 x ns
            frees->getMatrixXfMap(3, 4, 0) = free_matrix.colwise() + origin.getVector3fMap();
        }
        return frees;
    }

    OCTOMAP_TDECL PointCloudXYZ::Ptr
    OCTOMAP_CLASS::beam_sample_random(
        const pcl::PointXYZL &hit, const pcl::PointXYZ &origin, int samples_per_beam)
    {
        PointCloudXYZ::Ptr frees(new PointCloudXYZ());
        frees->resize(samples_per_beam);

        Eigen::Vector3f d = hit.getVector3fMap() - origin.getVector3fMap();
        float l = d.norm();
        Eigen::Vector3f normd = d.array() / l;

        // Eigen::Rand::Vmt19937_64 rng;
        auto r = Eigen::Rand::uniformReal<Eigen::VectorXf>(samples_per_beam, 1, _rng);
        Eigen::VectorXf scaled = (r.array() * (l * l)).sqrt();
        auto free_matrix = normd * scaled.transpose();
        frees->getMatrixXfMap(3, 4, 0) = free_matrix.colwise() + origin.getVector3fMap();
        return frees;
    }

    OCTOMAP_TDECL std::vector<BlockHashKey>
    OCTOMAP_CLASS::get_blocks_in_bbox(
        const pcl::PointXYZ &lim_min, const pcl::PointXYZ &lim_max) const
    {
        std::vector<BlockHashKey> blocks;
        for (float x = lim_min.x - _block_size; x <= lim_max.x + 2 * _block_size; x += _block_size)
            for (float y = lim_min.y - _block_size; y <= lim_max.y + 2 * _block_size; y += _block_size)
                for (float z = lim_min.z - _block_size; z <= lim_max.z + 2 * _block_size; z += _block_size)
                    blocks.push_back(loc_to_block_key(x, y, z));

        return blocks;
    }

    OCTOMAP_TDECL std::vector<BlockHashKey>
    OCTOMAP_CLASS::get_blocks_in_bbox(
        const pcl::PointXYZ &lim_min, const pcl::PointXYZ &lim_max, const pcl::PointXYZ &origin) const
    {
        if (_max_range < 0)
            return get_blocks_in_bbox(lim_min, lim_max);

        std::vector<BlockHashKey> blocks;
        float range2 = _max_range * _max_range;
        for (float x = lim_min.x - _block_size; x <= lim_max.x + 2 * _block_size; x += _block_size)
        {
            float x2 = x * x;
            for (float y = lim_min.y - _block_size; y <= lim_max.y + 2 * _block_size; y += _block_size)
            {
                float y2 = y * y;
                for (float z = lim_min.z - _block_size; z <= lim_max.z + 2 * _block_size; z += _block_size)
                    if (x2 + y2 + z * z < range2)
                        blocks.push_back(loc_to_block_key(x, y, z));
            }
        }

        return blocks;
    }

    OCTOMAP_TDECL std::vector<BlockHashKey>
    OCTOMAP_CLASS::get_mapped_blocks_in_bbox(
        const pcl::PointXYZ &lim_min, const pcl::PointXYZ &lim_max) const
    {
        std::vector<BlockHashKey> blocks;
        for (auto iter = _blocks.cbegin(); iter != _blocks.cend(); iter++)
        {
            pcl::PointXYZ center = block_key_to_center(iter->first);
            if (center.x > lim_min.x && center.x < lim_max.x &&
                center.y > lim_min.y && center.y < lim_max.y &&
                center.z > lim_min.z && center.z < lim_max.z)
                blocks.push_back(iter->first);
        }
        return blocks;
    }

    OCTOMAP_TDECL std::vector<BlockHashKey>
    OCTOMAP_CLASS::get_mapped_blocks_in_bbox(
        const pcl::PointXYZ &lim_min, const pcl::PointXYZ &lim_max, const pcl::PointXYZ &origin) const
    {
        if (_max_range < 0)
            return get_mapped_blocks_in_bbox(lim_min, lim_max);

        std::vector<BlockHashKey> blocks;
        auto origin_v = origin.getArray4fMap();
        float range2 = _max_range * _max_range;
        for (auto iter = _blocks.cbegin(); iter != _blocks.cend(); iter++)
        {
            pcl::PointXYZ center = block_key_to_center(iter->first);
            if (center.x > lim_min.x && center.x < lim_max.x &&
                center.y > lim_min.y && center.y < lim_max.y &&
                center.z > lim_min.z && center.z < lim_max.z &&
                (center.getArray4fMap() - origin_v).square().sum() < range2)
                blocks.push_back(iter->first);
        }
        return blocks;
    }

    OCTOMAP_TDECL typename OCTOMAP_CLASS::const_leaf_iterator
    OCTOMAP_CLASS::search(float x, float y, float z) const
    {
        auto block_iter = _blocks.find(loc_to_block_key(x, y, z));
        if (block_iter == _blocks.cend())
            return cend_leaf();
        else
            return const_leaf_iterator(this, block_iter, block_iter->second.search(x, y, z));
    }

    OCTOMAP_TDECL typename OCTOMAP_CLASS::leaf_iterator
    OCTOMAP_CLASS::search(float x, float y, float z)
    {
        auto block_iter = const_cast<const OCTOMAP_CLASS::BlockMap &>(_blocks).find(loc_to_block_key(x, y, z));
        if (block_iter == _blocks.cend())
            return end_leaf();
        else
            return leaf_iterator(this, block_iter, block_iter->second.search(x, y, z));
    }

    OCTOMAP_TDECL size_t
    OCTOMAP_CLASS::dump_map(const std::string &path) const
    {
        std::ofstream output;
        output.open(path);
        msgpack::packer<std::ofstream> packer(output);

        // pack header
        packer.pack_array(4);
        packer.pack(_resolution);
        packer.pack(BlockDepth);
        packer.pack(NumClass);

        // pack blocks
        packer.pack_map(_blocks.size());
        for (auto iter = _blocks.begin(); iter != _blocks.end(); iter++)
        {
            packer.pack(iter->first);
            packer.pack(iter->second);
        }

        size_t size = output.tellp();
        output.close();
        return size;
    }

    OCTOMAP_TDECL inline BlockHashKey
    OCTOMAP_CLASS::loc_to_block_key(float x, float y, float z, float block_size) const
    {
        return (uint64_t(x / block_size + 524288.5) << 40) |
               (uint64_t(y / block_size + 524288.5) << 20) |
               (uint64_t(z / block_size + 524288.5));
    }

    OCTOMAP_TDECL inline ChunkHashKey
    OCTOMAP_CLASS::block_to_chunk_key(BlockHashKey key) const
    {
        const auto chunk_bits = _chunk_depth - 1;
        return (((key >> (40 + chunk_bits))) << 40) |
               (((key >> 20) & 0xFFFFF) >> chunk_bits << 20) |
               ((key & 0xFFFFF) >> chunk_bits);
    }

    OCTOMAP_TDECL inline pcl::PointXYZ
    OCTOMAP_CLASS::block_key_to_center(BlockHashKey key) const
    {
        return pcl::PointXYZ((int64_t(key >> 40) - 524288) * _block_size,
                             (int64_t((key >> 20) & 0xFFFFF) - 524288) * _block_size,
                             (int64_t(key & 0xFFFFF) - 524288) * _block_size);
    }

    OCTOMAP_TDECL inline pcl::PointXYZ
    OCTOMAP_CLASS::chunk_key_to_center(ChunkHashKey key) const
    {
        const auto chunk_offset = 524288 >> (_chunk_depth - 1);
        return pcl::PointXYZ((int64_t(key >> 40) - chunk_offset) * _chunk_size,
                             (int64_t((key >> 20) & 0xFFFFF) - chunk_offset) * _chunk_size,
                             (int64_t(key & 0xFFFFF) - chunk_offset) * _chunk_size);
    }

    OCTOMAP_TDECL typename OCTOMAP_CLASS::ExtendedBlock
    OCTOMAP_CLASS::get_extended_block(BlockHashKey key) const
    {
        ExtendedBlock eblocks;
        pcl::PointXYZ center = block_key_to_center(key);
        float x = center.x;
        float y = center.y;
        float z = center.z;

        float ex, ey, ez;
#ifdef EXPAND_PREDICTION
        // here current block is still the first block
        for (int i = 0; i < 27; ++i)
        {
            int ix = i % 3;
            int iy = (i / 3) % 3;
            int iz = (i / 9);
            ex = (ix == 1) ? (ix == 2 ? _block_size : -_block_size) : 0;
            ey = (iy == 1) ? (iy == 2 ? _block_size : -_block_size) : 0;
            ez = (iz == 1) ? (iz == 2 ? _block_size : -_block_size) : 0;
            eblocks[i] = loc_to_block_key(ex + x, ey + y, ez + z);
        }
#else
        eblocks[0] = key;
        for (int i = 0; i < 6; ++i)
        {
            ex = (i / 2 == 0) ? (i % 2 == 0 ? _block_size : -_block_size) : 0;
            ey = (i / 2 == 1) ? (i % 2 == 0 ? _block_size : -_block_size) : 0;
            ez = (i / 2 == 2) ? (i % 2 == 0 ? _block_size : -_block_size) : 0;
            eblocks[i + 1] = loc_to_block_key(ex + x, ey + y, ez + z);
        }
#endif
        return eblocks;
    }

    OCTOMAP_TDECL template <bool Constant>
    OCTOMAP_CLASS::LeafIterator<Constant>::LeafIterator(const SemanticBKIOctoMap *map)
    {
        assert(map != nullptr);

        block_it = map->_blocks.cbegin();
        end_block = map->_blocks.cend();

        if (map->_blocks.size() > 0)
        {
            leaf_it = block_it->second.cbegin_leaf();
            end_leaf = block_it->second.cend_leaf();
        }
        else
        {
            leaf_it = typename BlockType::const_leaf_iterator();
            end_leaf = typename BlockType::const_leaf_iterator();
        }
    }

    OCTOMAP_TDECL template <bool Constant>
    OCTOMAP_CLASS::LeafIterator<Constant> &
    OCTOMAP_CLASS::LeafIterator<Constant>::operator++()
    {
        ++leaf_it;
        if (leaf_it == end_leaf)
        {
            ++block_it;
            if (block_it != end_block)
            {
                leaf_it = block_it->second.cbegin_leaf();
                end_leaf = block_it->second.cend_leaf();
            }
        }
        return *this;
    }
    
    OCTOMAP_TDECL std::string
    OCTOMAP_CLASS::summary() const
    {
        std::stringstream ss;
        ss << "Total blocks:" << _blocks.size() << ", total chunks:" << _chunks.size() << std::endl;
        ss << "Blocks stats:" << std::endl;
        std::array<int, SemanticClass::NumClass> block_counts;
        block_counts.fill(0);

        for (auto leaf_iter = cbegin_leaf(); leaf_iter != cend_leaf(); leaf_iter++)
            if (leaf_iter->is_classified())
                block_counts[leaf_iter->get_semantics()]++;
        for (int i = 0; i < SemanticClass::NumClass; i++)
            ss << "  " << i << ": " << block_counts[i] << " blocks" << std::endl;
        return ss.str();
    }
}

#undef OCTOMAP_TDECL
#undef OCTOMAP_CLASS
