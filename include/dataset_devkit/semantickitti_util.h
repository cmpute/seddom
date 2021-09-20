#pragma once

#include <fstream>
#include <sstream>
#include <iomanip>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "visualizer.h"
#include "storage.h"

class SemanticKITTIData
{
public:
    SemanticKITTIData(ros::NodeHandle &nh,
                      float resolution,
                      int block_depth,
                      int chunk_depth,
                      float sf2, float ell,
                      int num_class,
                      float ds_resolution,
                      float free_resolution, float max_range,
                      std::string map_topic,
                      float prior, int samples_per_beam)
        : nh_(nh), resolution_(resolution), ds_resolution_(ds_resolution), free_resolution_(free_resolution), samples_per_beam_(samples_per_beam)
    {
        assert(block_depth == 3);
        assert(num_class == 20);
        map_ = new seddom::SemanticBKIOctoMap<seddom::SemanticKITTI, 3>(
            resolution, chunk_depth, sf2, ell, prior, max_range);
        m_pub_ = new seddom::OctomapVisualizer(nh_, map_topic);
        storage_ = new seddom::OctomapStorage("test.db3", 100);
        storage_->load_around(*map_, pcl::PointXYZ(0,0,0));
        publish_map();

        std::cout << "params compatible: " << storage_->check_params(*map_) << std::endl;
        init_trans_to_ground_ << 1,  0, 0, 0,
                                 0,  0, 1, 0,
                                 0, -1, 0, 1,
                                 0,  0, 0, 1;
    }

    bool read_lidar_poses(const std::string lidar_pose_name)
    {
        if (std::ifstream(lidar_pose_name))
        {
            std::ifstream fPoses;
            fPoses.open(lidar_pose_name.c_str());
            while (!fPoses.eof())
            {
                std::string s;
                std::getline(fPoses, s);
                if (!s.empty())
                {
                    std::stringstream ss;
                    ss << s;
                    Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();
                    for (int i = 0; i < 3; ++i)
                        for (int j = 0; j < 4; ++j)
                            ss >> t_matrix(i, j);
                    lidar_poses_.push_back(t_matrix);
                }
            }
            fPoses.close();
            return true;
        }
        else
        {
            ROS_ERROR_STREAM("Cannot open evaluation list file " << lidar_pose_name);
            return false;
        }
    }

    bool process_scans(std::string input_data_dir, std::string input_label_dir, int scan_num, bool query, bool visualize)
    {
        pcl::PointXYZ origin;
        for (int scan_id = 0; scan_id < scan_num; ++scan_id)
        {
            std::stringstream scan_name;
            scan_name << input_data_dir << std::setfill('0') << std::setw(6) << scan_id << ".bin";
            std::stringstream label_name;
            label_name << input_label_dir << std::setfill('0') << std::setw(6) << scan_id << ".label";

            pcl::PointCloud<pcl::PointXYZL>::Ptr cloud = kitti2pcl(scan_name.str(), label_name.str());
            Eigen::Matrix4d transform = lidar_poses_[scan_id];
            Eigen::Matrix4d calibration;

            // 00-02: 2011_10_03_drive
            //calibration << 0.000427680238558, -0.999967248494602, -0.008084491683471, -0.011984599277133,
            //	      -0.007210626507497,  0.008081198471645, -0.999941316450383, -0.054039847297480,
            //	       0.999973864590328,  0.000485948581039, -0.007206933692422, -0.292196864868591,
            //	       0                ,  0                ,  0                ,  1.000000000000000;

            // 03: 2011_09_26_drive_0067
            //calibration << 0.000234773698147, -0.999944154543764, -0.010563477811052, -0.002796816941295,
            //               0.010449407416593,  0.010565353641379, -0.999889574117649, -0.075108791382965,
            //               0.999945388562002,  0.000124365378387,  0.010451302995669, -0.272132796405873,
            //               0                ,  0                ,  0                ,  1.000000000000000;

            // 04-10: 2011_09_30_drive
            calibration <<  -0.001857739385241, -0.999965951350955, -0.008039975204516, -0.004784029760483,
                            -0.006481465826011,  0.008051860151134, -0.999946608177406, -0.073374294642306,
                             0.999977309828677, -0.001805528627661, -0.006496203536139, -0.333996806443304,
                             0                ,  0                ,  0                ,  1.000000000000000;

            Eigen::Matrix4d new_transform = init_trans_to_ground_ * transform * calibration;
            pcl::transformPointCloud(*cloud, *cloud, new_transform);

            pcl::PointCloud<pcl::PointXYZ> cloud_origin;
            cloud_origin.push_back(pcl::PointXYZ());
            pcl::transformPointCloud(cloud_origin, cloud_origin, new_transform);
            origin = cloud_origin.at(0);

            ros::Time tstart = ros::Time::now();
            if (free_resolution_ >= 0)
                map_->insert_pointcloud<seddom::KernelType::BGK>(cloud, origin, ds_resolution_, free_resolution_);
            else if (samples_per_beam_ >= 0)
                map_->insert_pointcloud<seddom::KernelType::BGK>(cloud, origin, ds_resolution_, samples_per_beam_);
            else
                map_->insert_pointcloud_pl<seddom::KernelType::SBGK>(cloud, origin, ds_resolution_);
            ros::Time tend = ros::Time::now();
            std::cout << "Inserted point cloud at " << scan_name.str() <<
                         ", currently " << map_->block_count() << " blocks and " << map_->chunk_count() << " chunks" <<
                         ", takes " << std::setprecision(4) << (tend - tstart).toSec() * 1000 << " ms" << std::endl;

            if (query)
            {
                for (int query_id = scan_id - 10; query_id >= 0 && query_id <= scan_id; ++query_id)
                    query_scan(input_data_dir, query_id);
            }

            storage_->sync(*map_);
            if (visualize)
                publish_map();
        }
        storage_->close();
        map_->dump_map("/home/jacobz/Coding/ws/src/seddom/map_dump.bin"); // TODO: move this to a ROS service
        return 1;
    }

    void publish_map()
    {
        m_pub_->publish_octomap<seddom::SemanticKITTI, 3, seddom::OctomapVisualizeMode::SEMANTICS>(*map_);
        ros::spinOnce();
    }

    void set_up_evaluation(const std::string gt_label_dir, const std::string evaluation_result_dir)
    {
        gt_label_dir_ = gt_label_dir;
        evaluation_result_dir_ = evaluation_result_dir;
    }

    void query_scan(std::string input_data_dir, int scan_id)
    {
        char scan_id_c[256];
        sprintf(scan_id_c, "%06d", scan_id);
        std::string scan_name = input_data_dir + std::string(scan_id_c) + ".bin";
        std::string gt_name = gt_label_dir_ + std::string(scan_id_c) + ".label";
        std::string result_name = evaluation_result_dir_ + std::string(scan_id_c) + ".txt";
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud = kitti2pcl(scan_name, gt_name);
        Eigen::Matrix4d transform = lidar_poses_[scan_id];
        Eigen::Matrix4d calibration;

        // 00-02: 2011_10_03_drive
        //calibration << 0.000427680238558, -0.999967248494602, -0.008084491683471, -0.011984599277133,
        //	      -0.007210626507497,  0.008081198471645, -0.999941316450383, -0.054039847297480,
        //	       0.999973864590328,  0.000485948581039, -0.007206933692422, -0.292196864868591,
        //	       0                ,  0                ,  0                ,  1.000000000000000;

        // 03: 2011_09_26_drive_0067
        //calibration << 0.000234773698147, -0.999944154543764, -0.010563477811052, -0.002796816941295,
        //               0.010449407416593,  0.010565353641379, -0.999889574117649, -0.075108791382965,
        //               0.999945388562002,  0.000124365378387,  0.010451302995669, -0.272132796405873,
        //               0                ,  0                ,  0                ,  1.000000000000000;

        // 04-10: 2011_09_30_drive
        calibration <<  -0.001857739385241, -0.999965951350955, -0.008039975204516, -0.004784029760483,
                        -0.006481465826011,  0.008051860151134, -0.999946608177406, -0.073374294642306,
                        0.999977309828677, -0.001805528627661, -0.006496203536139, -0.333996806443304,
                        0                ,  0                ,  0                ,  1.000000000000000;

        Eigen::Matrix4d new_transform = transform * calibration;
        pcl::transformPointCloud(*cloud, *cloud, new_transform);

        std::ofstream result_file;
        result_file.open(result_name);
        for (int i = 0; i < cloud->points.size(); ++i)
        {
            auto node_iter = map_->search(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            int pred_label = 0;
            if (node_iter != map_->end_leaf() && node_iter->get_state() == seddom::State::OCCUPIED)
                pred_label = node_iter->get_semantics();
            result_file << cloud->points[i].label << " " << pred_label << "\n";
        }
        result_file.close();
    }

private:
    ros::NodeHandle nh_;
    float resolution_;
    float ds_resolution_;
    float free_resolution_;
    int samples_per_beam_;
    seddom::SemanticBKIOctoMap<seddom::SemanticKITTI, 3> *map_;
    seddom::OctomapVisualizer *m_pub_;
    seddom::OctomapStorage *storage_;
    ros::Publisher color_octomap_publisher_;
    tf::TransformListener listener_;
    std::ofstream pose_file_;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> lidar_poses_;
    std::string gt_label_dir_;
    std::string evaluation_result_dir_;
    Eigen::Matrix4d init_trans_to_ground_;

    int check_element_in_vector(const long long element, const std::vector<long long> &vec_check)
    {
        for (int i = 0; i < vec_check.size(); ++i)
            if (element == vec_check[i])
                return i;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZL>::Ptr kitti2pcl(std::string fn, std::string fn_label)
    {
        FILE *fp_label = std::fopen(fn_label.c_str(), "r");
        if (!fp_label)
        {
            std::perror("File opening failed");
        }
        std::fseek(fp_label, 0L, SEEK_END);
        std::rewind(fp_label);
        FILE *fp = std::fopen(fn.c_str(), "r");
        if (!fp)
        {
            std::perror("File opening failed");
        }
        std::fseek(fp, 0L, SEEK_END);
        size_t sz = std::ftell(fp);
        std::rewind(fp);
        int n_hits = sz / (sizeof(float) * 4);
        pcl::PointCloud<pcl::PointXYZL>::Ptr pc(new pcl::PointCloud<pcl::PointXYZL>);
        for (int i = 0; i < n_hits; i++)
        {
            pcl::PointXYZL point;
            float intensity;
            if (fread(&point.x, sizeof(float), 1, fp) == 0)
                break;
            if (fread(&point.y, sizeof(float), 1, fp) == 0)
                break;
            if (fread(&point.z, sizeof(float), 1, fp) == 0)
                break;
            if (fread(&intensity, sizeof(float), 1, fp) == 0)
                break;
            if (fread(&point.label, sizeof(float), 1, fp_label) == 0)
                break;
            pc->push_back(point);
        }
        std::fclose(fp);
        std::fclose(fp_label);
        return pc;
    }
};
