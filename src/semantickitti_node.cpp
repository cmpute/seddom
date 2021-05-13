#include <string>
#include <iostream>
#include <ros/ros.h>

#include "bkioctomap.h"
#include "semantickitti_util.h"

int main(int argc, char **argv)
{
    PROFILE_ENABLE_LOCAL;
    ros::init(argc, argv, "semantickitti_node");
    ros::NodeHandle nh("~");

    std::string map_topic("/occupied_cells_vis_array");
    int block_depth = 3;
    int chunk_depth = 6;
    double sf2 = 1.0;
    double ell = 1.0;
    float prior = 1.0f;
    double resolution = 0.1;
    int num_class = 20;
    double free_resolution = 0.5;
    double ds_resolution = 0.1;
    int scan_num = 0;
    double max_range = -1;
    int random_samples_per_beam = -1;

    // SemanticKITTI
    std::string dir;
    std::string input_data_prefix;
    std::string input_label_prefix;
    std::string lidar_pose_file;
    std::string gt_label_prefix;
    std::string evaluation_result_prefix;
    bool query = false;
    bool visualize = false;

    nh.param<int>("block_depth", block_depth, block_depth);
    assert(block_depth == 3);
    nh.param<int>("chunk_depth", chunk_depth, chunk_depth);
    nh.param<double>("sf2", sf2, sf2);
    nh.param<double>("ell", ell, ell);
    nh.param<float>("prior", prior, prior);
    nh.param<double>("resolution", resolution, resolution);
    nh.param<int>("num_class", num_class, num_class);
    assert(num_class == 20);
    nh.param<double>("free_resolution", free_resolution, free_resolution);
    nh.param<double>("ds_resolution", ds_resolution, ds_resolution);
    nh.param<int>("scan_num", scan_num, scan_num);
    nh.param<double>("max_range", max_range, max_range);
    nh.param<int>("random_samples_per_beam", random_samples_per_beam, random_samples_per_beam);

    // SemanticKITTI
    nh.param<std::string>("dir", dir, dir);
    nh.param<std::string>("input_data_prefix", input_data_prefix, input_data_prefix);
    nh.param<std::string>("input_label_prefix", input_label_prefix, input_label_prefix);
    nh.param<std::string>("lidar_pose_file", lidar_pose_file, lidar_pose_file);
    nh.param<std::string>("gt_label_prefix", gt_label_prefix, gt_label_prefix);
    nh.param<std::string>("evaluation_result_prefix", evaluation_result_prefix, evaluation_result_prefix);
    nh.param<bool>("query", query, query);
    nh.param<bool>("visualize", visualize, visualize);

    ROS_INFO_STREAM("Parameters:" << std::endl
                                  << "block_depth: " << block_depth << std::endl
                                  << "sf2: " << sf2 << std::endl
                                  << "ell: " << ell << std::endl
                                  << "prior:" << prior << std::endl
                                  << "resolution: " << resolution << std::endl
                                  << "num_class: " << num_class << std::endl
                                  << "free_resolution: " << free_resolution << std::endl
                                  << "ds_resolution: " << ds_resolution << std::endl
                                  << "random_samples_per_beam: " << random_samples_per_beam << std::endl
                                  << "max_range: " << max_range << std::endl
                                  <<

                    "SemanticKITTI:" << std::endl
                                     << "dir: " << dir << std::endl
                                     << "input_data_prefix: " << input_data_prefix << std::endl
                                     << "input_label_prefix: " << input_label_prefix << std::endl
                                     << "lidar_pose_file: " << lidar_pose_file << std::endl
                                     << "gt_label_prefix: " << gt_label_prefix << std::endl
                                     << "evaluation_result_prefix: " << evaluation_result_prefix << std::endl
                                     << "query: " << query << std::endl
                                     << "scan_num: " << scan_num << std::endl
                                     << "visualize:" << visualize);

    ///////// Build Map /////////////////////
    SemanticKITTIData semantic_kitti_data(nh, resolution, block_depth, chunk_depth, sf2, ell, num_class, ds_resolution, free_resolution, max_range, map_topic, prior, random_samples_per_beam);
    semantic_kitti_data.read_lidar_poses(dir + '/' + lidar_pose_file);
    semantic_kitti_data.set_up_evaluation(dir + '/' + gt_label_prefix, dir + '/' + evaluation_result_prefix);
    semantic_kitti_data.process_scans(dir + '/' + input_data_prefix, dir + '/' + input_label_prefix, scan_num, query, visualize);

    ros::spin();

#ifdef BUILD_WITH_EASY_PROFILER
    profiler::dumpBlocksToFile("/home/jacobz/Coding/ws/src/BKISemanticMapping/profile.prof");
#endif

    return 0;
}
