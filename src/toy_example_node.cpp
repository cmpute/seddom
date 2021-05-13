#include <string>
#include <iostream>
#include <ros/ros.h>
#include "semantics.h"
#include "bkioctomap.h"
#include "visualizer.h"

void load_pcd(std::string filename, pcl::PointXYZ &origin, seddom::PointCloudXYZL &cloud) {
    pcl::PCLPointCloud2 cloud2;
    Eigen::Vector4f _origin;
    Eigen::Quaternionf orientaion;
    pcl::io::loadPCDFile(filename, cloud2, _origin, orientaion);
    pcl::fromPCLPointCloud2(cloud2, cloud);
    origin.x = _origin[0];
    origin.y = _origin[1];
    origin.z = _origin[2];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "toy_example_node");
    ros::NodeHandle nh("~");
    
    std::string map_topic_csm("/semantic_csm");
    std::string map_topic("/seddom");
    std::string var_topic_csm("/semantic_csm_variance");
    std::string var_topic("/seddom_variance");
    std::string dir;
    std::string prefix;
    int block_depth = 3;
    int chunk_depth = 0;
    double sf2 = 1.0;
    double ell = 1.0;
    float prior = 1.0f;
    double resolution = 0.1;
    int num_class = 4;
    double free_resolution = 0.5;
    double ds_resolution = 0.1;
    int scan_num = 0;
    double max_range = -1;

    nh.param<std::string>("dir", dir, dir);
    nh.param<std::string>("prefix", prefix, prefix);
    nh.param<int>("block_depth", block_depth, block_depth);
    assert (block_depth == 3);
    nh.param<double>("sf2", sf2, sf2);
    nh.param<double>("ell", ell, ell);
    nh.param<float>("prior", prior, prior);
    nh.param<double>("resolution", resolution, resolution);
    nh.param<int>("num_class", num_class, num_class);
    assert(num_class == 4);
    nh.param<double>("free_resolution", free_resolution, free_resolution);
    nh.param<double>("ds_resolution", ds_resolution, ds_resolution);
    nh.param<int>("scan_num", scan_num, scan_num);
    nh.param<double>("max_range", max_range, max_range);
   
    ROS_INFO_STREAM("Parameters:" << std::endl <<
            "dir: " << dir << std::endl <<
            "prefix: " << prefix << std::endl <<
            "block_depth: " << block_depth << std::endl <<
            "sf2: " << sf2 << std::endl <<
            "ell: " << ell << std::endl <<
            "prior: " << prior << std::endl <<
            "resolution: " << resolution << std::endl <<
            "num_class: " << num_class << std::endl <<
            "free_resolution: " << free_resolution << std::endl <<
            "ds_resolution: " << ds_resolution << std::endl <<
            "scan_sum: " << scan_num << std::endl <<
            "max_range: " << max_range
            );

    /////////////////////// Semantic CSM //////////////////////
    seddom::SemanticBKIOctoMap<seddom::ToyDataset, 1> map_csm(resolution, chunk_depth, sf2, ell, prior, false, false);
    ros::Time start = ros::Time::now();
    for (int scan_id = 1; scan_id <= scan_num; ++scan_id) {
        seddom::PointCloudXYZL::Ptr cloud(new seddom::PointCloudXYZL());
        pcl::PointXYZ origin;
        std::string filename(dir + "/" + prefix + "_" + std::to_string(scan_id) + ".pcd");
        load_pcd(filename, origin, *cloud);
        map_csm.insert_pointcloud<seddom::KernelType::CSM>(cloud, origin, resolution, free_resolution, max_range);
        ROS_INFO_STREAM("Scan " << scan_id << " done");
    }
    ros::Time end = ros::Time::now();
    ROS_INFO_STREAM("Semantic CSM finished in " << (end - start).toSec() << "s");

    /////////////////////// Publish Map //////////////////////
    seddom::OctomapVisualizer m_pub_csm(nh, map_topic_csm);
    m_pub_csm.publish_octomap<seddom::ToyDataset, 1, seddom::OctomapVisualizeMode::SEMANTICS>(map_csm);
    ros::spinOnce();
    
    /////////////////////// Variance Map //////////////////////
    seddom::OctomapVisualizer v_pub_csm(nh, var_topic_csm);
    v_pub_csm.publish_octomap<seddom::ToyDataset, 1, seddom::OctomapVisualizeMode::VARIANCE>(map_csm);
    ros::spinOnce();
    
    /////////////////////// Semantic BKI //////////////////////
    seddom::SemanticBKIOctoMap<seddom::ToyDataset, 3> map(resolution, chunk_depth, sf2, ell, prior, false, false);
    start = ros::Time::now();
    for (int scan_id = 1; scan_id <= scan_num; ++scan_id) {
        seddom::PointCloudXYZL::Ptr cloud(new seddom::PointCloudXYZL());
        pcl::PointXYZ origin;
        std::string filename(dir + "/" + prefix + "_" + std::to_string(scan_id) + ".pcd");
        load_pcd(filename, origin, *cloud);
        map.insert_pointcloud<seddom::KernelType::BGK>(cloud, origin, resolution, free_resolution, max_range);
        ROS_INFO_STREAM("Scan " << scan_id << " done");
    }
    end = ros::Time::now();
    ROS_INFO_STREAM("Semantic BKI finished in " << (end - start).toSec() << "s");
 
    
    /////////////////////// Publish Map //////////////////////
    seddom::OctomapVisualizer m_pub(nh, map_topic);
    m_pub.publish_octomap<seddom::ToyDataset, 3, seddom::OctomapVisualizeMode::SEMANTICS>(map);
    ros::spinOnce();

    /////////////////////// Variance Map //////////////////////
    seddom::OctomapVisualizer v_pub(nh, var_topic);
    v_pub.publish_octomap<seddom::ToyDataset, 3, seddom::OctomapVisualizeMode::VARIANCE>(map);
    ros::spinOnce();

    ros::spin();
    return 0;
}
