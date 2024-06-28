#pragma once
#ifndef POINTCLOUD_H
#define POINTCLOUD_H
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <string>
#include <fstream>
#include <map>
#include <ctime>

// for text to pcd
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// for normalization
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>

// PCL segmentation
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types_conversion.h>

#include <pcl/features/normal_3d.h>
#include <filesystem>

#endif // POINTCLOUD_H

class PointCloud
{
public:
    using PointT = pcl::PointXYZRGBA;  // Define PointT once

    int CheckAndCreateProject(const std::string& folderName);

    int Txt2pcd(std::string& inputFile, std::string& outputFile, std::unordered_map<std::string, int>& columnIndex);
    int Pcd2txt(std::string& inputFile, std::string& outputFile);
    int SaveNormals(const std::string& inputFile, const std::string& outputFile);

    typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

    void segmentPointCloud(const std::string& input_cloud_file,
        const std::string& input_normals_file,
        const std::string& output_folder,
        float voxel_resolution,
        float seed_resolution,
        float color_importance,
        float spatial_importance,
        float normal_importance,
        bool use_single_cam_transform,
        bool use_supervoxel_refinement,
        float concavity_tolerance_threshold,
        float smoothness_threshold,
        uint32_t min_segment_size,
        bool use_extended_convexity,
        bool use_sanity_criterion,
        bool colorful_segmentation,
        const std::string& saving_type);

private:
    void saveSegmentedClouds(const pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_cloud,
        const std::string& output_folder,
        bool colorful_segmentation,
        const std::string& raw_file, const std::string& saving_type);

    void clearFolder(const std::filesystem::path& folderPath);
};

// Derived class to access protected method
template<typename PointT>
class AccessibleLCCPSegmentation : public pcl::LCCPSegmentation<PointT> {
public:
    using pcl::LCCPSegmentation<PointT>::mergeSmallSegments;
};
