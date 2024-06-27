#include "PointCloud.h"
#include <pcl/console/parse.h>
#include <ctime>
#define _CRT_SECURE_NO_WARNINGS
#include <string>
#include <iostream>

int main(int argc, char** argv)
{
    std::clock_t start;
    double duration;
    start = std::clock();

    PointCloud data;

    std::string project_name = "trees_new";
    std::string project_file = "trees.txt";

    bool pcl_convert_bool = false;
    bool save_normal_bool = false;
    bool segmentation_bool = true;

    data.CheckAndCreateProject(project_name);
    data.CheckAndCreateProject(project_name + "/Segments");

    // Convert text file to PCD
    if (pcl_convert_bool) {
        std::string input_file = "Datasets/" + project_file;
        std::string output_file = "Projects/" + project_name + "/raw.pcd";
        std::unordered_map<std::string, int> columnIndex;
        data.Txt2pcd(input_file, output_file, columnIndex);
        std::string raw_txt_file = "Projects/" + project_name + "/raw.txt";
        data.Pcd2txt(output_file, raw_txt_file);

        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        std::cout << "time elapsed: " << duration << " seconds." << std::endl;
    }

    
    // Save normals to a PCD file
    if (save_normal_bool) {
        std::string input_file = "Projects/" + project_name + "/raw.pcd";
        std::string output_file = "Projects/" + project_name + "/normal.pcd";
        data.SaveNormals(input_file, output_file);

        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        std::cout << "time elapsed: " << duration << " seconds." << std::endl;
    }
    
    if (segmentation_bool) {
        std::string raw_input_file = "Projects/" + project_name + "/raw.pcd";
        std::string normal_input_file = "Projects/" + project_name + "/normal.pcd";
        std::string output_folder = "Projects/" + project_name + "/Segments";

        data.segmentPointCloud(
            raw_input_file,
            normal_input_file, output_folder,
            1.0f,   //voxel_resolution
            3.0f,   //seed_resolution
            0.0f,   //color_importance
            10.0f,  //spatial_importance
            10.0f,  //normal_importance
            false,  //use_single_cam_transform
            true,   //use_supervoxel_refinement
            10.0f,  //concavity_tolerance_threshold
            0.8f,   //smoothness_threshold
            1,      //min_segment_size
            true,   //use_extended_convexity
            true,   //use_sanity_criterion
            true,   //colorful_segmentation
            "pcd"   //saving_type
        );

        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        std::cout << "time elapsed: " << duration << " seconds." << std::endl;
    }

        

    // Measure and print duration
    

    return 0;
}