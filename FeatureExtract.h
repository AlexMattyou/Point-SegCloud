#pragma once
#ifndef FEATUREEXTRACT_H
#define FEATUREEXTRACT_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
#include <sstream>
#include <limits>
#include <regex>
#include <algorithm>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <Eigen/Dense>

#include <pcl/features/normal_3d.h>

class FeatureExtract
{
public:
    void AllFeatureExtract(const std::string& project_files, const std::vector<std::string>& features_need);

private:
    struct Point {
        double x, y, z;
    };

    // Shape Features
    double FindPtness(std::string filepath);
    double FindCurveness(std::string filepath);
    double FindSurfaceness(std::string filepath);
    double FindLinearity(std::string filepath);
    double FindPlanarity(std::string filepath);
    double FindVolumetric(std::string filepath);
};

#endif // FEATUREEXTRACT_H