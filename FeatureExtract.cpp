#include "FeatureExtract.h"

void FeatureExtract::AllFeatureExtract(const std::string& project_files, const std::vector<std::string>& features_need) {
    // Open the CSV file for writing
    std::string csv_path = project_files + "/feature.csv";
    std::ofstream csv_file(csv_path, std::ios::out | std::ios::trunc);

    if (!csv_file.is_open()) {
        std::cerr << "Error: Could not open CSV file " << csv_path << std::endl;
        return;
    }

    // Write the headers based on the required features
    csv_file << "SID";
    if (std::find(features_need.begin(), features_need.end(), "Geometrical") != features_need.end()) {
        csv_file << ",Noofpoints,Xmax,Xmin,Ymax,Ymin,Zmax,Zmin,Zrange";
    }
    if (std::find(features_need.begin(), features_need.end(), "Statistical") != features_need.end()) {
        csv_file << ",meanabsht";
    }
    if (std::find(features_need.begin(), features_need.end(), "Shape") != features_need.end()) {
        csv_file << ",ptness,curveness,surfaceness,linearity,planarity,volumetric";
    }
    if (std::find(features_need.begin(), features_need.end(), "Density") != features_need.end()) {
        csv_file << ",Density,HeightVariance,VerticalProfile";
    }
    if (std::find(features_need.begin(), features_need.end(), "Color") != features_need.end()) {
        csv_file << ",Rmax,Rmin,Gmax,Gmin,Bmax,Bmin,Rmean,Gmean,Bmean,NDVI";
    }
    if (std::find(features_need.begin(), features_need.end(), "Area") != features_need.end()) {
        csv_file << ",Aconv,Apoly,Bpoly";
    }
    csv_file << "\n";

    // Iterate through the segments folder
    std::string segments_folder = project_files + "/segments";
    for (const auto& entry : std::filesystem::directory_iterator(segments_folder)) {
        std::string filepath = entry.path().string();
        std::string filename = entry.path().filename().string();
        std::regex re("segment_(\\d+)");
        std::smatch match;

        if (std::regex_search(filename, match, re)) {
            int segment_id = std::stoi(match[1].str());

            // Read segment file
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filepath, *cloud) == -1) {
                std::cerr << "Couldn't read file " << filepath << std::endl;
                continue;
            }

            if (cloud->empty()) {
                std::cerr << "Error: No points found in file " << filepath << std::endl;
                continue;
            }

        // Initialize feature variables
            int no_of_points = cloud->points.size();
            double xmax = std::numeric_limits<double>::lowest();
            double xmin = std::numeric_limits<double>::max();
            double ymax = std::numeric_limits<double>::lowest();
            double ymin = std::numeric_limits<double>::max();
            double zmax = std::numeric_limits<double>::lowest();
            double zmin = std::numeric_limits<double>::max();
            double zrange = 0;
            double meanabsht = 0;
            double rmax = 0, rmin = 255, gmax = 0, gmin = 255, bmax = 0, bmin = 255;
            double rsum = 0, gsum = 0, bsum = 0;
            //geometric
            double ptness = 0.0;
            double curveness = 0.0;
            double surfaceness = 0.0;
            double linearity = 0.0;
            double planarity = 0.0;
            double volumetric = 0.0;

            // Calculate features
            for (const auto& point : cloud->points) {
                double x = point.x, y = point.y, z = point.z;
                xmax = std::max(xmax, x);
                xmin = std::min(xmin, x);
                ymax = std::max(ymax, y);
                ymin = std::min(ymin, y);
                zmax = std::max(zmax, z);
                zmin = std::min(zmin, z);
                meanabsht += std::abs(z);

                // Color features
                rmax = std::max(rmax, static_cast<double>(point.r));
                rmin = std::min(rmin, static_cast<double>(point.r));
                gmax = std::max(gmax, static_cast<double>(point.g));
                gmin = std::min(gmin, static_cast<double>(point.g));
                bmax = std::max(bmax, static_cast<double>(point.b));
                bmin = std::min(bmin, static_cast<double>(point.b));

                rsum += point.r;
                gsum += point.g;
                bsum += point.b;
            }
            zrange = zmax - zmin;
            if (no_of_points > 0) meanabsht /= no_of_points;

            double rmean = rsum / no_of_points;
            double gmean = gsum / no_of_points;
            double bmean = bsum / no_of_points;

            // Calculate NDVI
            double ndvi = gmean - (rmean + bmean) / 2.0;

            // Write the features to the CSV file
            csv_file << segment_id;
            if (std::find(features_need.begin(), features_need.end(), "Geometrical") != features_need.end()) {
                csv_file << "," << no_of_points << "," << xmax << "," << xmin << "," << ymax << "," << ymin << "," << zmax << "," << zmin << "," << zrange;
            }
            if (std::find(features_need.begin(), features_need.end(), "Statistical") != features_need.end()) {
                csv_file << "," << meanabsht;
            }
            if (std::find(features_need.begin(), features_need.end(), "Shape") != features_need.end()) {
                // Calculate shape features
                ptness = FindPtness(filepath);
                curveness = FindCurveness(filepath);
                surfaceness = FindSurfaceness(filepath);
                linearity = FindLinearity(filepath);
                planarity = FindPlanarity(filepath);
                volumetric = FindVolumetric(filepath);

                csv_file << "," << ptness << "," << curveness << "," << surfaceness << "," << linearity << "," << planarity << "," << volumetric;
            }
            if (std::find(features_need.begin(), features_need.end(), "Color") != features_need.end()) {
                csv_file << "," << rmax << "," << rmin << "," << gmax << "," << gmin << "," << bmax << "," << bmin << "," << rmean << "," << gmean << "," << bmean << "," << ndvi;
            }
            // Other features would be similarly calculated and written here

            csv_file << "\n";
        }
    }

    csv_file.close();
}



double FeatureExtract::FindPtness(std::string filepath)
{

    /*
        This function may be wrong:
        Definition: Ratio of the smallest to the largest eigenvalue from PCA.
        Purpose: Indicates how pointy the segment is.
    */

    // Load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud) == -1) {
        std::cerr << "Couldn't read file %s \n", filepath;
        return (0.0);
    }

    // Compute centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    // Compute covariance matrix
    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance_matrix);

    // Compute eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
    if (eigen_solver.info() != Eigen::Success) {
        std::cerr <<"Failed to compute eigenvalues!\n";
        return (0.0);
    }

    Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();

    // Calculate ptness
    double ptness = eigenvalues[0] / eigenvalues[2]; // Ratio of smallest to largest eigenvalue

    if (std::isnan(ptness)) {
        ptness = 0;
    }

    return ptness;
}

double FeatureExtract::FindCurveness(std::string filepath)
{

    /*
        Definition: Difference between the largest and middle eigenvalue divided by the largest eigenvalue.
        Purpose: Measures the curvature of the segment.
    */

    // Read the point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(filepath, *point_cloud) == -1) {
        PCL_ERROR("Couldn't read file %s.\n", filepath);
        return 0;
    }

    // Program to find curveness goes here
    double curveness = 0.0;

    return curveness;
}

double FeatureExtract::FindSurfaceness(std::string filepath)
{
    /*
        Defiinition: Difference between the smallest and middle eigenvalue divided by the largest eigenvalue.
        Purpose: Indicates the surface-like quality of the segment.
    */

    // Read the point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(filepath, *point_cloud) == -1) {
        PCL_ERROR("Couldn't read file %s.\n", filepath);
        return 0;
    }

    // Program to find surfaceness goes here
    double surfaceness = 0.0;

    return surfaceness;
}

double FeatureExtract::FindLinearity(std::string filepath)
{
    /*
        Definition: (Largest eigenvalue - Middle eigenvalue) / Largest eigenvalue.
        purpose: Represents how linear the segment is.
    */

    // Read the point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(filepath, *point_cloud) == -1) {
        PCL_ERROR("Couldn't read file %s.\n", filepath);
        return 0;
    }

    // Program to find linearity goes here
    double linearity = 0.0;

    return linearity;
}

double FeatureExtract::FindPlanarity(std::string filepath)
{
    /*
        Definition: (Middle eigenvalue - Smallest eigenvalue) / Largest eigenvalue.
        purpose: Indicates how planar the segment is.
    */

    // Read the point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(filepath, *point_cloud) == -1) {
        PCL_ERROR("Couldn't read file %s.\n", filepath);
        return 0;
    }

    // Program to find planarity goes here
    double planarity = 0.0;

    return planarity;
}

double FeatureExtract::FindVolumetric(std::string filepath)
{
    /*
        Definition: Volume occupied by the points in the segment.
        Purpose: Provides the 3D space occupied by the segment.
    */

    // Read the point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(filepath, *point_cloud) == -1) {
        PCL_ERROR("Couldn't read file %s.\n", filepath);
        return 0;
    }

    // Program to find volumetric goes here
    double volumetric = 0.0;

    return volumetric;
}

