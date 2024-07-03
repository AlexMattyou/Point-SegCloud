#include "MainFrame.h"

void MainFrame::AllFeatureExtract(const std::string& project_files, const std::vector<std::string>& features_need) {
    // Open the CSV file for writing
    std::string csv_path = project_files + "/feature.csv";
    std::ofstream csv_file(csv_path, std::ios::out | std::ios::trunc);

    if (!csv_file.is_open()) {
        std::cerr << "Error: Could not open CSV file " << csv_path << std::endl;
        return;
    }
    Log("Created csv: " + project_files + "/feature.csv");

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
        csv_file << ",Density,HeightVariance";
    }
    if (std::find(features_need.begin(), features_need.end(), "Color") != features_need.end()) {
        csv_file << ",Rmax,Rmin,Gmax,Gmin,Bmax,Bmin,Rmean,Gmean,Bmean,NDVI";
    }
    if (std::find(features_need.begin(), features_need.end(), "Area") != features_need.end()) {
        csv_file << ",Aconv,Apoly,Bpoly";
    }
    csv_file << "\n";
    ProgressBar(1);
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

            //geometrical
            int no_of_points = cloud->points.size();
            double xmax = std::numeric_limits<double>::lowest();
            double xmin = std::numeric_limits<double>::max();
            double ymax = std::numeric_limits<double>::lowest();
            double ymin = std::numeric_limits<double>::max();
            double zmax = std::numeric_limits<double>::lowest();
            double zmin = std::numeric_limits<double>::max();
            double zrange = 0;

            // stastical
            double meanabsht = 0;

            //color
            double rmax = 0, rmin = 255, gmax = 0, gmin = 255, bmax = 0, bmin = 255;
            double rsum = 0, gsum = 0, bsum = 0;

            //Density
            Density = 0.0;
            HeightVariance = 0.0;

            //Area
            double Aconv = 0.0;
            double Apoly = 0.0;
            double Bpoly = 0.0;

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
            double ndvi = 0;

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
                ptness = 0;
                curveness = 0;
                surfaceness = 0;
                linearity = 0;
                planarity = 0;
                volumetric = 0;
                arma::mat data = PclToArmaMatrix(filepath);
                ComputeShapeFeatures(data);

                csv_file << "," << ptness << "," << curveness << "," << surfaceness << "," << linearity << "," << planarity << "," << volumetric;
            }
            if (std::find(features_need.begin(), features_need.end(), "Density") != features_need.end()) {
                // Calculate density features
                Density = 0.0;
                HeightVariance = 0.0;
                ComputeDensityFeatures(filepath);
                
                csv_file << "," << Density << "," << HeightVariance;
            }
            if (std::find(features_need.begin(), features_need.end(), "Color") != features_need.end()) {
                csv_file << "," << rmax << "," << rmin << "," << gmax << "," << gmin << "," << bmax << "," << bmin << "," << rmean << "," << gmean << "," << bmean << "," << ndvi;
            }
            // Other features would be similarly calculated and written here
            if (std::find(features_need.begin(), features_need.end(), "Area") != features_need.end()) {
                // Calculate shape features
                Aconv = 0.0;
                Apoly = 0.0;
                Bpoly = 0.0;
                ComputeAreaFeatures(filepath);
                csv_file << "," << Aconv << "," << Apoly << "," << Bpoly;
            }
            csv_file << "\n";
        }
    }
    Log("Features extracted successfully");

    csv_file.close();
}

arma::mat MainFrame::PclToArmaMatrix(const std::string& filePath) {
    // Load point cloud data from PCD file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filePath, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", filePath.c_str());
        throw std::runtime_error("Error loading PCD file");
    }

    // Convert to Armadillo matrix
    arma::mat data(cloud->points.size(), 3);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        data(i, 0) = cloud->points[i].x;
        data(i, 1) = cloud->points[i].y;
        data(i, 2) = cloud->points[i].z;
    }

    return data;
}


void MainFrame::ComputeShapeFeatures(const arma::mat& data) {
    // Center the data
    arma::mat centered = data.each_row() - arma::mean(data, 0);

    // Compute the covariance matrix
    arma::mat cov = arma::cov(centered);

    // Perform eigen decomposition
    arma::vec eigval;
    arma::mat eigvec;

    // Check if eigen decomposition fails
    if (!arma::eig_sym(eigval, eigvec, cov)) {
        return;  // Exit the function or handle the error appropriately
    }

    // Ensure eigenvalues are valid and populated
    if (eigval.is_empty() || eigval.size() < 3) {
        return;  // Exit the function or handle the error appropriately
    }

    // Sort eigenvalues in ascending order
    try {
        eigval = arma::sort(eigval);
    }
    catch (const std::runtime_error& e) {
        return;  // Exit the function or handle the error appropriately
    }

    // Extract eigenvalues
    double lambda1 = eigval(0);
    double lambda2 = eigval(1);
    double lambda3 = eigval(2);

    // Handle division by zero errors
    if (lambda3 != 0.0) {
        ptness = lambda1 / lambda3;
        surfaceness = (lambda2 - lambda1) / lambda3;
        linearity = (lambda3 - lambda2) / lambda3;
        planarity = (lambda2 - lambda1) / lambda3;
        volumetric = lambda1 * lambda2 * lambda3;
    }
    else {
        // Handle the case where lambda3 is zero
        ptness = 0.0;
        surfaceness = 0.0;
        linearity = 0.0;
        planarity = 0.0;
        volumetric = 0.0;
    }

    // Calculate curveness (handling division by zero separately)
    double sum_lambda = lambda1 + lambda2 + lambda3;
    if (sum_lambda != 0.0) {
        curveness = lambda1 / sum_lambda;
    }
    else {
        curveness = 0.0;
    }
}

void MainFrame::ComputeDensityFeatures(const std::string& filePath) {
    // Load point cloud data from PCD file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filePath, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", filePath.c_str());
        Log("Error loading PCD file");
        return;  // Exit if file loading fails
    }

    // Compute density
    Density = ComputeDensity(cloud);

    // Compute height variance
    HeightVariance = ComputeHeightVariance(cloud);

}

double MainFrame::ComputeDensity(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    // Calculate the volume of the convex hull
    pcl::ConvexHull<pcl::PointXYZRGB> hull;
    hull.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::Vertices> polygons;
    hull.reconstruct(*hull_cloud, polygons);

    if (hull_cloud->points.empty() || polygons.empty()) {
        return 0.0;
    }

    // Calculate the volume of the convex hull using the tetrahedron method
    double hull_volume = 0.0;
    for (const auto& polygon : polygons) {
        if (polygon.vertices.size() != 3) continue; // Only process triangles

        const pcl::PointXYZRGB& p1 = hull_cloud->points[polygon.vertices[0]];
        const pcl::PointXYZRGB& p2 = hull_cloud->points[polygon.vertices[1]];
        const pcl::PointXYZRGB& p3 = hull_cloud->points[polygon.vertices[2]];

        // Calculate the volume of the tetrahedron formed by the triangle and the origin
        double volume = std::abs(p1.x * (p2.y * p3.z - p3.y * p2.z) -
            p1.y * (p2.x * p3.z - p3.x * p2.z) +
            p1.z * (p2.x * p3.y - p3.x * p2.y)) / 6.0;
        hull_volume += volume;
    }

    if (hull_volume == 0.0) {
        return 0.0;
    }

    // Calculate density
    double density = static_cast<double>(cloud->size()) / hull_volume;

    return density;
}

double MainFrame::ComputeHeightVariance(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    // Compute mean Z value
    double mean_z = 0.0;
    for (size_t i = 0; i < cloud->size(); ++i) {
        mean_z += cloud->points[i].z;
    }
    mean_z /= cloud->size();

    // Compute variance
    double variance = 0.0;
    for (size_t i = 0; i < cloud->size(); ++i) {
        double diff = cloud->points[i].z - mean_z;
        variance += diff * diff;
    }
    variance /= cloud->size();

    return variance;
}

void MainFrame::ComputeAreaFeatures(const std::string& filePath) {
    // Load point cloud data from PCD file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filePath, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", filePath.c_str());
        wxMessageBox("Error loading PCD file");
        Aconv = 0;
        Apoly = 0;
        Bpoly = 0;
        return;
    }

    // Project point cloud onto XY plane for Aconv and Apoly
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud->points) {
        cloud_xy->points.emplace_back(point.x, point.y, 0.0);
    }

    // Calculate Aconv (area of convex hull in XY plane)
    try {
        pcl::ConvexHull<pcl::PointXYZ> hull;
        pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<pcl::Vertices> polygons;
        hull.setInputCloud(cloud_xy);
        hull.reconstruct(*hull_cloud, polygons);
        Aconv = hull.getTotalArea();

        // Calculate Apoly (area of polygon in XY plane)
        Apoly = 0.0;
        for (const auto& polygon : polygons) {
            if (polygon.vertices.size() < 3) continue; // Skip degenerate polygons

            double area = 0.0;
            for (size_t i = 1; i < polygon.vertices.size() - 1; ++i) {
                const auto& p0 = hull_cloud->points[polygon.vertices[0]];
                const auto& p1 = hull_cloud->points[polygon.vertices[i]];
                const auto& p2 = hull_cloud->points[polygon.vertices[i + 1]];
                area += std::abs(p0.x * (p1.y - p2.y) + p1.x * (p2.y - p0.y) + p2.x * (p0.y - p1.y)) / 2.0;
            }
            Apoly += area;
        }
    }
    catch (const std::exception& e) {
        Aconv = 0;
        Apoly = 0;
    }

    // Project point cloud onto XZ plane for Bpoly
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xz(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud->points) {
        cloud_xz->points.emplace_back(point.x, 0.0, point.z);
    }

    // Calculate Bpoly (area of polygon in XZ plane)
    try {
        pcl::ConvexHull<pcl::PointXYZ> hull_xz;
        pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud_xz(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<pcl::Vertices> polygons;
        hull_xz.setInputCloud(cloud_xz);
        hull_xz.reconstruct(*hull_cloud_xz, polygons);
        Bpoly = 0.0;
        for (const auto& polygon : polygons) {
            if (polygon.vertices.size() < 3) continue; // Skip degenerate polygons

            double area = 0.0;
            for (size_t i = 1; i < polygon.vertices.size() - 1; ++i) {
                const auto& p0 = hull_cloud_xz->points[polygon.vertices[0]];
                const auto& p1 = hull_cloud_xz->points[polygon.vertices[i]];
                const auto& p2 = hull_cloud_xz->points[polygon.vertices[i + 1]];
                area += std::abs(p0.x * (p1.z - p2.z) + p1.x * (p2.z - p0.z) + p2.x * (p0.z - p1.z)) / 2.0;
            }
            Bpoly += area;
        }
    }
    catch (const std::exception& e) {
        Bpoly = 0;
    }
}
