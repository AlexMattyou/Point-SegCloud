#include "PointCloud.h"

// Typedefs for simplicity
typedef pcl::PointXYZ PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

// Global variables for visualization control
bool show_normals = false, normals_changed = false;
bool show_adjacency = false;
bool show_supervoxels = false;
bool show_help = true;
float normals_scale;

int PointCloud::CheckAndCreateProject(const std::string& folderName) {
    std::string fullFolderPath = "Projects/" + folderName;
    std::filesystem::path folderPath(fullFolderPath);

    // Check if the folder exists
    if (!std::filesystem::exists(folderPath)) {
        // Folder does not exist, create it
        if (std::filesystem::create_directory(folderPath)) {
            std::cout << "Folder created: " << fullFolderPath << std::endl;
        }
        else {
            std::cerr << "Failed to create folder: " << fullFolderPath << std::endl;
            return -1; // Return an error code if folder creation fails
        }
    }
    else {
        std::cout << "Folder already exists: " << fullFolderPath << std::endl;
    }
    return 0; // Return 0 if the folder exists or was created successfully
}

int PointCloud::Txt2pcd(std::string& inputFile, std::string& outputFile, std::unordered_map<std::string, int>& columnIndex) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.is_dense = false;

    std::cerr << "Opening the text file...\n";
    std::ifstream infile(inputFile);
    if (!infile.is_open()) {
        std::cerr << "Error opening file: " << inputFile << std::endl;
        return -1;
    }

    std::cerr << "Reading header from file...\n";
    std::string line;
    std::getline(infile, line);
    std::istringstream headerStream(line);
    std::string column;
    int index = 0;
    bool isHeader = true;

    // Check if the first line contains headers
    while (headerStream >> column) {
        if (std::isdigit(column[0]) || column[0] == '-') {
            isHeader = false;
            break;
        }
        columnIndex[column] = index++;
    }

    if (!isHeader) {
        // Rewind file if no header
        infile.clear();
        infile.seekg(0);
        
        // Read the first data line to determine the number of columns
        std::getline(infile, line);
        std::istringstream dataStream(line);
        std::vector<float> values((std::istream_iterator<float>(dataStream)), std::istream_iterator<float>());
        
        if (values.size() == 6) {
            columnIndex = {{"X", 0}, {"Y", 1}, {"Z", 2}, {"Red", 3}, {"Green", 4}, {"Blue", 5}};
            index = 6;
        } else if (values.size() == 3) {
            columnIndex = {{"X", 0}, {"Y", 1}, {"Z", 2}};
            index = 3;
        } else {
            std::cerr << "Unexpected number of columns in the data file." << std::endl;
            return -1;
        }

        // Rewind again to read from the beginning
        infile.clear();
        infile.seekg(0);
    }

    bool hasColor = columnIndex.find("Red") != columnIndex.end() &&
                    columnIndex.find("Green") != columnIndex.end() &&
                    columnIndex.find("Blue") != columnIndex.end();

    std::cerr << "Reading points from file...\n";
    std::vector<float> values(index);
    size_t numPoints = 0;
    while (std::getline(infile, line)) {
        std::istringstream lineStream(line);
        for (int i = 0; i < index; ++i) {
            lineStream >> values[i];
        }

        pcl::PointXYZRGB point;
        point.x = values[columnIndex["X"]];
        point.y = values[columnIndex["Y"]];
        point.z = values[columnIndex["Z"]];
        if (hasColor) {
            point.r = static_cast<uint8_t>(values[columnIndex["Red"]]);
            point.g = static_cast<uint8_t>(values[columnIndex["Green"]]);
            point.b = static_cast<uint8_t>(values[columnIndex["Blue"]]);
        } else {
            point.r = point.g = point.b = 255; // Default color if not provided
        }

        cloud.points.emplace_back(point);
        ++numPoints;
    }

    infile.close();

    cloud.width = numPoints;
    cloud.height = 1;

    std::cerr << "Converting text to binary PCD...\n";
    pcl::io::savePCDFileASCII(outputFile, cloud);
    std::cerr << "Saved " << cloud.points.size() << " data points to " << outputFile << std::endl;

    return 0;
}


int PointCloud::Pcd2txt(std::string& inputFile, std::string& outputFile) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    // Load the PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(inputFile, cloud) == -1) {
        std::cerr << "Error loading file: " << inputFile << std::endl;
        return -1;
    }

    // Open the text file for writing
    std::ofstream outfile(outputFile);
    if (!outfile.is_open()) {
        std::cerr << "Error opening file: " << outputFile << std::endl;
        return -1;
    }

    // Write the header
    outfile << "X Y Z";
    if (cloud.points[0].r != 0 || cloud.points[0].g != 0 || cloud.points[0].b != 0) {
        outfile << " Red Green Blue";
    }
    outfile << std::endl;

    // Write the points
    for (const auto& point : cloud.points) {
        outfile << point.x << " " << point.y << " " << point.z;
        if (point.r != 0 || point.g != 0 || point.b != 0) {
            outfile << " " << static_cast<int>(point.r) << " " << static_cast<int>(point.g) << " " << static_cast<int>(point.b);
        }
        outfile << std::endl;
    }

    outfile.close();

    std::cerr << "Converted PCD to text and saved to " << outputFile << std::endl;
    return 0;
}


int PointCloud::SaveNormals(const std::string& inputFile, const std::string& outputFile) {
    // Load the input point cloud
    std::cerr << "Opening the input PCD file: " << inputFile << "...\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(inputFile, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", inputFile.c_str());
        return -1;
    }
    std::cerr << "Successfully loaded " << cloud->size() << " points.\n";

    // Estimate normals
    std::cerr << "Estimating normals...\n";
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setSearchMethod(tree);
    normalEstimation.setRadiusSearch(1.0); // Adjust the radius as per your requirements
    normalEstimation.compute(*normals);
    std::cerr << "Normals computed for " << normals->size() << " points.\n";

    // Concatenate XYZ and normal fields
    std::cerr << "Concatenating XYZ and normal fields...\n";
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);

    // Save output
    std::cerr << "Saving output to PCD file: " << outputFile << "...\n";
    if (pcl::io::savePCDFile(outputFile, *cloudWithNormals) == -1) {
        PCL_ERROR("Couldn't save file %s\n", outputFile.c_str());
        return -1;
    }
    std::cerr << "Process completed successfully, saved output to " << outputFile << std::endl;

    return 0;
}




void PointCloud::segmentPointCloud(const std::string& input_cloud_file,
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
    const std::string& saving_type){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr input_normals_ptr(new pcl::PointCloud<pcl::Normal>);

    if (pcl::io::loadPCDFile(input_cloud_file, *input_cloud_ptr) == -1) {
        PCL_ERROR("Error reading input point cloud file %s\n", input_cloud_file.c_str());
        return;
    }

    if (pcl::io::loadPCDFile(input_normals_file, *input_normals_ptr) == -1) {
        PCL_ERROR("Error reading input normals cloud file %s\n", input_normals_file.c_str());
        return;
    }

    pcl::SupervoxelClustering<pcl::PointXYZRGB> super(voxel_resolution, seed_resolution);
    super.setInputCloud(input_cloud_ptr);
    super.setNormalCloud(input_normals_ptr);
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);

    std::cerr << "Extracting supervoxels...\n";
    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> supervoxel_clusters;
    super.extract(supervoxel_clusters);

    if (use_supervoxel_refinement) {
        std::cerr << "Refining supervoxels...\n";
        super.refineSupervoxels(2, supervoxel_clusters);
    }

    std::cerr << "Getting supervoxel adjacency...\n";
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    std::cerr << "Creating supervoxel centroid with normals cloud...\n";
    pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud =
        pcl::SupervoxelClustering<pcl::PointXYZRGB>::makeSupervoxelNormalCloud(supervoxel_clusters);

    std::cerr << "Performing LCCP segmentation...\n";
    AccessibleLCCPSegmentation<pcl::PointXYZRGB> lccp;
    lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
    lccp.setSanityCheck(use_sanity_criterion);
    lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
    lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
    lccp.segment();

    if (min_segment_size > 0) {
        std::cerr << "Merging small segments...\n";
        lccp.setMinSegmentSize(min_segment_size);
        lccp.mergeSmallSegments();
    }

    std::cerr << "Relabeling cloud...\n";
    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud()->makeShared();
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
    lccp.relabelCloud(*lccp_labeled_cloud);

    std::cerr << "Saving segments...\n";
    saveSegmentedClouds(lccp_labeled_cloud, output_folder, colorful_segmentation, input_cloud_file, saving_type);
}

void PointCloud::saveSegmentedClouds(const pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_cloud,
    const std::string& output_folder, bool colorful_segmentation, const std::string& raw_file, const std::string& saving_type) {

    clearFolder(output_folder);

    if (!std::filesystem::exists(output_folder)) {
        std::cerr << "Error: Output folder does not exist: " << output_folder << std::endl;
        return;
    }

    std::map<uint32_t, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segment_map;

    if (colorful_segmentation && !raw_file.empty()) {
        // Load raw cloud for RGB information
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if (pcl::io::loadPCDFile(raw_file, *raw_cloud) == -1) {
            PCL_ERROR("Error reading raw point cloud file %s\n", raw_file.c_str());
            return;
        }

        // Create a KdTree for fast spatial searching
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        kdtree.setInputCloud(raw_cloud);

        // Iterate through labeled points and find corresponding RGB values
        for (const auto& point : labeled_cloud->points) {
            if (segment_map.find(point.label) == segment_map.end()) {
                segment_map[point.label] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            }

            pcl::PointXYZRGB search_point;
            search_point.x = point.x;
            search_point.y = point.y;
            search_point.z = point.z;

            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);

            if (kdtree.nearestKSearch(search_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                // Assign RGB values from raw cloud
                pcl::PointXYZRGB colored_point = raw_cloud->points[pointIdxNKNSearch[0]];
                segment_map[point.label]->points.push_back(colored_point);
            }
        }
    }
    else {
        // Non-colorful segmentation or no raw file provided
        for (const auto& point : labeled_cloud->points) {
            if (segment_map.find(point.label) == segment_map.end()) {
                segment_map[point.label] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            }
            segment_map[point.label]->points.push_back(pcl::PointXYZRGB(point.x, point.y, point.z));
        }
    }

    // Save segmented clouds
    for (const auto& segment : segment_map) {
        if (segment.second->points.empty()) {
            std::cerr << "Warning: Segment " << segment.first << " has no points." << std::endl;
            continue;
        }

        segment.second->width = segment.second->points.size();
        segment.second->height = 1;
        segment.second->is_dense = true;

        std::string filename = output_folder + "/segment_" + std::to_string(segment.first) + "." + saving_type;

        std::cerr << "Saving segment " << segment.first << " to " << filename << "...\n";

        try {
            if (saving_type == "txt") {
                std::ofstream ofs(filename);
                if (!ofs.is_open()) {
                    std::cerr << "Error opening file: " << filename << std::endl;
                    continue;
                }
                ofs << "X Y Z Red Green Blue\n";
                for (const auto& point : segment.second->points) {
                    ofs << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " " << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << "\n";
                }
                ofs.close();
            }
            else {
                int result = pcl::io::savePCDFileASCII(filename, *segment.second);  // Save as ASCII PCD
                if (result == -1) {
                    std::cerr << "Error saving PCD file: " << filename << std::endl;
                }
                else {
                    std::cerr << "Successfully saved segment " << segment.first << " to " << filename << std::endl;
                }
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Exception while saving segment " << segment.first << ": " << e.what() << std::endl;
        }
    }

    std::cerr << "Process completed successfully\n";
}

void PointCloud::clearFolder(const std::filesystem::path& folderPath)
{
    if (!std::filesystem::exists(folderPath)) {
        std::cerr << "Error: Folder '" << folderPath << "' does not exist." << std::endl;
        return;
    }

    for (const auto& entry : std::filesystem::directory_iterator(folderPath)) {
        if (std::filesystem::is_regular_file(entry)) {
            try {
                std::filesystem::remove(entry);
                std::cout << "Deleted file: " << entry.path() << std::endl; // Optional: Log deleted files
            }
            catch (const std::filesystem::filesystem_error& e) {
                std::cerr << "Error deleting file '" << entry.path() << "': " << e.what() << std::endl;
            }
        }
        else if (std::filesystem::is_directory(entry)) {
            // Recursively clear subdirectories (optional, comment out if not desired)
            // clearFolder(entry); // Uncomment to recursively clear subdirectories
        }
    }
}
