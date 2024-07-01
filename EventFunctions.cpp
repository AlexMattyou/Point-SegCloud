#include "MainFrame.h"

void MainFrame::OnCloseEvt(wxCloseEvent& event) {
    SaveProjectOptions();
    ConfirmClose();
}

void MainFrame::OnFileDropEvt(wxDropFilesEvent& event) {

    event.Skip();
}

void MainFrame::OnMenuNew(wxCommandEvent& event) {

    // Clear project name input
    project_name_input->Clear();

    // Clear dataset input
    dataset_input->SetPath(wxEmptyString);
    event.Skip();
}

void MainFrame::OnMenuOpen(wxCommandEvent& event) {

    // Create a new wxFrame for listing projects
    wxFrame* projectListFrame = new wxFrame(this, wxID_ANY, "Project List", wxDefaultPosition, wxSize(300, 400));

    // Set wxArt icon
    wxIcon icon;
    icon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_FILE_OPEN)); // Replace with the desired wxArt icon type
    projectListFrame->SetIcon(icon);

    // Create a wxScrolledWindow to host the project list
    wxScrolledWindow* scrolledWindow = new wxScrolledWindow(projectListFrame, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL);

    // Create a wxListBox to display project names
    wxListBox* listBox = new wxListBox(scrolledWindow, wxID_ANY, wxDefaultPosition, wxDefaultSize);

    // Get all directories (projects) from "Projects/" folder
    wxString projectsFolder = "Projects/";
    wxDir projectsDir(projectsFolder);

    if (projectsDir.IsOpened()) {
        wxString projectName;
        bool cont = projectsDir.GetFirst(&projectName, wxEmptyString, wxDIR_DIRS);
        while (cont) {
            // Add each project name to the list box
            listBox->Append(projectName);
            cont = projectsDir.GetNext(&projectName);
        }
    }

    // Bind selection event
    listBox->Bind(wxEVT_LISTBOX, &MainFrame::OnProjectSelection, this);

    // Set up layout
    wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(listBox, 1, wxEXPAND);
    scrolledWindow->SetSizer(sizer);
    scrolledWindow->FitInside();

    // Show the project list frame
    projectListFrame->Show();

    event.Skip();
}

void MainFrame::OnProjectSelection(wxCommandEvent& event) {
    wxListBox* listBox = dynamic_cast<wxListBox*>(event.GetEventObject());
    if (listBox) {
        int selectedIdx = listBox->GetSelection();
        if (selectedIdx != wxNOT_FOUND) {
            wxString projectName = listBox->GetString(selectedIdx);
            project_name = projectName;

            // Update project_name_input
            project_name_input->SetValue(projectName);

            // Update dataset_input
            wxString datasetPath = "Projects/" + projectName + "/raw.pcd";
            if (!wxFileExists(datasetPath)) {
                datasetPath = "Projects/" + projectName + "/raw.txt";
                if (!wxFileExists(datasetPath)) {
                    datasetPath = wxEmptyString;
                }
            }
            dataset_input->SetPath(datasetPath);
        }
    }
}

void MainFrame::OnMenuDelete(wxCommandEvent& event) {
    // Get project name from input field
    project_name = project_name_input->GetValue();

    // Check if project name is empty
    if (project_name.empty()) {
        // Show a message or do nothing because project name is not provided
        wxMessageBox("Please enter a project name to delete.", "Delete Project", wxOK | wxICON_INFORMATION);
        return;  // Exit function early
    }

    // Construct the folder path
    wxString projectFolderPath = "Projects/" + project_name;

    // Convert wxString to std::string for filesystem operations
    std::string projectFolderStdString = std::string(projectFolderPath.mb_str());

    // Confirm deletion with a message box
    wxMessageDialog dialog(this, "Are you sure you want to delete this project?", "Confirm Deletion", wxYES_NO | wxICON_QUESTION);
    int result = dialog.ShowModal();

    if (result == wxID_YES) {
        // Attempt to delete the directory and its contents
        try {
            std::filesystem::remove_all(projectFolderStdString);
            status_bar->SetStatusText("Project folder deleted successfully.");
        }
        catch (const std::filesystem::filesystem_error& e) {
            status_bar->SetStatusText("Failed to delete project folder: " + wxString(e.what(), wxConvUTF8));
        }
    }

    event.Skip();  // Continue handling the event normally
}


void MainFrame::OnMenuExit(wxCommandEvent& event) {
    SaveProjectOptions();
    ConfirmClose();
}

void MainFrame::OnHelpDocs(wxCommandEvent& event) {
    wxLaunchDefaultBrowser("https://github.com/AlexMattyou/PointCloudSegmentation/blob/master/README.md");
    event.Skip();
}

void MainFrame::OnHelpReport(wxCommandEvent& event) {
    wxLaunchDefaultBrowser("https://github.com/AlexMattyou/PointCloudSegmentation/issues");
    event.Skip();
}

void MainFrame::OpenInFileManager(const wxString& path) {
#ifdef _WIN32
    wxExecute("explorer " + path);
#elif __APPLE__
    wxExecute("open " + path);
#else
    wxExecute("xdg-open " + path);
#endif
}

void MainFrame::ViewProjectEvt(wxMouseEvent& event) {
    // Get project name from input field
    project_name = project_name_input->GetValue();

    // Check if project name is empty
    if (project_name.empty()) {
        wxMessageBox("Please enter a project name to view the project folder.", "View Project", wxOK | wxICON_INFORMATION);
        return;
    }

    // Construct the project folder path
    wxString projectFolderPath = "Projects/" + project_name;

    // Get the absolute path
    wxFileName projectFolderAbsPath(projectFolderPath);
    wxString absolutePath = projectFolderAbsPath.GetFullPath();

    // Check if project folder exists
    if (!std::filesystem::exists(std::string(absolutePath.mb_str()))) {
        wxMessageBox("Project folder not found. Please check the project name.", "View Project", wxOK | wxICON_INFORMATION);
        return;
    }

    // Open the project folder in the file manager
    OpenInFileManager(absolutePath);

    event.Skip();
}

void MainFrame::ViewSegmentationEvt(wxMouseEvent& event) {
    // Get project name from input field
    project_name = project_name_input->GetValue();

    // Check if project name is empty
    if (project_name.empty()) {
        wxMessageBox("Please enter a project name to view its segments.", "View Segments", wxOK | wxICON_INFORMATION);
        return;
    }

    // Construct the segments folder path
    wxString segmentsFolderPath = "Projects/" + project_name + "/Segments";

    // Get the absolute path
    wxFileName segmentsFolderAbsPath(segmentsFolderPath);
    wxString absolutePath = segmentsFolderAbsPath.GetFullPath();

    // Check if segments folder exists
    if (!std::filesystem::exists(std::string(absolutePath.mb_str()))) {
        wxMessageBox("Segments folder not found. Please perform segmentation first.", "View Segments", wxOK | wxICON_INFORMATION);
        return;
    }

    // Open the segments folder in the file manager
    OpenInFileManager(absolutePath);

    event.Skip();
}

void MainFrame::OpenFeaturesEvt(wxMouseEvent& event) {
    // Get project name from input field
    project_name = project_name_input->GetValue();

    // Check if project name is empty
    if (project_name.empty()) {
        wxMessageBox("Please enter a project name to view the feature file.", "Open Feature File", wxOK | wxICON_INFORMATION);
        return;
    }

    // Construct the feature file path
    wxString featureFilePath = "Projects/" + project_name + "/feature.csv";

    // Get the absolute path
    wxFileName featureFileAbsPath(featureFilePath);
    wxString absolutePath = featureFileAbsPath.GetFullPath();

    // Check if feature file exists
    if (!std::filesystem::exists(std::string(absolutePath.mb_str()))) {
        wxMessageBox("Feature file not found. Please check if the feature file exists in the project folder.", "Open Feature File", wxOK | wxICON_INFORMATION);
        return;
    }

    // Open the feature file in the default application
    OpenInFileManager(absolutePath);

    event.Skip();
}


void MainFrame::StartProcessEvt(wxMouseEvent& event) {
    SaveProjectOptions();
}

////////////////////////////////////////////////////////////////////////////////////

void MainFrame::ConfirmClose() {
    wxMessageDialog confirmDialog(
        this,
        "Are you sure you want to exit?",
        "Confirm Exit",
        wxYES_NO | wxNO_DEFAULT | wxICON_QUESTION
    );

    if (confirmDialog.ShowModal() == wxID_YES) {
        Destroy();  // Destroy the frame, avoids triggering the close event again
    }
}

void MainFrame::SaveOptions() {
    project_name = std::string(project_name_input->GetValue().mb_str());

    // Get the absolute path of the dataset
    wxFileName datasetFile(dataset_input->GetPath());
    datasetFile.MakeAbsolute();
    dataset = std::string(datasetFile.GetFullPath().mb_str());

    // Check if project_name or dataset is empty
    if (project_name.empty() || dataset.empty()) {
        wxMessageBox("Project name or dataset path cannot be empty.", "Invalid Input", wxOK | wxICON_WARNING);
        return;
    }

    wxString segment_choice = segmant_type_input->GetStringSelection();
    segment_type = segment_choice.SubString(0, 3).ToStdString(); // Store only first 4 characters

    // Validate and convert input values
    try {
        voxel_resolution = std::stof(voxel_input->GetValue().ToStdString());
    }
    catch (const std::exception&) {
        wxMessageBox("Invalid voxel resolution. Please enter a valid number.", "Invalid Input", wxOK | wxICON_WARNING);
        return;
    }

    try {
        seed_resolution = std::stof(seed_input->GetValue().ToStdString());
    }
    catch (const std::exception&) {
        wxMessageBox("Invalid seed resolution. Please enter a valid number.", "Invalid Input", wxOK | wxICON_WARNING);
        return;
    }

    try {
        color_importance = std::stof(color_input->GetValue().ToStdString());
    }
    catch (const std::exception&) {
        wxMessageBox("Invalid color importance. Please enter a valid number.", "Invalid Input", wxOK | wxICON_WARNING);
        return;
    }

    try {
        spatial_importance = std::stof(spatial_input->GetValue().ToStdString());
    }
    catch (const std::exception&) {
        wxMessageBox("Invalid spatial importance. Please enter a valid number.", "Invalid Input", wxOK | wxICON_WARNING);
        return;
    }

    try {
        normal_importance = std::stof(normal_input->GetValue().ToStdString());
    }
    catch (const std::exception&) {
        wxMessageBox("Invalid normal importance. Please enter a valid number.", "Invalid Input", wxOK | wxICON_WARNING);
        return;
    }

    try {
        concavity_tolerance_threshold = std::stof(concavity_input->GetValue().ToStdString());
    }
    catch (const std::exception&) {
        wxMessageBox("Invalid concavity tolerance threshold. Please enter a valid number.", "Invalid Input", wxOK | wxICON_WARNING);
        return;
    }

    try {
        smoothness_threshold = std::stof(smoothness_input->GetValue().ToStdString());
    }
    catch (const std::exception&) {
        wxMessageBox("Invalid smoothness threshold. Please enter a valid number.", "Invalid Input", wxOK | wxICON_WARNING);
        return;
    }

    try {
        min_segment_size = std::stoul(segment_size_input->GetValue().ToStdString());
    }
    catch (const std::exception&) {
        wxMessageBox("Invalid minimum segment size. Please enter a valid number.", "Invalid Input", wxOK | wxICON_WARNING);
        return;
    }

    save_normal_bool = action_normal->GetValue();
    segmentation_bool = action_segmantation->GetValue();
    extract_features_bool = action_feature_extraction->GetValue();

    if (dataset.ends_with(".pcd")) {
        pcl_convert_bool = false;
    }
    else if (dataset.ends_with(".txt")) {
        pcl_convert_bool = true;
    }
    else {
        wxMessageBox("Please choose a .txt or .pcd file.", "Invalid File Type", wxOK | wxICON_INFORMATION);
        return;
    }

    use_single_cam_transform = single_camera_check->GetValue();
    use_supervoxel_refinement = refinement_check->GetValue();
    use_extended_convexity = convexty_check->GetValue();
    use_sanity_criterion = sanity_check->GetValue();

    features_need.clear();
    if (geometrical_check->GetValue()) features_need.push_back("Geometrical");
    if (statistical_check->GetValue()) features_need.push_back("Statistical");
    if (shape_check->GetValue()) features_need.push_back("Shape");
    if (density_check->GetValue()) features_need.push_back("Density");
    if (color_check->GetValue()) features_need.push_back("Color");
    if (area_check->GetValue()) features_need.push_back("Area");

    // Combine all variables into a single message
    std::stringstream msg;
    msg << "project_name: " << project_name << "\n";
    msg << "dataset: " << dataset << "\n";
    msg << "segment_type: " << segment_type << "\n";
    msg << "voxel_resolution: " << voxel_resolution << "\n";
    msg << "seed_resolution: " << seed_resolution << "\n";
    msg << "color_importance: " << color_importance << "\n";
    msg << "spatial_importance: " << spatial_importance << "\n";
    msg << "normal_importance: " << normal_importance << "\n";
    msg << "concavity_tolerance_threshold: " << concavity_tolerance_threshold << "\n";
    msg << "smoothness_threshold: " << smoothness_threshold << "\n";
    msg << "min_segment_size: " << min_segment_size << "\n";
    msg << "save_normal_bool: " << save_normal_bool << "\n";
    msg << "segmentation_bool: " << segmentation_bool << "\n";
    msg << "extract_features_bool: " << extract_features_bool << "\n";
    msg << "pcl_convert_bool: " << pcl_convert_bool << "\n";
    msg << "use_single_cam_transform: " << use_single_cam_transform << "\n";
    msg << "use_supervoxel_refinement: " << use_supervoxel_refinement << "\n";
    msg << "use_extended_convexity: " << use_extended_convexity << "\n";
    msg << "use_sanity_criterion: " << use_sanity_criterion << "\n";
    msg << "features_need: ";
    for (const auto& feature : features_need) {
        msg << feature << " ";
    }

    // Show the message box
    wxMessageBox(msg.str(), "Saved Options", wxOK | wxICON_INFORMATION);
}

void MainFrame::SaveProjectOptions() {
    SaveOptions();

    // Check if project_name is empty
    if (project_name.empty()) {
        return; // Do not save if project name is empty
    }

    std::string path = "Projects/" + project_name;
    if (!std::filesystem::exists(path)) {
        return; // Do not save if the folder does not exist
    }
    nlohmann::json jsonData;

    jsonData["project_name"] = project_name;
    jsonData["dataset"] = dataset;
    jsonData["segment_type"] = segment_type;
    jsonData["voxel_resolution"] = voxel_resolution;
    jsonData["seed_resolution"] = seed_resolution;
    jsonData["color_importance"] = color_importance;
    jsonData["spatial_importance"] = spatial_importance;
    jsonData["normal_importance"] = normal_importance;
    jsonData["concavity_tolerance_threshold"] = concavity_tolerance_threshold;
    jsonData["smoothness_threshold"] = smoothness_threshold;
    jsonData["min_segment_size"] = min_segment_size;
    jsonData["save_normal_bool"] = save_normal_bool;
    jsonData["segmentation_bool"] = segmentation_bool;
    jsonData["extract_features_bool"] = extract_features_bool;
    jsonData["pcl_convert_bool"] = pcl_convert_bool;
    jsonData["use_single_cam_transform"] = use_single_cam_transform;
    jsonData["use_supervoxel_refinement"] = use_supervoxel_refinement;
    jsonData["use_extended_convexity"] = use_extended_convexity;
    jsonData["use_sanity_criterion"] = use_sanity_criterion;

    for (const auto& feature : features_need) {
        jsonData["features_need"].push_back(feature);
    }

    std::string optionsPath = path + "/options.json";
    std::ofstream file(optionsPath);
    file << jsonData.dump(4); // Pretty print with 4 spaces
    file.close();
}

void MainFrame::ReadProjectOptions() {
    std::string path = "Projects/" + project_name + "/options.json";
    std::ifstream file(path);
    if (!file.is_open()) {
        return; // Silently pass if the file does not exist
    }

    nlohmann::json jsonData;
    file >> jsonData;
    file.close();

    project_name_input->SetValue(jsonData["project_name"].get<std::string>());
    dataset_input->SetPath(jsonData["dataset"].get<std::string>());
    segment_type = jsonData["segment_type"].get<std::string>();

    voxel_input->SetValue(std::to_string(jsonData["voxel_resolution"].get<float>()));
    seed_input->SetValue(std::to_string(jsonData["seed_resolution"].get<float>()));
    color_input->SetValue(std::to_string(jsonData["color_importance"].get<float>()));
    spatial_input->SetValue(std::to_string(jsonData["spatial_importance"].get<float>()));
    normal_input->SetValue(std::to_string(jsonData["normal_importance"].get<float>()));
    concavity_input->SetValue(std::to_string(jsonData["concavity_tolerance_threshold"].get<float>()));
    smoothness_input->SetValue(std::to_string(jsonData["smoothness_threshold"].get<float>()));
    segment_size_input->SetValue(std::to_string(jsonData["min_segment_size"].get<unsigned long>()));

    action_normal->SetValue(jsonData["save_normal_bool"].get<bool>());
    action_segmantation->SetValue(jsonData["segmentation_bool"].get<bool>());
    action_feature_extraction->SetValue(jsonData["extract_features_bool"].get<bool>());
    pcl_convert_bool = jsonData["pcl_convert_bool"].get<bool>();
    single_camera_check->SetValue(jsonData["use_single_cam_transform"].get<bool>());
    refinement_check->SetValue(jsonData["use_supervoxel_refinement"].get<bool>());
    convexty_check->SetValue(jsonData["use_extended_convexity"].get<bool>());
    sanity_check->SetValue(jsonData["use_sanity_criterion"].get<bool>());

    features_need.clear();
    for (const auto& feature : jsonData["features_need"]) {
        features_need.push_back(feature.get<std::string>());
    }

    geometrical_check->SetValue(std::find(features_need.begin(), features_need.end(), "Geometrical") != features_need.end());
    statistical_check->SetValue(std::find(features_need.begin(), features_need.end(), "Statistical") != features_need.end());
    shape_check->SetValue(std::find(features_need.begin(), features_need.end(), "Shape") != features_need.end());
    density_check->SetValue(std::find(features_need.begin(), features_need.end(), "Density") != features_need.end());
    color_check->SetValue(std::find(features_need.begin(), features_need.end(), "Color") != features_need.end());
    area_check->SetValue(std::find(features_need.begin(), features_need.end(), "Area") != features_need.end());
}
