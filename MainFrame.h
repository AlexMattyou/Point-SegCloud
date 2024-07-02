#pragma once

#ifndef MAINFRAME_H
#define MAINFRAME_H

#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <string>
#include <map>
#include <ctime>
#include <fstream>
#include <vector>
#include <filesystem>
#include <sstream>
#include <limits>
#include <regex>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <thread>

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

#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <Eigen/Dense>

// wx widgets
#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
#include <wx/intl.h>
#include <wx/string.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/menu.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/stattext.h>
#include <wx/sizer.h>
#include <wx/textctrl.h>
#include <wx/filepicker.h>
#include <wx/choice.h>
#include <wx/checkbox.h>
#include <wx/button.h>
#include <wx/gauge.h>
#include <wx/statusbr.h>
#include <wx/frame.h>
#include <wx/msgdlg.h> // Include wxWidgets message box header for dialogs
#include <wx/listbox.h> 
#include <wx/scrolwin.h>
#include <wx/dir.h>
#include <wx/webview.h>
#include <wx/event.h>   // Include for event handling
#include <wx/wx.h>
#include <wx/validate.h>
#include <wx/valtext.h>

class MainFrame : public wxFrame
{
private:

protected:
	enum
	{
		wxID_MAIN_FRAME = 6000,
		wxID_NEW_PROJECT,
		wxOPEN_PROJECT,
		wxID_DELETE_PROJECT,
		wxID_EXIT_PROJECT,
		wxID_OPEN_DOCS,
		wxID_REPORT,
		wxID_PROJECT_NAME,
		wxID_DATA_LOC,
		wxID_SEGMENT_TYPE,
		wxID_NORMAL_BOOL,
		wxID_SEGMENT_BOOL,
		wxID_FEATURE_BOOL,
		wxID_VIEW_FILES,
		wxID_VIEW_SEGMENT,
		wxID_OPEN_FEATURE,
		wxID_LOG_TEXT,
		wxID_PROGRESS_BAR,
		wxSTART_BUTTON,
		wxID_VOXEL,
		wxID_SEED,
		wxID_COLOR,
		wxID_SPATIAL,
		wxID_NORMAL,
		wxID_SINGLE_CAMERA_BOOL,
		wxID_REFINEMENT_BOOL,
		wxID_CONCAVITY,
		wxID_SMOOTHNESS,
		wxID_SEGMENT_SIZE,
		wxID_CONVEXITY,
		wxID_SANITY,
		wxID_F_GEO,
		wxID_F_STAT,
		wxID_F_SHAPE,
		wxID_F_DENSITY,
		wxID_F_COLOR,
		wxID_F_AREA,
		wxID_STATUS_BAR,
		wxID_OPEN,
		wxID_NEW
	};

	wxMenuBar* menu_bar;
	wxMenu* file_menu;
	wxMenu* help_menu;
	wxStaticText* project_settings_header;
	wxStaticText* project_name_tag;
	wxStaticText* dataset_tag;
	wxStaticText* file_type_tag;
	wxStaticText* x11;
	wxStaticText* x12;
	wxStaticText* x13;
	wxTextCtrl* project_name_input;
	wxFilePickerCtrl* dataset_input;
	wxChoice* segmant_type_input;
	wxStaticText* process_header;
	wxCheckBox* action_normal;
	wxCheckBox* action_segmantation;
	wxCheckBox* action_feature_extraction;
	wxButton* view_project_button;
	wxButton* view_segmant_button;
	wxButton* open_feature_button;
	wxStaticText* log_output;
	wxButton* start_button;
	wxStaticText* parameter_options_header;
	wxStaticText* resolution_tag;
	wxStaticText* voxel_tag;
	wxTextCtrl* voxel_input;
	wxStaticText* seed_tag;
	wxTextCtrl* seed_input;
	wxStaticText* importance_tag;
	wxStaticText* color_tag;
	wxTextCtrl* color_input;
	wxStaticText* spatial_tag;
	wxTextCtrl* spatial_input;
	wxStaticText* normal_tag;
	wxTextCtrl* normal_input;
	wxCheckBox* single_camera_check;
	wxCheckBox* refinement_check;
	wxStaticText* lccp_header;
	wxStaticText* concavity_tag;
	wxStaticText* smoothness_tag;
	wxStaticText* segmant_size_tag;
	wxStaticText* x21;
	wxStaticText* x22;
	wxStaticText* x23;
	wxTextCtrl* concavity_input;
	wxTextCtrl* smoothness_input;
	wxTextCtrl* segment_size_input;
	wxCheckBox* convexty_check;
	wxCheckBox* sanity_check;
	wxStaticText* extract_header;
	wxCheckBox* geometrical_check;
	wxCheckBox* statistical_check;
	wxCheckBox* shape_check;
	wxCheckBox* density_check;
	wxCheckBox* color_check;
	wxCheckBox* area_check;

	// event handlers:
	void OnCloseEvt(wxCloseEvent& event);
	void OnFileDropEvt(wxDropFilesEvent& event);
	void OnMenuNew(wxCommandEvent& event);
	void OnMenuOpen(wxCommandEvent& event);
	void OnMenuDelete(wxCommandEvent& event);
	void OnMenuExit(wxCommandEvent& event);
	void OnHelpDocs(wxCommandEvent& event);
	void OnHelpReport(wxCommandEvent& event);
	void ViewProjectEvt(wxMouseEvent& event);
	void ViewSegmentationEvt(wxMouseEvent& event);
	void OpenFeaturesEvt(wxMouseEvent& event);
	void StartProcessEvt(wxMouseEvent& event);

	// processing functions:
	void OnProjectSelection(wxCommandEvent& event);
	void OpenInFileManager(const wxString& path);
	void ConfirmClose();
	void SaveOptions(bool exit);
	void SaveProjectOptions(bool exit);
	void ReadProjectOptions();


public:

	wxStatusBar* status_bar;
	wxGauge* gauge;

	MainFrame(wxWindow* parent, wxWindowID id = wxID_MAIN_FRAME, const wxString& title = _("Point Cloud Segmenter 1.0"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize(800, 450), long style = wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);

	~MainFrame();

	// Variables to store values
	std::string project_name;
	std::string dataset;
	std::string segment_type;
	double voxel_resolution;
	double seed_resolution;
	double color_importance;
	double spatial_importance;
	double normal_importance;
	double concavity_tolerance_threshold;
	double smoothness_threshold;
	int min_segment_size;

	bool use_single_cam_transform;
	bool use_supervoxel_refinement;
	bool use_extended_convexity;
	bool use_sanity_criterion;

	std::vector<std::string> features_need;

	// some public functions:
	void ShowStatus(const std::string& text);
	void ProcessThreadFunction();

	double scale;
	double score;

	bool pcl_convert_bool;
	bool save_normal_bool;
	bool segmentation_bool;
	bool extract_features_bool;

	// Point Cloud -----------------------------------------------------------------------------------


};

#endif // MAINFRAME_H

