#include "MainFrame.h"

// Credits: this code was created with the help of wxFormBuilder

MainFrame::MainFrame(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style) : wxFrame(parent, id, title, pos, size, style)
{
	this->SetSizeHints(wxSize(800, 450));
	this->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));
	this->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));

	// Set the application icon
	wxIcon appIcon("app_icon.ico", wxBITMAP_TYPE_ICO);
	this->SetIcon(appIcon);

	menu_bar = new wxMenuBar(0);
	file_menu = new wxMenu();
	wxMenuItem* new_file_menu;
	new_file_menu = new wxMenuItem(file_menu, wxID_NEW_PROJECT, wxString(_("New")) + wxT('\t') + wxT(" "), _("Create a new project"), wxITEM_NORMAL);
#ifdef __WXMSW__
	new_file_menu->SetBitmaps(wxArtProvider::GetBitmap(wxASCII_STR(wxART_NEW_DIR), wxASCII_STR(wxART_MENU)));
#elif (defined( __WXGTK__ ) || defined( __WXOSX__ ))
	new_file_menu->SetBitmap(wxArtProvider::GetBitmap(wxASCII_STR(wxART_NEW_DIR), wxASCII_STR(wxART_MENU)));
#endif
	file_menu->Append(new_file_menu);

	wxMenuItem* open_file_menu;
	open_file_menu = new wxMenuItem(file_menu, wxOPEN_PROJECT, wxString(_("Open")) + wxT('\t') + wxT("Ctrl + O"), _("Open up an existing project"), wxITEM_NORMAL);
#ifdef __WXMSW__
	open_file_menu->SetBitmaps(wxArtProvider::GetBitmap(wxASCII_STR(wxART_FILE_OPEN), wxASCII_STR(wxART_MENU)));
#elif (defined( __WXGTK__ ) || defined( __WXOSX__ ))
	open_file_menu->SetBitmap(wxArtProvider::GetBitmap(wxASCII_STR(wxART_FILE_OPEN), wxASCII_STR(wxART_MENU)));
#endif
	file_menu->Append(open_file_menu);

	wxMenuItem* delete_file_menu;
	delete_file_menu = new wxMenuItem(file_menu, wxID_DELETE_PROJECT, wxString(_("Delete")), wxEmptyString, wxITEM_NORMAL);
#ifdef __WXMSW__
	delete_file_menu->SetBitmaps(wxArtProvider::GetBitmap(wxASCII_STR(wxART_DELETE), wxASCII_STR(wxART_MENU)));
#elif (defined( __WXGTK__ ) || defined( __WXOSX__ ))
	delete_file_menu->SetBitmap(wxArtProvider::GetBitmap(wxASCII_STR(wxART_DELETE), wxASCII_STR(wxART_MENU)));
#endif
	file_menu->Append(delete_file_menu);

	file_menu->AppendSeparator();

	wxMenuItem* exit_file_menu;
	exit_file_menu = new wxMenuItem(file_menu, wxID_EXIT_PROJECT, wxString(_("Exit")), wxEmptyString, wxITEM_NORMAL);
#ifdef __WXMSW__
	exit_file_menu->SetBitmaps(wxArtProvider::GetBitmap(wxASCII_STR(wxART_QUIT), wxASCII_STR(wxART_MENU)));
#elif (defined( __WXGTK__ ) || defined( __WXOSX__ ))
	exit_file_menu->SetBitmap(wxArtProvider::GetBitmap(wxASCII_STR(wxART_QUIT), wxASCII_STR(wxART_MENU)));
#endif
	file_menu->Append(exit_file_menu);

	menu_bar->Append(file_menu, _("File"));

	help_menu = new wxMenu();
	wxMenuItem* doc_help_menu;
	doc_help_menu = new wxMenuItem(help_menu, wxID_OPEN_DOCS, wxString(_("Docs")), wxEmptyString, wxITEM_NORMAL);
#ifdef __WXMSW__
	doc_help_menu->SetBitmaps(wxArtProvider::GetBitmap(wxASCII_STR(wxART_HELP_PAGE), wxASCII_STR(wxART_MENU)));
#elif (defined( __WXGTK__ ) || defined( __WXOSX__ ))
	doc_help_menu->SetBitmap(wxArtProvider::GetBitmap(wxASCII_STR(wxART_HELP_PAGE), wxASCII_STR(wxART_MENU)));
#endif
	help_menu->Append(doc_help_menu);

	wxMenuItem* report_help_menu;
	report_help_menu = new wxMenuItem(help_menu, wxID_REPORT, wxString(_("Report")), wxEmptyString, wxITEM_NORMAL);
#ifdef __WXMSW__
	report_help_menu->SetBitmaps(wxArtProvider::GetBitmap(wxASCII_STR(wxART_QUESTION), wxASCII_STR(wxART_MENU)));
#elif (defined( __WXGTK__ ) || defined( __WXOSX__ ))
	report_help_menu->SetBitmap(wxArtProvider::GetBitmap(wxASCII_STR(wxART_QUESTION), wxASCII_STR(wxART_MENU)));
#endif
	help_menu->Append(report_help_menu);

	menu_bar->Append(help_menu, _("Help"));

	this->SetMenuBar(menu_bar);

	wxBoxSizer* main_sizer;
	main_sizer = new wxBoxSizer(wxHORIZONTAL);

	wxBoxSizer* right_main_sizer;
	right_main_sizer = new wxBoxSizer(wxVERTICAL);

	wxBoxSizer* project_settings_sizer;
	project_settings_sizer = new wxBoxSizer(wxVERTICAL);

	project_settings_header = new wxStaticText(this, wxID_ANY, _("Project Settings"), wxDefaultPosition, wxDefaultSize, 0);
	project_settings_header->Wrap(0);
	project_settings_header->SetFont(wxFont(9, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Arial")));
	project_settings_header->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_INACTIVECAPTIONTEXT));
	project_settings_header->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));

	project_settings_sizer->Add(project_settings_header, 0, wxALL, 5);

	wxBoxSizer* project_settings_main;
	project_settings_main = new wxBoxSizer(wxHORIZONTAL);

	wxBoxSizer* project_settings_tags;
	project_settings_tags = new wxBoxSizer(wxVERTICAL);

	project_name_tag = new wxStaticText(this, wxID_ANY, _("Project Name"), wxDefaultPosition, wxDefaultSize, 0);
	project_name_tag->Wrap(-1);
	project_name_tag->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_INACTIVECAPTIONTEXT));

	project_settings_tags->Add(project_name_tag, 1, wxALL, 5);

	dataset_tag = new wxStaticText(this, wxID_ANY, _("LiDAR Data"), wxDefaultPosition, wxDefaultSize, 0);
	dataset_tag->Wrap(-1);
	dataset_tag->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_INACTIVECAPTIONTEXT));

	project_settings_tags->Add(dataset_tag, 1, wxALL, 5);

	file_type_tag = new wxStaticText(this, wxID_ANY, _("Segment File Type"), wxDefaultPosition, wxDefaultSize, 0);
	file_type_tag->Wrap(-1);
	file_type_tag->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_INACTIVECAPTIONTEXT));

	project_settings_tags->Add(file_type_tag, 1, wxALL, 5);


	project_settings_main->Add(project_settings_tags, 2, wxEXPAND, 5);

	wxBoxSizer* x1_slizer;
	x1_slizer = new wxBoxSizer(wxVERTICAL);

	x11 = new wxStaticText(this, wxID_ANY, _(":"), wxDefaultPosition, wxDefaultSize, 0);
	x11->Wrap(-1);
	x11->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	x1_slizer->Add(x11, 1, wxALL, 5);

	x12 = new wxStaticText(this, wxID_ANY, _(":"), wxDefaultPosition, wxDefaultSize, 0);
	x12->Wrap(-1);
	x12->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	x1_slizer->Add(x12, 1, wxALL, 5);

	x13 = new wxStaticText(this, wxID_ANY, _(":"), wxDefaultPosition, wxDefaultSize, 0);
	x13->Wrap(-1);
	x13->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	x1_slizer->Add(x13, 1, wxALL, 5);


	project_settings_main->Add(x1_slizer, 0, wxEXPAND, 5);

	wxBoxSizer* project_input_sizer;
	project_input_sizer = new wxBoxSizer(wxVERTICAL);

	project_name_input = new wxTextCtrl(this, wxID_PROJECT_NAME, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
	project_name_input->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	project_input_sizer->Add(project_name_input, 1, wxALL | wxEXPAND, 5);

	dataset_input = new wxFilePickerCtrl(this, wxID_DATA_LOC, wxEmptyString, _("Select a file"), _("*.*"), wxDefaultPosition, wxDefaultSize, wxFLP_DEFAULT_STYLE);
	dataset_input->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	project_input_sizer->Add(dataset_input, 1, wxALL | wxEXPAND, 5);

	wxString segmant_type_inputChoices[] = { _(".pcd (Point Cloud Data)"), _(".txt (Text file) Feature Extraction not supported curently!") };
	int segmant_type_inputNChoices = sizeof(segmant_type_inputChoices) / sizeof(wxString);
	segmant_type_input = new wxChoice(this, wxID_SEGMENT_TYPE, wxDefaultPosition, wxDefaultSize, segmant_type_inputNChoices, segmant_type_inputChoices, 0);
	segmant_type_input->SetSelection(0);
	segmant_type_input->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	project_input_sizer->Add(segmant_type_input, 1, wxALL | wxEXPAND, 5);


	project_settings_main->Add(project_input_sizer, 5, wxEXPAND, 5);


	project_settings_sizer->Add(project_settings_main, 0, wxEXPAND, 5);

	process_header = new wxStaticText(this, wxID_ANY, _("Process"), wxDefaultPosition, wxDefaultSize, 0);
	process_header->Wrap(-1);
	process_header->SetFont(wxFont(9, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Arial")));
	process_header->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	project_settings_sizer->Add(process_header, 0, wxALL, 5);

	wxBoxSizer* actions_sizer;
	actions_sizer = new wxBoxSizer(wxHORIZONTAL);

	action_normal = new wxCheckBox(this, wxID_NORMAL_BOOL, _("Find Normal"), wxDefaultPosition, wxDefaultSize, 0);
	action_normal->SetValue(true);
	action_normal->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));
	action_normal->SetToolTip(_("Finds normal for the data.\nIf you found them already,\nuntick the checkbox."));

	actions_sizer->Add(action_normal, 1, wxALL, 5);

	action_segmantation = new wxCheckBox(this, wxID_SEGMENT_BOOL, _("Segmentation"), wxDefaultPosition, wxDefaultSize, 0);
	action_segmantation->SetValue(true);
	action_segmantation->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));
	action_segmantation->SetToolTip(_("Segmant the data file\ninto small files, untick\nif you already did it."));

	actions_sizer->Add(action_segmantation, 1, wxALL, 5);

	action_feature_extraction = new wxCheckBox(this, wxID_FEATURE_BOOL, _("Feature Extraction"), wxDefaultPosition, wxDefaultSize, 0);
	action_feature_extraction->SetValue(true);
	action_feature_extraction->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));
	action_feature_extraction->SetToolTip(_("Extract features from segmants,\nand store them in feature.csv,\nuntick if you don't need to\nextract features."));

	actions_sizer->Add(action_feature_extraction, 1, wxALL, 5);


	project_settings_sizer->Add(actions_sizer, 1, wxALL | wxEXPAND, 5);

	wxBoxSizer* dir_slizer;
	dir_slizer = new wxBoxSizer(wxHORIZONTAL);

	view_project_button = new wxButton(this, wxID_VIEW_FILES, _("View Project Files"), wxDefaultPosition, wxSize(-1, 30), 0);
	view_project_button->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNTEXT));

	dir_slizer->Add(view_project_button, 1, wxALL, 5);

	view_segmant_button = new wxButton(this, wxID_VIEW_SEGMENT, _("View Segmantation"), wxDefaultPosition, wxSize(-1, 30), 0);
	view_segmant_button->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNTEXT));

	dir_slizer->Add(view_segmant_button, 1, wxALL, 5);

	open_feature_button = new wxButton(this, wxID_OPEN_FEATURE, _("Open Features"), wxDefaultPosition, wxSize(-1, 30), 0);
	open_feature_button->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNTEXT));

	dir_slizer->Add(open_feature_button, 1, wxALL, 5);


	project_settings_sizer->Add(dir_slizer, 0, wxEXPAND, 5);


	right_main_sizer->Add(project_settings_sizer, 5, wxEXPAND, 5);

	wxBoxSizer* progress_slizer;
	progress_slizer = new wxBoxSizer(wxVERTICAL);

	log_output = new wxStaticText(this, wxID_LOG_TEXT, _("Click the Start button to stop processing"), wxDefaultPosition, wxDefaultSize, 0);
	log_output->Wrap(-1);
	log_output->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_INACTIVECAPTIONTEXT));

	progress_slizer->Add(log_output, 0, wxALL, 5);

	wxBoxSizer* gauge_slizer;
	gauge_slizer = new wxBoxSizer(wxHORIZONTAL);

	gauge = new wxGauge(this, wxID_PROGRESS_BAR, 100, wxDefaultPosition, wxSize(-1, -1), wxGA_HORIZONTAL | wxGA_SMOOTH);
	gauge->SetValue(0);

	gauge_slizer->Add(gauge, 1, wxALL, 5);


	progress_slizer->Add(gauge_slizer, 0, wxEXPAND, 5);

	wxBoxSizer* start_btn_slizer;
	start_btn_slizer = new wxBoxSizer(wxHORIZONTAL);


	start_btn_slizer->Add(0, 0, 1, wxEXPAND, 5);

	start_button = new wxButton(this, wxSTART_BUTTON, _("Start Processing"), wxDefaultPosition, wxSize(200, 40), 0);
	start_button->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));
	start_button->SetMinSize(wxSize(200, 40));
	start_button->SetMaxSize(wxSize(220, 50));

	start_btn_slizer->Add(start_button, 0, wxALL, 5);


	progress_slizer->Add(start_btn_slizer, 1, wxEXPAND, 5);


	right_main_sizer->Add(progress_slizer, 1, wxEXPAND, 5);


	main_sizer->Add(right_main_sizer, 4, wxEXPAND, 5);

	wxBoxSizer* right_sizer;
	right_sizer = new wxBoxSizer(wxVERTICAL);

	wxBoxSizer* supervoxel_sizer;
	supervoxel_sizer = new wxBoxSizer(wxVERTICAL);

	parameter_options_header = new wxStaticText(this, wxID_ANY, _("Supervoxel Options"), wxDefaultPosition, wxDefaultSize, 0);
	parameter_options_header->Wrap(-1);
	parameter_options_header->SetFont(wxFont(9, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Arial")));
	parameter_options_header->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	supervoxel_sizer->Add(parameter_options_header, 0, wxALL, 5);

	resolution_tag = new wxStaticText(this, wxID_ANY, _("Resolution"), wxDefaultPosition, wxDefaultSize, 0);
	resolution_tag->Wrap(-1);
	resolution_tag->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	supervoxel_sizer->Add(resolution_tag, 0, wxLEFT | wxTOP, 5);

	wxBoxSizer* resolution_sizer;
	resolution_sizer = new wxBoxSizer(wxHORIZONTAL);

	voxel_tag = new wxStaticText(this, wxID_ANY, _("Voxel"), wxDefaultPosition, wxDefaultSize, 0);
	voxel_tag->Wrap(-1);
	voxel_tag->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	resolution_sizer->Add(voxel_tag, 0, wxALL, 5);

	voxel_input = new wxTextCtrl(this, wxID_VOXEL, _("0.7"), wxDefaultPosition, wxSize(50, -1), 0);
	voxel_input->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	resolution_sizer->Add(voxel_input, 0, wxALL | wxEXPAND, 5);

	seed_tag = new wxStaticText(this, wxID_ANY, _("   Seed"), wxDefaultPosition, wxDefaultSize, 0);
	seed_tag->Wrap(-1);
	seed_tag->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	resolution_sizer->Add(seed_tag, 0, wxALL, 5);

	seed_input = new wxTextCtrl(this, wxID_SEED, _("3.0"), wxDefaultPosition, wxSize(50, -1), 0);
	seed_input->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	resolution_sizer->Add(seed_input, 0, wxALL | wxEXPAND, 5);


	supervoxel_sizer->Add(resolution_sizer, 1, wxEXPAND, 5);

	importance_tag = new wxStaticText(this, wxID_ANY, _("Importance"), wxDefaultPosition, wxDefaultSize, 0);
	importance_tag->Wrap(-1);
	importance_tag->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	supervoxel_sizer->Add(importance_tag, 0, wxLEFT | wxTOP, 5);

	wxBoxSizer* importance_sizer;
	importance_sizer = new wxBoxSizer(wxHORIZONTAL);

	color_tag = new wxStaticText(this, wxID_ANY, _("Color"), wxDefaultPosition, wxDefaultSize, 0);
	color_tag->Wrap(-1);
	color_tag->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	importance_sizer->Add(color_tag, 0, wxALL, 5);

	color_input = new wxTextCtrl(this, wxID_COLOR, _("0.0"), wxDefaultPosition, wxSize(50, -1), 0);
	color_input->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	importance_sizer->Add(color_input, 0, wxALL, 5);

	spatial_tag = new wxStaticText(this, wxID_ANY, _("Spatial"), wxDefaultPosition, wxDefaultSize, 0);
	spatial_tag->Wrap(-1);
	spatial_tag->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	importance_sizer->Add(spatial_tag, 0, wxALL, 5);

	spatial_input = new wxTextCtrl(this, wxID_SPATIAL, _("10.0"), wxDefaultPosition, wxSize(50, -1), 0);
	spatial_input->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	importance_sizer->Add(spatial_input, 0, wxALL, 5);

	normal_tag = new wxStaticText(this, wxID_ANY, _("Normal"), wxDefaultPosition, wxDefaultSize, 0);
	normal_tag->Wrap(-1);
	normal_tag->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	importance_sizer->Add(normal_tag, 0, wxALL, 5);

	normal_input = new wxTextCtrl(this, wxID_NORMAL, _("10.0"), wxDefaultPosition, wxSize(50, -1), 0);
	normal_input->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	importance_sizer->Add(normal_input, 0, wxALL, 5);


	supervoxel_sizer->Add(importance_sizer, 1, wxEXPAND, 5);

	wxBoxSizer* supervoxel_checkbox_sizer;
	supervoxel_checkbox_sizer = new wxBoxSizer(wxHORIZONTAL);

	refinement_check = new wxCheckBox(this, wxID_REFINEMENT_BOOL, _("Supervoxel Refinement"), wxDefaultPosition, wxDefaultSize, 0);
	refinement_check->SetValue(true);
	refinement_check->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	supervoxel_checkbox_sizer->Add(refinement_check, 1, wxBOTTOM | wxLEFT, 5);


	supervoxel_sizer->Add(supervoxel_checkbox_sizer, 0, wxEXPAND, 5);


	right_sizer->Add(supervoxel_sizer, 0, wxEXPAND, 5);

	wxBoxSizer* lccp_sizer;
	lccp_sizer = new wxBoxSizer(wxVERTICAL);

	lccp_header = new wxStaticText(this, wxID_ANY, _("LCCP Segmentation Options"), wxDefaultPosition, wxDefaultSize, 0);
	lccp_header->Wrap(-1);
	lccp_header->SetFont(wxFont(9, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Arial")));
	lccp_header->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	lccp_sizer->Add(lccp_header, 0, wxALL, 5);

	wxBoxSizer* lccp_main_sizer;
	lccp_main_sizer = new wxBoxSizer(wxHORIZONTAL);

	wxBoxSizer* lccp_values_tag_sizer;
	lccp_values_tag_sizer = new wxBoxSizer(wxVERTICAL);

	concavity_tag = new wxStaticText(this, wxID_ANY, _("Concavity Tolerance Threshold"), wxDefaultPosition, wxDefaultSize, 0);
	concavity_tag->Wrap(-1);
	concavity_tag->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	lccp_values_tag_sizer->Add(concavity_tag, 1, wxALL, 5);

	smoothness_tag = new wxStaticText(this, wxID_ANY, _("Smoothness Threshold"), wxDefaultPosition, wxDefaultSize, 0);
	smoothness_tag->Wrap(-1);
	smoothness_tag->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	lccp_values_tag_sizer->Add(smoothness_tag, 1, wxALL, 5);

	segmant_size_tag = new wxStaticText(this, wxID_ANY, _("Minimum Segment Size"), wxDefaultPosition, wxDefaultSize, 0);
	segmant_size_tag->Wrap(-1);
	segmant_size_tag->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	lccp_values_tag_sizer->Add(segmant_size_tag, 1, wxALL, 5);


	lccp_main_sizer->Add(lccp_values_tag_sizer, 1, wxEXPAND, 5);

	wxBoxSizer* x2_sizer;
	x2_sizer = new wxBoxSizer(wxVERTICAL);

	x21 = new wxStaticText(this, wxID_ANY, _(":"), wxDefaultPosition, wxDefaultSize, 0);
	x21->Wrap(-1);
	x21->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	x2_sizer->Add(x21, 1, wxALL, 5);

	x22 = new wxStaticText(this, wxID_ANY, _(":"), wxDefaultPosition, wxDefaultSize, 0);
	x22->Wrap(-1);
	x22->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	x2_sizer->Add(x22, 1, wxALL, 5);

	x23 = new wxStaticText(this, wxID_ANY, _(":"), wxDefaultPosition, wxDefaultSize, 0);
	x23->Wrap(-1);
	x23->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	x2_sizer->Add(x23, 1, wxALL, 5);


	lccp_main_sizer->Add(x2_sizer, 0, wxEXPAND, 5);

	wxBoxSizer* lccp_values_sizer;
	lccp_values_sizer = new wxBoxSizer(wxVERTICAL);

	concavity_input = new wxTextCtrl(this, wxID_CONCAVITY, _("0.7"), wxDefaultPosition, wxDefaultSize, 0);
	concavity_input->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	lccp_values_sizer->Add(concavity_input, 0, wxALL, 5);

	smoothness_input = new wxTextCtrl(this, wxID_SMOOTHNESS, _("7"), wxDefaultPosition, wxDefaultSize, 0);
	smoothness_input->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	lccp_values_sizer->Add(smoothness_input, 0, wxALL, 5);

	segment_size_input = new wxTextCtrl(this, wxID_SEGMENT_SIZE, _("4"), wxDefaultPosition, wxDefaultSize, 0);
	segment_size_input->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	lccp_values_sizer->Add(segment_size_input, 0, wxALL, 5);


	lccp_main_sizer->Add(lccp_values_sizer, 1, wxEXPAND, 5);


	lccp_sizer->Add(lccp_main_sizer, 1, wxEXPAND, 5);

	wxBoxSizer* lccp_checkbox_sizer;
	lccp_checkbox_sizer = new wxBoxSizer(wxHORIZONTAL);

	convexty_check = new wxCheckBox(this, wxID_CONVEXITY, _("Extended Convexity"), wxDefaultPosition, wxDefaultSize, 0);
	convexty_check->SetValue(true);
	convexty_check->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	lccp_checkbox_sizer->Add(convexty_check, 1, wxBOTTOM | wxLEFT, 5);

	sanity_check = new wxCheckBox(this, wxID_SANITY, _("Sanity Criterion"), wxDefaultPosition, wxDefaultSize, 0);
	sanity_check->SetValue(true);
	sanity_check->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	lccp_checkbox_sizer->Add(sanity_check, 1, wxBOTTOM | wxLEFT, 5);


	lccp_sizer->Add(lccp_checkbox_sizer, 0, wxEXPAND, 5);


	right_sizer->Add(lccp_sizer, 0, wxEXPAND, 5);

	wxBoxSizer* extract_sizer;
	extract_sizer = new wxBoxSizer(wxVERTICAL);

	extract_header = new wxStaticText(this, wxID_ANY, _("Features To Extract"), wxDefaultPosition, wxDefaultSize, 0);
	extract_header->Wrap(-1);
	extract_header->SetFont(wxFont(9, wxFONTFAMILY_SWISS, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD, false, wxT("Arial")));
	extract_header->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));

	extract_sizer->Add(extract_header, 0, wxALL, 5);

	wxGridSizer* feature_need_sizer;
	feature_need_sizer = new wxGridSizer(2, 3, 0, 0);

	geometrical_check = new wxCheckBox(this, wxID_F_GEO, _("Geometrical"), wxDefaultPosition, wxDefaultSize, 0);
	geometrical_check->SetValue(true);
	geometrical_check->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));
	geometrical_check->SetToolTip(_("Features: Noofpoints, Xmax, Xmin, Ymax, Ymin, Zmax, Zmin, Zrange"));

	feature_need_sizer->Add(geometrical_check, 0, wxALL, 4);

	statistical_check = new wxCheckBox(this, wxID_F_STAT, _("Statistical"), wxDefaultPosition, wxDefaultSize, 0);
	statistical_check->SetValue(true);
	statistical_check->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));
	statistical_check->SetToolTip(_("Features: meanabsht"));

	feature_need_sizer->Add(statistical_check, 0, wxALL, 4);

	shape_check = new wxCheckBox(this, wxID_F_SHAPE, _("Shape"), wxDefaultPosition, wxDefaultSize, 0);
	shape_check->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));
	shape_check->SetToolTip(_("Features: ptness, curveness, surfaceness, linearity, planarity, volumetric"));

	feature_need_sizer->Add(shape_check, 0, wxALL, 4);

	density_check = new wxCheckBox(this, wxID_F_DENSITY, _("Density"), wxDefaultPosition, wxDefaultSize, 0);
	density_check->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));
	density_check->SetToolTip(_("Features: Density, HeightVariance"));

	feature_need_sizer->Add(density_check, 0, wxALL, 4);

	color_check = new wxCheckBox(this, wxID_F_COLOR, _("Color"), wxDefaultPosition, wxDefaultSize, 0);
	color_check->SetValue(true);
	color_check->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));
	color_check->SetToolTip(_("Features: Rmax, Rmin, Gmax, Gmin, Bmax, Bmin, Rmean, Gmean, Bmean, NDVI"));

	feature_need_sizer->Add(color_check, 0, wxALL, 4);

	area_check = new wxCheckBox(this, wxID_F_AREA, _("Area"), wxDefaultPosition, wxDefaultSize, 0);
	area_check->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));
	area_check->SetToolTip(_("Features: Aconv, Apoly, Bpoly"));

	feature_need_sizer->Add(area_check, 0, wxALL, 4);


	extract_sizer->Add(feature_need_sizer, 0, wxEXPAND, 5);


	right_sizer->Add(extract_sizer, 0, wxEXPAND, 5);


	main_sizer->Add(right_sizer, 2, wxEXPAND, 5);


	this->SetSizer(main_sizer);
	this->Layout();
	status_bar = this->CreateStatusBar(1, wxSTB_SIZEGRIP, wxID_STATUS_BAR);
	status_bar->SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BACKGROUND));


	this->Centre(wxBOTH);

	// Connect Events
	this->Connect(wxEVT_CLOSE_WINDOW, wxCloseEventHandler(MainFrame::OnCloseEvt));
	this->Connect(wxEVT_DROP_FILES, wxDropFilesEventHandler(MainFrame::OnFileDropEvt));
	file_menu->Bind(wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrame::OnMenuNew), this, new_file_menu->GetId());
	file_menu->Bind(wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrame::OnMenuOpen), this, open_file_menu->GetId());
	file_menu->Bind(wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrame::OnMenuDelete), this, delete_file_menu->GetId());
	file_menu->Bind(wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrame::OnMenuExit), this, exit_file_menu->GetId());
	help_menu->Bind(wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrame::OnHelpDocs), this, doc_help_menu->GetId());
	help_menu->Bind(wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MainFrame::OnHelpReport), this, report_help_menu->GetId());
	view_project_button->Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(MainFrame::ViewProjectEvt), NULL, this);
	view_segmant_button->Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(MainFrame::ViewSegmentationEvt), NULL, this);
	open_feature_button->Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(MainFrame::OpenFeaturesEvt), NULL, this);
	start_button->Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler(MainFrame::StartProcessEvt), NULL, this);

	// Bind Ctrl + O for OnMenuOpen
	Bind(wxEVT_MENU, &MainFrame::OnMenuOpen, this, wxID_OPEN);
	wxAcceleratorEntry accelOpen(wxACCEL_CTRL, (int)'O', wxID_OPEN);
	wxAcceleratorTable accelTableOpen(1, &accelOpen);
	SetAcceleratorTable(accelTableOpen);

	ReadProjectOptions(); // load options.json
}

MainFrame::~MainFrame()
{
}
