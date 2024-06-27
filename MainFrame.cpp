/*
#include <wx/wx.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkRenderWindowInteractor.h>

class MyApp : public wxApp {
public:
    virtual bool OnInit();
};

class MyFrame : public wxFrame {
public:
    MyFrame(const wxString& title);

private:
    void OnButtonClicked(wxCommandEvent& event);
    wxPanel* panel;
    wxButton* button;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderer> renderer;
};

wxIMPLEMENT_APP(MyApp);

bool MyApp::OnInit() {
    MyFrame* frame = new MyFrame("PCD Viewer");
    frame->Show(true);
    return true;
}

MyFrame::MyFrame(const wxString& title) : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(800, 600)) {
    wxBoxSizer* sizer = new wxBoxSizer(wxVERTICAL);

    panel = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxSize(800, 500));
    sizer->Add(panel, 1, wxEXPAND);

    button = new wxButton(this, wxID_ANY, wxT("Load PCD"), wxDefaultPosition, wxSize(780, 30));
    sizer->Add(button, 0, wxALIGN_CENTER | wxTOP | wxBOTTOM, 10);

    this->SetSizer(sizer);

    button->Bind(wxEVT_BUTTON, &MyFrame::OnButtonClicked, this);

    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer->setBackgroundColor(0, 0, 0);

    renderWindow = viewer->getRenderWindow();
    renderWindow->SetParentId(panel->GetHandle());
    renderWindow->SetSize(panel->GetSize().GetWidth(), panel->GetSize().GetHeight());
}

void MyFrame::OnButtonClicked(wxCommandEvent& event) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("Tree.pcd", *cloud) == -1) {
        wxLogMessage("Couldn't read file Tree.pcd");
        return;
    }

    viewer->removeAllPointClouds();
    viewer->addPointCloud(cloud);
    viewer->resetCamera();
    renderWindow->Render();
}

int main(int argc, char** argv) {
    wxEntryStart(argc, argv);
    MyApp* app = new MyApp();
    wxApp::SetInstance(app);
    wxEntry(argc, argv);
    return 0;
}
*/