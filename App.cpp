#include "App.h"
#include "MainFrame.h"

wxIMPLEMENT_APP(App);

bool App::OnInit()
{
    MainFrame* mainFrame = new MainFrame(nullptr, wxID_ANY, "Point segCloud 1.2", wxDefaultPosition, wxSize(800, 450), wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);
    mainFrame->SetClientSize(800, 450);
    mainFrame->Center();
    mainFrame->Show();
    return true;
}
