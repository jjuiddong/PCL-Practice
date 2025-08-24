//
// 2024-12-04, jjuiddong
// point cloud library practice
//  - cloud viewer practice
//  - reference: https://github.com/LimHyungTae/pcl_tutorial
//
#include "stdafx.h"
#include "../commonlib/pointcloud.h"
#include "3dview.h"

using namespace graphic;
using namespace framework;
class cApp : public framework::cGameMain2
{
public:
	cApp();
	virtual ~cApp();
	virtual bool OnInit() override;
	virtual void OnEventProc(const sf::Event& evt) override;
};
INIT_FRAMEWORK3(cApp);


cApp::cApp()
{
	m_windowName = L"PCL Practice";
	m_isLazyMode = true;
	//const RECT r = { 0, 0, 1024, 768 };
	//const RECT r = { 0, 0, 1224, 768 };
	//const RECT r = { 0, 0, 1424, 768 };
	const RECT r = { 0, 0, 1280, 960 };
	m_windowRect = r;
	graphic::cResourceManager::Get()->SetMediaDirectory("./media/");
}

cApp::~cApp()
{
}


bool cApp::OnInit()
{
	c3DView* view = new c3DView();
	view->Create(eDockState::DOCKWINDOW, eDockSlot::TAB, this, NULL);
	view->Init(m_renderer);

	m_gui.SetContext();
	m_gui.SetStyleColorsDark();
	return true;
}


void cApp::OnEventProc(const sf::Event& evt)
{
	if (sf::Event::KeyPressed == evt.type)
		if (sf::Keyboard::Escape == evt.key.cmd)
			close();
}
