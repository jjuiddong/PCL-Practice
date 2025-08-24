
#include "stdafx.h"
#include "3dview.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

using namespace graphic;
using namespace framework;


c3DView::c3DView()
	: framework::cDockWindow("3DView")
	, m_isRenderPc1(true)
	, m_isRenderPc2(true)
{
}

c3DView::~c3DView()
{
	Clear();
}


// reference: https://limhyungtae.github.io/2021-09-13-ROS-Point-Cloud-Library-(PCL)-10.-Normal-Estimation/
template<typename KDTree>
void calc_normal(KDTree& kdtree,
	const Vector3 &center,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& src,
	pcl::PointCloud<pcl::Normal>::Ptr& cloud_normal
) 
{
	vector<int> idxes;
	vector<float> sqr_dists;
	idxes.reserve(100);
	sqr_dists.reserve(100);

	kdtree.setInputCloud(src);

	for (int i = 0; i < src->points.size(); ++i)
	{
		pcl::PointXYZRGB& query = src->points[i];
		kdtree.radiusSearch(query, 0.2f, idxes, sqr_dists);

		Vector3 p0;
		Vector3 dir;
		for (uint k = 0; k < (uint)idxes.size(); ++k)
		{
			const Vector3 p1 = *(Vector3*)&src->points[idxes[k]];
			if (0 != k)
				dir += (p1 - p0);
			p0 = p1;
		}
		dir.Normalize();
		Vector3 n = Vector3(0, 1, 0).CrossProduct(dir).Normal();
		const Vector3 pos = *(Vector3*)&query;
		const Vector3 v = (pos - center).Normal();
		if (n.DotProduct(v) > 0.f)
			n = -n;

		const pcl::Normal normal_tmp(n.x, n.y, n.z);
		cloud_normal->points.emplace_back(normal_tmp);

		idxes.clear();
		sqr_dists.clear();
	}
}


template<typename KDTree>
void detectWall(KDTree& kdtree,
	cPointCloud &pc1,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out
)
{
	vector<int> idxes;
	vector<float> sqr_dists;
	idxes.reserve(100);
	sqr_dists.reserve(100);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src = pc1.m_points;
	pcl::PointCloud<pcl::Normal>::Ptr norms = pc1.m_norms;

	//kdtree.setInputCloud(src);

	for (int i = 0; i < src->points.size(); ++i)
	{
		pcl::PointXYZRGB& query = src->points[i];
		kdtree.radiusSearch(query, 0.2f, idxes, sqr_dists);

		if (idxes.size() < 5)
		{
			idxes.clear();
			sqr_dists.clear();
			continue;
		}

		bool isWall = true;
		Vector3 dir;
		for (uint k = 0; k < (uint)idxes.size(); ++k)
		{
			const Vector3 n = *(Vector3*)&norms->points[idxes[k]];
			if (0 != k)
			{
				const float d = dir.DotProduct(n);
				if (abs(d) < 0.9f)
				{
					isWall = false;
					break;
				}
			}
			dir = n;
		}

		if (isWall)
			out->points.emplace_back(src->points[i]);

		idxes.clear();
		sqr_dists.clear();
	}
}


// initialize
bool c3DView::Init(graphic::cRenderer &renderer)
{
	const Vector3 eyePos(0, 5, -5);
	const Vector3 lookAt(0, 0, 0);
	m_camera.SetCamera(eyePos, lookAt, Vector3(0, 1, 0));
	m_camera.SetProjection(MATH_PI / 4.f, m_rect.Width() / m_rect.Height(), 0.1f, 100000.f);
	m_camera.SetViewPort(m_rect.Width(), m_rect.Height());

	GetMainLight().Init(graphic::cLight::LIGHT_DIRECTIONAL);
	GetMainLight().SetDirection(Vector3(-1, -2, -1.3f).Normal());

	sf::Vector2u size((uint)m_rect.Width() - 15, (uint)m_rect.Height() - 50);
	cViewport vp = renderer.m_viewPort;
	vp.m_vp.Width = (float)size.x;
	vp.m_vp.Height = (float)size.y;
	m_renderTarget.Create(renderer, vp, DXGI_FORMAT_R8G8B8A8_UNORM, true, true
		, DXGI_FORMAT_D24_UNORM_S8_UINT);

	m_gridLine.Create(renderer, 1000, 1000, 1.f, 1.f
		, (eVertexType::POSITION | eVertexType::COLOR));

	m_axis.Create(renderer);
	cBoundingBox bbox3(Vector3(0, 0, 0), Vector3(10, 0, 10), Quaternion());
	m_axis.SetAxis(0.025f, bbox3, false);

	m_pc1.Init(renderer);
	m_pc2.Init(renderer);
	m_pc1.ReadCaptureFile("lidar-capture-0.bin");

	// voxelization
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr src = m_pc1.m_points;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
	//float voxelsize = 0.1;
	//voxel_filter.setInputCloud(src);
	//voxel_filter.setLeafSize(voxelsize, voxelsize, voxelsize);
	//voxel_filter.filter(*ptr_filtered);
	//m_pc2.m_points = ptr_filtered;
	//m_pc2.SetColor(cColor::RED);

	// sor
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(m_pc1.m_points);
	sor.setMeanK(20);
	sor.setStddevMulThresh(0.5f);
	sor.filter(*output);
	m_pc1.m_points->clear();
	m_pc1.m_points->reserve(output->size());
	for (uint i = 0; i < (uint)output->size(); ++i)
	{
		auto& s = output->at(i);
		m_pc1.m_points->push_back(s);
	}
	//~

	// normal estimate
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
	cloud_normal->points.reserve(m_pc1.m_points->size());
	calc_normal(kdtree, m_pc1.GetCenter(), m_pc1.m_points, cloud_normal);
	m_pc1.m_norms = cloud_normal;
	//~

	// detect wall
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr wall(new pcl::PointCloud<pcl::PointXYZRGB>);
	detectWall(kdtree, m_pc1, wall);
	m_pc2.m_points = wall;


	return true;
}


void c3DView::OnUpdate(const float deltaSeconds)
{
}


void c3DView::OnRender(const float deltaSeconds)
{
	ImVec2 pos = ImGui::GetCursorScreenPos();
	m_viewPos = { (int)(pos.x), (int)(pos.y) };
	m_viewRect = { pos.x + 5, pos.y, pos.x + m_rect.Width() - 30, pos.y + m_rect.Height() - 42 };

	// HUD
	bool isOpen = true;
	ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration
		| ImGuiWindowFlags_NoBackground
		;

	ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0, 0, 0, 0));
	ImGui::Image(m_renderTarget.m_resolvedSRV, ImVec2(m_rect.Width() - 15, m_rect.Height() - 42));

	// Render Information
	ImGui::SetNextWindowPos(ImVec2(pos.x, pos.y));
	ImGui::SetNextWindowBgAlpha(0.f);
	ImGui::SetNextWindowSize(ImVec2(min(m_viewRect.Width(), 350.f), m_viewRect.Height()));
	if (ImGui::Begin("3DView", &isOpen, flags))
	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::Checkbox("PC1", &m_isRenderPc1);
		ImGui::Checkbox("PC2", &m_isRenderPc2);
	}
	ImGui::End();
	ImGui::PopStyleColor();
}


void c3DView::OnPreRender(const float deltaSeconds)
{
	cRenderer &renderer = GetRenderer();

	cAutoCam cam(&m_camera);

	renderer.UnbindShaderAll();
	renderer.UnbindTextureAll();
	GetMainCamera().Bind(renderer);
	GetMainLight().Bind(renderer);

	if (m_renderTarget.Begin(renderer))
	{
		cAutoCam cam(&m_camera);
		renderer.UnbindShaderAll();
		renderer.UnbindTextureAll();
		GetMainCamera().Bind(renderer);
		GetMainLight().Bind(renderer);
		renderer.GetDevContext()->RSSetState(renderer.m_renderState.CullCounterClockwise());
		m_gridLine.Render(renderer);
		m_axis.Render(renderer);
		RenderScene(renderer, deltaSeconds);
	}
	m_renderTarget.End(renderer);
	renderer.UnbindTextureAll();
}


// render shared data
void c3DView::RenderScene(graphic::cRenderer& renderer, const float deltaSeconds)
{
	if (m_isRenderPc1)
	{
		m_pc1.Render(renderer);
		m_pc1.RenderNormal(renderer);
	}
	if (m_isRenderPc2)
		m_pc2.Render(renderer);

	//renderer.m_dbgSphere.m_transform.pos = m_mousePickPos;
	//renderer.m_dbgSphere.m_transform.scale = Vector3::Ones * 0.2f;
	//renderer.m_dbgSphere.Render(renderer);
}


// search point
bool c3DView::SearchPointCloud(const Vector3& pos)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src = m_pc1.m_points;

	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	const int N = 2;
	const double radius = 0.2;
	std::vector<int> idxes;
	std::vector<float> sqr_dists;
	kdtree.setInputCloud(src);
	pcl::PointXYZRGB query(pos.x, pos.y, pos.z);
	kdtree.radiusSearch(query, radius, idxes, sqr_dists);
	//kdtree.nearestKSearch(query, N, idxes, sqr_dists);

	m_pc1.SetColor(cColor::WHITE); // clear
	for (const auto& idx : idxes)
		src->points[idx].rgb = *(float*)&cColor::RED.m_color;

	return true;
}


void c3DView::OnResizeEnd(const framework::eDockResize::Enum type, const sRectf &rect)
{
	if (type == eDockResize::DOCK_WINDOW)
		m_owner->RequestResetDeviceNextFrame();
}


void c3DView::UpdateLookAt()
{
	GetMainCamera().MoveCancel();

	const float centerX = GetMainCamera().m_width / 2;
	const float centerY = GetMainCamera().m_height / 2;
	const Ray ray = GetMainCamera().GetRay((int)centerX, (int)centerY);
	const Plane groundPlane(Vector3(0, 1, 0), 0);
	const float distance = groundPlane.Collision(ray.dir);
	if (distance < -0.2f)
	{
		GetMainCamera().m_lookAt = groundPlane.Pick(ray.orig, ray.dir);
	}
	else
	{ // horizontal viewing
		const Vector3 lookAt = GetMainCamera().m_eyePos + GetMainCamera().GetDirection() * 5.f;
		GetMainCamera().m_lookAt = lookAt;
	}

	GetMainCamera().UpdateViewMatrix();
}


void c3DView::OnWheelMove(const float delta, const POINT mousePt)
{
	UpdateLookAt();

	float len = 0;
	const Plane groundPlane(Vector3(0, 1, 0), 0);
	const Ray ray = GetMainCamera().GetRay(mousePt.x, mousePt.y);
	Vector3 lookAt = groundPlane.Pick(ray.orig, ray.dir);
	len = min(50.f, (ray.orig - lookAt).Length());

	const int lv = 10;
	const float zoomLen = min(len * 0.1f, (float)(2 << (16 - lv)));

	GetMainCamera().Zoom(ray.dir, (delta < 0) ? -zoomLen : zoomLen);
}


// Handling Mouse Move Event
void c3DView::OnMouseMove(const POINT mousePt)
{
	const POINT delta = { mousePt.x - m_mousePos.x, mousePt.y - m_mousePos.y };
	m_mousePos = mousePt;

	if (m_mouseDown[0])
	{
		Vector3 dir = GetMainCamera().GetDirection();
		Vector3 right = GetMainCamera().GetRight();
		dir.y = 0;
		dir.Normalize();
		right.y = 0;
		right.Normalize();

		GetMainCamera().MoveRight(-delta.x * m_rotateLen * 0.001f);
		GetMainCamera().MoveFrontHorizontal(delta.y * m_rotateLen * 0.001f);
	}
	else if (m_mouseDown[1])
	{
		const float scale = 0.002f;
		m_camera.Yaw2(delta.x * scale, Vector3(0, 1, 0));
		m_camera.Pitch2(delta.y * scale, Vector3(0, 1, 0));
	}
	else if (m_mouseDown[2])
	{
		const float len = GetMainCamera().GetDistance();
		GetMainCamera().MoveRight(-delta.x * len * 0.001f);
		GetMainCamera().MoveUp(delta.y * len * 0.001f);
	}
}


// Handling Mouse Button Down Event
void c3DView::OnMouseDown(const sf::Mouse::Button &button, const POINT mousePt)
{
	m_mousePos = mousePt;
	m_mouseClickPos = mousePt;
	UpdateLookAt();
	SetCapture();

	const Ray ray = GetMainCamera().GetRay(mousePt.x, mousePt.y);
	const Plane groundPlane(Vector3(0, 1, 0), 0);
	const Vector3 target = groundPlane.Pick(ray.orig, ray.dir);
	m_rotateLen = ray.orig.y * 0.9f;// (target - ray.orig).Length();
	m_mousePickPos = target;

	switch (button)
	{
	case sf::Mouse::Left:
	{
		m_mouseDown[0] = true;
		SearchPointCloud(target);
	}
	break;
	case sf::Mouse::Right:
		m_mouseDown[1] = true;
		break;
	case sf::Mouse::Middle:
		m_mouseDown[2] = true;
		break;
	}
}


void c3DView::OnMouseUp(const sf::Mouse::Button &button, const POINT mousePt)
{
	const POINT delta = { mousePt.x - m_mousePos.x, mousePt.y - m_mousePos.y };
	m_mousePos = mousePt;
	ReleaseCapture();


	switch (button)
	{
	case sf::Mouse::Left:
		m_mouseDown[0] = false;
		break;
	case sf::Mouse::Right:
	{
		m_mouseDown[1] = false;
		const int dx = m_mouseClickPos.x - mousePt.x;
		const int dy = m_mouseClickPos.y - mousePt.y;
		if (sqrt(dx*dx + dy * dy) > 10)
			break; // move long distance, do not show popup menu
	}
	break;
	case sf::Mouse::Middle:
		m_mouseDown[2] = false;
		break;
	}
}


void c3DView::OnEventProc(const sf::Event &evt)
{
	ImGuiIO& io = ImGui::GetIO();
	switch (evt.type)
	{
	case sf::Event::KeyPressed:
		break;

	case sf::Event::MouseMoved:
	{
		cAutoCam cam(&m_camera);

		POINT curPos;
		GetCursorPos(&curPos); // sf::event mouse position has noise so we use GetCursorPos() function
		ScreenToClient(m_owner->getSystemHandle(), &curPos);
		POINT pos = { curPos.x - m_viewPos.x, curPos.y - m_viewPos.y };
		OnMouseMove(pos);
	}
	break;

	case sf::Event::MouseButtonPressed:
	case sf::Event::MouseButtonReleased:
	{
		cAutoCam cam(&m_camera);

		POINT curPos;
		GetCursorPos(&curPos); // sf::event mouse position has noise so we use GetCursorPos() function
		ScreenToClient(m_owner->getSystemHandle(), &curPos);
		const POINT pos = { curPos.x - m_viewPos.x, curPos.y - m_viewPos.y };
		const sRectf viewRect = GetWindowSizeAvailible(true);

		if (sf::Event::MouseButtonPressed == evt.type)
		{
			if (viewRect.IsIn((float)pos.x, (float)pos.y))
				OnMouseDown(evt.mouseButton.button, pos);
		}
		else
		{
			if (viewRect.IsIn((float)pos.x, (float)pos.y)
				|| (this == GetCapture()))
				OnMouseUp(evt.mouseButton.button, pos);
		}
	}
	break;

	case sf::Event::MouseWheelScrolled:
	{
		cAutoCam cam(&m_camera);

		POINT curPos;
		GetCursorPos(&curPos); // sf::event mouse position has noise so we use GetCursorPos() function
		ScreenToClient(m_owner->getSystemHandle(), &curPos);
		const POINT pos = { curPos.x - m_viewPos.x, curPos.y - m_viewPos.y };
		OnWheelMove(evt.mouseWheelScroll.delta, pos);
	}
	break;
	}
}


void c3DView::OnResetDevice()
{
	cRenderer &renderer = GetRenderer();

	// update viewport
	sRectf viewRect = { 0, 0, m_rect.Width() - 15, m_rect.Height() - 50 };
	m_camera.SetViewPort(viewRect.Width(), viewRect.Height());

	cViewport vp = GetRenderer().m_viewPort;
	vp.m_vp.Width = viewRect.Width();
	vp.m_vp.Height = viewRect.Height();
	m_renderTarget.Create(renderer, vp, DXGI_FORMAT_R8G8B8A8_UNORM, true, true, DXGI_FORMAT_D24_UNORM_S8_UINT);
}


// clear
void c3DView::Clear()
{
}

