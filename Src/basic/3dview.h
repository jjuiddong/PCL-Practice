//
// 2024-12-04, jjuiddong
// 3d view
//
#pragma once

#include "../commonlib/pointcloud.h"


class c3DView : public framework::cDockWindow
{
public:
	c3DView();
	virtual ~c3DView();

	bool Init(graphic::cRenderer &renderer);
	virtual void OnUpdate(const float deltaSeconds) override;
	virtual void OnRender(const float deltaSeconds) override;
	virtual void OnPreRender(const float deltaSeconds) override;
	virtual void OnResizeEnd(const framework::eDockResize::Enum type, const sRectf &rect) override;
	virtual void OnEventProc(const sf::Event &evt) override;
	virtual void OnResetDevice() override;
	void Clear();


protected:
	void RenderScene(graphic::cRenderer& renderer, const float deltaSeconds);
	void UpdateLookAt();
	void OnWheelMove(const float delta, const POINT mousePt);
	void OnMouseMove(const POINT mousePt);
	void OnMouseDown(const sf::Mouse::Button &button, const POINT mousePt);
	void OnMouseUp(const sf::Mouse::Button &button, const POINT mousePt);

	bool SearchPointCloud(const Vector3 &pos);


public:
	graphic::cRenderTarget m_renderTarget;
	graphic::cGridLine m_gridLine;
	graphic::cDbgAxis m_axis;

	// MouseMove Variable
	POINT m_viewPos;
	sRectf m_viewRect; // detect mouse event area
	POINT m_mousePos; // window 2d mouse pos
	POINT m_mouseClickPos; // window 2d mouse pos
	Vector3 m_mousePickPos; // mouse cursor pos in ground picking
	bool m_mouseDown[3]; // Left, Right, Middle
	float m_rotateLen;

	bool m_isRenderPc1;
	bool m_isRenderPc2;
	cPointCloud m_pc1;
	cPointCloud m_pc2;
};
