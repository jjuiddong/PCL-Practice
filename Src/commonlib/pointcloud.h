//
// 2024-11-28, jjuiddong
// point cloud 
//
#pragma once


class cPointCloud
{
public:
	cPointCloud();
	virtual ~cPointCloud();

	bool Init(graphic::cRenderer& renderer);
	bool Render(graphic::cRenderer& renderer, const XMMATRIX& parentTm = graphic::XMIdentity);
	bool RenderNormal(graphic::cRenderer& renderer, const XMMATRIX& parentTm = graphic::XMIdentity);
	bool ReadCaptureFile(const string& fileName);
	bool SetColor(const graphic::cColor& color);
	Vector3 GetCenter();
	bool IsEmpty();
	void Clear();


public:
	graphic::cCube m_cube; // instancing render
	graphic::cColor m_color; // default white	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_points;
	pcl::PointCloud<pcl::Normal>::Ptr m_norms;

	vector<Matrix44> m_mats; // instancing renderer matrix buffer
	vector<Vector3> m_colors; // instancing renderer color buffer
};

