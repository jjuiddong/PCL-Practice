
#include "stdafx.h"
#include "pointcloud.h"

using namespace graphic;


cPointCloud::cPointCloud()
	: m_color(1.f,1.f,1.f)
	, m_points(new pcl::PointCloud<pcl::PointXYZRGB>)
{
}

cPointCloud::~cPointCloud()
{
	Clear();
}


// initialize
bool cPointCloud::Init(graphic::cRenderer& renderer)
{
	const cBoundingBox bbox(Vector3(), Vector3::Ones, Quaternion());
	m_cube.Create(renderer, bbox, (eVertexType::POSITION | eVertexType::COLOR));

	return true;
}


// render
bool cPointCloud::Render(graphic::cRenderer& renderer
	, const XMMATRIX& parentTm //= XMIdentity
)
{
	RETV(!m_points, true);
	if (m_points->empty())
		return false; // nothing to do
	if (m_mats.size() != 256)
		m_mats.resize(256);
	if (m_colors.size() != 256)
		m_colors.resize(256);

	m_cube.m_color = m_color;
	m_cube.m_transform.scale = Vector3::Ones;
	m_cube.m_transform.pos = Vector3(0, 0, 0);
	m_cube.m_transform.rot = Quaternion();

	Matrix44 mat;
	mat.SetScale(Vector3::Ones * 0.005f);

	uint i = 0;
	while (i < (uint)m_points->size())
	{
		const uint count = min((uint)256, (uint)m_points->size() - i);
		for (uint k = 0; k < count; ++k)
		{
			mat.SetPosition(Vector3(m_points->points[k + i].x, 0.f, m_points->points[k + i].z));
			m_mats[k] = mat;

			const uint rgb = *(uint*)&m_points->points[k + i].rgb;
			const BYTE r = (rgb >> 16);
			const BYTE g = (rgb >> 8) & 0xFFF;
			const BYTE b = rgb & 0xFF;
			const Vector3 color(r / 255.f, g / 255.f, b / 255.f);
			m_colors[k] = color;
		}
		m_cube.RenderInstancing2(renderer, (int)count, &m_mats[0], &m_colors[0], parentTm);
		i += 256;
	}
	return true;
}


// render normal
bool cPointCloud::RenderNormal(graphic::cRenderer& renderer
	, const XMMATRIX& parentTm //= graphic::XMIdentity
)
{
	RETV(!m_points, true);
	if (m_points->empty() || m_norms->empty())
		return false; // nothing to do
	if (m_points->size() != m_norms->size())
		return false; // error return
	if (m_mats.size() != 256)
		m_mats.resize(256);

	m_cube.m_color = m_color;
	m_cube.m_transform.scale = Vector3::Ones;
	m_cube.m_transform.pos = Vector3(0, 0, 0);
	m_cube.m_transform.rot = Quaternion();

	Matrix44 mat;
	mat.SetScale(Vector3::Ones * 0.005f);

	uint i = 0;
	while (i < (uint)m_points->size())
	{
		const uint count = min((uint)256, (uint)m_points->size() - i);
		for (uint k = 0; k < count; ++k)
		{
			const Vector3 p0 = Vector3(m_points->points[k + i].x, 0.f, m_points->points[k + i].z);
			const Vector3 n = Vector3(m_norms->points[k + i].normal_x, 0.f, m_norms->points[k + i].normal_z);

			const float len = 0.2f;
			const float width = 0.002f;
			Transform tfm;
			tfm.pos = p0 + (n * len * 0.5f);
			tfm.scale = Vector3(width, width, len / 2.f);
			const Quaternion q(Vector3(0, 0, 1), n);
			tfm.rot = q;
			m_mats[k] = tfm.GetMatrix();
		}
		m_cube.RenderInstancing(renderer, (int)count, &m_mats[0], parentTm);
		i += 256;
	}
	return true;
}


// read capture file
// out: return scan data, if null update m_scan
bool cPointCloud::ReadCaptureFile(const string& fileName)
{
	// rplidar scan raw data
#pragma pack(push, 1)
	struct sl_lidar_response_measurement_node_hq_t
	{
		ushort angle_z_q14;
		uint dist_mm_q2;
		BYTE quality;
		BYTE flag;
	};
#pragma pack(pop)

	std::ifstream ifs(fileName, std::ios::binary);
	if (!ifs.is_open())
		return false; // error return

	char fmt[3] = { 0, 0, 0 };
	ifs.read(fmt, 3);
	if (('L' != fmt[0])
		|| ('I' != fmt[1])
		|| ('D' != fmt[2]))
	{
		// error file format
		return false;
	}

	int size = 0; // scan data size
	ifs.read((char*)&size, sizeof(size));
	if (size <= 0)
		return false; // error return

	const size_t totalByteSize = sizeof(sl_lidar_response_measurement_node_hq_t) * size;
	vector<BYTE> stream(totalByteSize);
	ifs.read((char*)&stream[0], totalByteSize);

	sl_lidar_response_measurement_node_hq_t* nodes =
		(sl_lidar_response_measurement_node_hq_t*)&stream[0];

	m_points->reserve(size);
	for (int i = 0; i < size; ++i)
	{
		sl_lidar_response_measurement_node_hq_t& src = nodes[i];
		const float angle = (src.angle_z_q14 * 90.f) / 16384.f;
		const float dist = src.dist_mm_q2 / 4.0f;
		const int quality = src.quality >> 2;

		if (quality < 40)
			continue;
		Vector2 pos;
		const float x = sin(ANGLE2RAD(angle)) * dist * 0.001f;
		const float y = cos(ANGLE2RAD(angle)) * dist * 0.001f;
		pcl::PointXYZRGB p(x, 0, y);
		const uint c = (255 << 16) | (255 << 8) | 255;
		p.rgb = *(float*)&c;
		m_points->push_back(p);
	}
	return true;
}


// set color
bool cPointCloud::SetColor(const graphic::cColor& color)
{
	const uint c = color.m_color;
	for (uint i = 0; i < (uint)m_points->size(); ++i)
		m_points->points[i].rgb = *(float*)&c;
	return true;
}


// return center point
Vector3 cPointCloud::GetCenter()
{
	Vector3 center;
	for (uint i = 0; i < (uint)m_points->size(); ++i)
		center += *(Vector3*)&m_points->points[i];
	center /= (uint)m_points->size();
	return center;
}


// is empty?
bool cPointCloud::IsEmpty()
{
	if (!m_points)
		return true;
	return m_points->empty();
}


// clear
void cPointCloud::Clear()
{
	if (!m_points)
		return;
	m_points->clear();
}
