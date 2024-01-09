#include "octree.h"

#include <glm/fwd.hpp>


std::array<Vec3, 8> OctreeNodeBase::genPoints()
{
	std::array<Vec3, 8> points;
	for ( uint32_t z = 0; z < 2; ++z )
	{
		for ( uint32_t y = 0; y < 2; ++y )
		{
			for ( uint32_t x = 0; x < 2; ++x )
			{
				uint32_t idx = x + y * 2 + z * 4;
				points[idx].x = m_diagPoint[x].x;
				points[idx].y = m_diagPoint[1 - y].y;
				points[idx].z = m_diagPoint[z].z;
			}
		}
	}
	return points;
}

void OctreeNodeBase::calculateCubeIdx(const std::function<Real(const Vec3&)>& sdf)
{
	auto gridPoints = this->genPoints();
	for (uint32_t i = 0; i < 8; ++i) m_pointVal[i] = sdf(gridPoints[i]);
	for (int i = 0; i < 8; ++i)
	{
		if (m_pointVal[i] < 0.0f) m_cubeIdx |= 1 << i;
	}
}


