#include "octree.h"

#include <glm/fwd.hpp>


/*
 * cell conner idx:
 *
 *             7---------------------6
 *            /|                    /|
 *           / |                   / |
 *			/  |                  /  |
 *         4---------------------5   |
 *		   |   |                 |   |
 *		   |   |                 |   |
 *		   |   |                 |   |
 *		   |   |                 |   |
 *		   |   3-----------------|---2
 *		   |  /                  |  / 
 *		   | /                   | /
 *		   |/                    |/
 *		   0---------------------1
 */
std::array<Vec3, 8> OctreeNodeBase::genPoints(const std::array<Vec3, 2>& diagPoint)
{
	std::array<Vec3, 8> points;
	for ( uint32_t z = 0; z < 2; ++z )
	{
		for ( uint32_t y = 0; y < 2; ++y )
		{
			for ( uint32_t x = 0; x < 2; ++x )
			{
				uint32_t idx = x + y * 2 + z * 4;
				points[idx].x = diagPoint[x].x;
				points[idx].y = diagPoint[1 - y].y;
				points[idx].z = diagPoint[z].z;
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

std::unique_ptr<OctreeNodeBase> OctreeNodeCreator::genNode(const std::array<Vec3, 2>& diagPoint)
{
	
}

