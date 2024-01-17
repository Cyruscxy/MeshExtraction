#include "octree.h"

#include "marching_cubes.h"

namespace 
{
	// grid edges with axis order
	constexpr std::array<std::array<uint8_t, 2>, 12> gridEdges{
		{
			// edges parallel to x axis
			{0, 1}, {2, 3}, {4, 5}, {6, 7},

			// edges parallel to y axis
			{0, 2}, {1, 3}, {4, 6}, {5, 7},

			// edges parallel to z axis
			{0, 4}, {1, 5}, {2, 6}, {3, 7}
		}
	};

	constexpr std::array<std::array<uint8_t, 3>, 8> gridToGridTable{
		{
			{1, 2, 4}, {0, 3, 5}, {0, 3, 6}, {1, 2, 7},
			{0, 5, 6}, {1, 4, 7}, {2, 4, 7}, {3, 5, 6}
		}
	};
}

/*
 * cell conner idx:
 *
 *             6----------g----------7(h)
 *            /|         /|         /|
 *           / |        e----------f |
 *			/  |        | |       /| |
 *         4---------------------5 | |
 *		   |   |        | |      | | |
 *		   |   |        | c------|-|-d
 *		   |   |        |/       | |/|
 *		   |   |        a--------|-b |
 *		   |   |                 |   |
 *         |   |                 |   |
 *		   |   |                 |   |
 *		   |   2-----------------|---3
 *		   |  /                  |  / 
 *		   | /                   | /
 *		   |/                    |/
 *		   0---------------------1
 */
std::array<Vec3, 8> OctreeNodeCreator::genPointsForGrid(const std::array<Vec3, 2>& diagPoint)
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
				points[idx].y = diagPoint[y].y;
				points[idx].z = diagPoint[z].z;
			}
		}
	}
	return points;
}

int OctreeNodeCreator::calculateCubeIdx(
	const std::array<Vec3, 8>& gridPoints, 
	std::array<Real, 8>& pointVal
) {
	int cubeIdx = 0;
	for (uint32_t i = 0; i < 8; ++i) pointVal[i] = m_surface(gridPoints[i]);
	for (int i = 0; i < 8; ++i)
	{
		if (pointVal[i] < 0.0f) cubeIdx = 1 << i;
	}
	return cubeIdx;
}

std::unique_ptr<OctreeNodeBase> OctreeNodeCreator::genNode(const std::array<Vec3, 2>& diagPoint)
{
	std::array<Vec3, 8> gridPoint = genPointsForGrid(diagPoint);

}

std::unique_ptr<OctreeNodeBase> OctreeNodeCreator::genRoot()
{
	constexpr std::array<Vec3, 2> rootSpace{ {{-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}} };

	std::array<Vec3, 8> rootGridPts = genPointsForGrid(rootSpace);
	std::array<Real, 8> ptVal;
	int cubeIdx = calculateCubeIdx(rootGridPts, ptVal);

	if (cubeIdx == 0) return nullptr;

	// calculate tree height
	std::vector<std::array<uint8_t, 2>> edges;
	for (uint8_t tail = 0; tail < 8; ++tail )
	{
		if (ptVal[tail] > 0.0f) continue;
		for ( auto head : gridToGridTable[tail] )
		{
			if (ptVal[head] < 0.0f) continue;
			edges.push_back({ tail, head });
		}
	}
}
