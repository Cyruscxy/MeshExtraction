#include "isosurface.h"
#include "grid.h"
#include "table.h"

namespace MarchingCubesConfig
{
	constexpr int resolution = 64;
	constexpr Real begin = -1.0f;
	constexpr Real end = 1.0f;
	constexpr Real gridSize = (end - begin) / resolution;
	constexpr Real isoLevel = 0.5f;
}

int MortonEncodeSpread(int num)
{
	num = (num | (num << 16)) & 0x030000FF;
	num = (num | (num << 8)) & 0x0300F00F;
	num = (num | (num << 4)) & 0x030C30C3;
	num = (num | (num << 2)) & 0x09249249;
	return num;
}

int MortonEncode(int x, int y, int z)
{
	x = MortonEncodeSpread(x);
	y = MortonEncodeSpread(y);
	z = MortonEncodeSpread(z);
	return x | (y << 1) | (z << 2);
}

void IsoSurface::polygonizeGrid()
{
	using namespace MarchingCubesConfig;
	int nEdges = resolution * (resolution + 1) * (resolution + 1) * 3;
	int nGridPoint = (resolution + 1) * (resolution + 1) * (resolution + 1);
	std::vector<int> edgeToVerticesTable(nEdges, -1);
	std::vector<Real> gridPointVal(nGridPoint);

	// calculate the scalar value of all grid point
	Vec3 point(begin, begin, begin);
	int idx = 0;
	for ( int z = 0; z < resolution + 1; ++z )
	{
		for ( int y = 0; y < resolution + 1; ++y )
		{
			for ( int x = 0; x < resolution + 1; ++x )
			{
				gridPointVal[idx] = m_scalarFunction(point);
				idx += 1;
				point.x += gridSize;
			}
			point.x = begin;
			point.y += gridSize;
		}
		point.y = begin;
		point.z += gridSize;
	}

	// calculate vertices resident on every edges
	for ( int dim = 0; dim < 3; ++dim ) // x, y, z dim
	{
		for ( int z = 0; z < resolution + 1; ++z )
		{
			for ( int y = 0; y < resolution + 1; ++y )
			{
				Vec3 tail(begin, begin + gridSize * y, begin + gridSize * z);
				for ( int x = 0; x < resolution; ++x )
				{
					Vec3 head(tail.x + gridSize, tail.y, tail.z);

				}
			}
		}
	}
	
}

